#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

// 配置信息
#define WIFI_SSID      "密码8个8"
#define WIFI_PASS      "1055480154"
#define SERVER_IP      "192.168.31.220"
#define SERVER_PORT    8080

static const char *TAG = "ESP32客户端";

// WiFi连接状态
static EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

// 延迟测量和性能统计
static int64_t last_receive_time = 0;

// 性能统计结构体
typedef struct {
    uint32_t total_packets;
    uint32_t failed_packets;
    int64_t min_latency;
    int64_t max_latency;
    int64_t total_latency;
    uint32_t reconnect_count;
} perf_stats_t;

static perf_stats_t perf_stats = {0};
static int reconnect_backoff_ms = 1000;

// 函数声明 - 修复：添加缺失的函数声明
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
static void ip_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
static void wifi_init_sta(void);
static void tcp_client_task(void *pvParameters);

// 优化的LED控制函数
static inline void control_led(const char *command)
{
    if (strcmp(command, "switch_on") == 0) {
        ESP_LOGI(TAG, "✅ 收到指令 switch_on");
    } else if (strcmp(command, "switch_off") == 0) {
        ESP_LOGI(TAG, "🔴 收到指令 switch_off");
    } else if (strcmp(command, "status") == 0) {
        ESP_LOGI(TAG, "📊 状态查询: 系统运行正常");
    } else if (strncmp(command, "PING", 4) == 0) {
        last_receive_time = esp_timer_get_time();
        
        if (strlen(command) > 5) {
            int64_t client_send_time = atoll(command + 5);
            int64_t one_way_latency = (last_receive_time - client_send_time) / 2;
            perf_stats.total_latency += one_way_latency;
            perf_stats.total_packets++;
            
            if (one_way_latency < perf_stats.min_latency || perf_stats.min_latency == 0) {
                perf_stats.min_latency = one_way_latency;
            }
            if (one_way_latency > perf_stats.max_latency) {
                perf_stats.max_latency = one_way_latency;
            }
        }
    } else {
        ESP_LOGW(TAG, "⚠️ 未知指令: %s", command);
    }
}

// 优化的响应生成函数
static inline void generate_latency_response(const char *original_cmd, char *response, size_t response_size)
{
    int64_t current_time = esp_timer_get_time();
    
    if (strncmp(original_cmd, "PING", 4) == 0) {
        snprintf(response, response_size, "PONG|%lld|%lld", 
                 last_receive_time, current_time);
    } else {
        snprintf(response, response_size, "ACK|%lld|执行完成", current_time);
    }
}

// 设置TCP socket参数以减少延迟
static void optimize_socket_params(int sock)
{
    int tcp_no_delay = 1;
    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &tcp_no_delay, sizeof(tcp_no_delay));
    
    int keepalive = 1;
    int keepidle = 5;
    int keepintvl = 1;
    int keepcnt = 3;
    
    setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive));
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepidle, sizeof(keepidle));
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepintvl, sizeof(keepintvl));
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepcnt, sizeof(keepcnt));
    
    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    
    int buf_size = 2048;
    setsockopt(sock, SOL_SOCKET, SO_SNDBUF, &buf_size, sizeof(buf_size));
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &buf_size, sizeof(buf_size));
}

// WiFi事件处理函数 - 修复：添加完整的函数实现
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                ESP_LOGI(TAG, "📡 WiFi站点模式启动");
                ESP_LOGI(TAG, "🔄 正在连接到WiFi: %s", WIFI_SSID);
                esp_wifi_connect();
                break;
                
            case WIFI_EVENT_STA_CONNECTED:
                ESP_LOGI(TAG, "🔗 已连接到AP");
                break;
                
            case WIFI_EVENT_STA_DISCONNECTED: {
                wifi_event_sta_disconnected_t* event = (wifi_event_sta_disconnected_t*) event_data;
                ESP_LOGW(TAG, "❌ WiFi连接断开, 原因: %d", event->reason);
                ESP_LOGI(TAG, "🔄 3秒后尝试重新连接...");
                vTaskDelay(3000 / portTICK_PERIOD_MS);
                esp_wifi_connect();
                xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
                break;
            }
            
            default:
                break;
        }
    }
}

// IP事件处理函数 - 修复：添加完整的函数实现
static void ip_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "🎉 WiFi连接成功!");
        ESP_LOGI(TAG, "📡 获取到IP地址: " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "🌐 子网掩码: " IPSTR, IP2STR(&event->ip_info.netmask));
        ESP_LOGI(TAG, "🚪 网关: " IPSTR, IP2STR(&event->ip_info.gw));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// 优化的WiFi配置
static void wifi_init_sta(void)
{
    ESP_LOGI(TAG, "🔄 初始化WiFi...");
    
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 设置国家代码以提升信号强度
    wifi_country_t country = {
        .cc = "CN",
        .schan = 1,
        .nchan = 13,
        .max_tx_power = 20,
        .policy = WIFI_COUNTRY_POLICY_AUTO
    };
    esp_wifi_set_country(&country);
    
    // 禁用节能模式以降低延迟
    esp_wifi_set_ps(WIFI_PS_NONE);

    // 修复：现在这些函数已经正确定义，可以正常注册
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL));

    // 优化的WiFi配置
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .scan_method = WIFI_FAST_SCAN,
            .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
            .threshold = {
                .rssi = -127,
                .authmode = WIFI_AUTH_WPA2_PSK,
            },
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    
    // 设置最大发射功率
    esp_wifi_set_max_tx_power(20);
    
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "✅ WiFi初始化完成");
}

// 智能重连机制
static void smart_reconnect(int *sock, int *retry_count)
{
    if (*sock >= 0) {
        close(*sock);
        *sock = -1;
    }
    
    // 指数退避算法
    if (*retry_count > 0) {
        reconnect_backoff_ms = (reconnect_backoff_ms * 2) > 10000 ? 10000 : (reconnect_backoff_ms * 2);
    } else {
        reconnect_backoff_ms = 1000;
    }
    
    perf_stats.reconnect_count++;
    ESP_LOGI(TAG, "🔄 %dms后尝试第%d次重连...", reconnect_backoff_ms, *retry_count + 1);
    vTaskDelay(reconnect_backoff_ms / portTICK_PERIOD_MS);
    (*retry_count)++;
}

// 连接健康检查
static bool connection_health_check(int sock)
{
    int64_t current_time = esp_timer_get_time();
    char heartbeat_msg[32];
    snprintf(heartbeat_msg, sizeof(heartbeat_msg), "HB|%lld", current_time);
    
    if (send(sock, heartbeat_msg, strlen(heartbeat_msg), MSG_DONTWAIT) < 0) {
        return false;
    }
    return true;
}



// 优化的TCP客户端任务
static void tcp_client_task(void *pvParameters)
{
    static char rx_buffer[128];
    static char tx_buffer[256];
    int retry_count = 0;
    
    ESP_LOGI(TAG, "⏳ 等待WiFi连接...");
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);

    while (1) {
        struct sockaddr_in server_addr;
        int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) {
            ESP_LOGE(TAG, "❌ 创建socket失败");
            smart_reconnect(&sock, &retry_count);
            continue;
        }

        // 优化socket参数
        optimize_socket_params(sock);

        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(SERVER_PORT);
        inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr);

        ESP_LOGI(TAG, "🔌 连接服务器 %s:%d", SERVER_IP, SERVER_PORT);
        
        if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) != 0) {
            ESP_LOGE(TAG, "❌ 连接服务器失败");
            smart_reconnect(&sock, &retry_count);
            continue;
        }

        ESP_LOGI(TAG, "✅ 成功连接到服务器!");
        retry_count = 0;
        reconnect_backoff_ms = 1000;

        // 发送连接消息
        int64_t connect_time = esp_timer_get_time();
        snprintf(tx_buffer, sizeof(tx_buffer), "ESP32-S3已连接|%lld", connect_time);
        send(sock, tx_buffer, strlen(tx_buffer), 0);

        int64_t last_health_check = esp_timer_get_time();
        bool connection_healthy = true;

        while (connection_healthy) {
            // 定期健康检查（每10秒）
            int64_t current_time = esp_timer_get_time();
            if (current_time - last_health_check > 10000000) {
                connection_healthy = connection_health_check(sock);
                last_health_check = current_time;
            }

            // 接收数据
            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            if (len < 0) {
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    ESP_LOGE(TAG, "❌ 接收数据失败");
                    break;
                }
                vTaskDelay(10 / portTICK_PERIOD_MS);
                continue;
            } else if (len == 0) {
                ESP_LOGI(TAG, "🔌 服务器断开连接");
                break;
            } else {
                rx_buffer[len] = '\0';
                ESP_LOGI(TAG, "📥 收到指令: %s", rx_buffer);
                
                // 处理指令
                control_led(rx_buffer);
                
                // 生成响应
                generate_latency_response(rx_buffer, tx_buffer, sizeof(tx_buffer));
                if (send(sock, tx_buffer, strlen(tx_buffer), 0) < 0) {
                    ESP_LOGE(TAG, "❌ 发送响应失败");
                    break;
                }
                ESP_LOGI(TAG, "📤 发送响应: %s", tx_buffer);
            }
        }
        
        close(sock);
        ESP_LOGI(TAG, "🔄 连接关闭，准备重连...");
        smart_reconnect(&sock, &retry_count);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "🚀 ESP32-S3 TCP客户端启动");
    
    // 初始化NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 初始化WiFi
    wifi_init_sta();
    
    // 启动TCP客户端任务
    xTaskCreate(tcp_client_task, "tcp_client", 8192, NULL, 8, NULL);

    ESP_LOGI(TAG, "🎯 系统初始化完成，开始运行...");
}