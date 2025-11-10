#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/uart.h"
#include "driver/gpio.h"
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
#include "tinyusb.h"
#include "tusb.h"
#include "class/hid/hid_device.h"

// 配置信息
#define WIFI_SSID      "密码8个8"
#define WIFI_PASS      "1055480154"
#define SERVER_IP      "192.168.31.220"
#define SERVER_PORT    8080

// UART配置参数
#define UART_PORT_NUM         UART_NUM_1
#define UART_BAUD_RATE        115200
#define UART_RX_PIN           4
#define UART_TX_PIN           5
#define UART_RX_BUFFER_SIZE   1024
#define UART_TX_BUFFER_SIZE   1024

static const char *TAG = "HID_TCP_UART_Client";

// WiFi连接状态
static EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

// 鼠标控制变量 - TCP和UART数据融合
static int16_t current_x = 0, current_y = 0;
static int8_t current_wheel_v = 0, current_wheel_h = 0;
static uint8_t current_buttons = 0;

// UART鼠标数据
static int16_t uart_x = 0, uart_y = 0;
static int8_t uart_wheel_v = 0;
static uint8_t uart_buttons = 0;

static TaskHandle_t mouse_task_handle = NULL;

// 互斥锁保护共享数据
static SemaphoreHandle_t mouse_data_mutex;

// ============================ HID鼠标部分 ============================

static const uint8_t hid_report_descriptor[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x02,        // Usage (Mouse)
    0xA1, 0x01,        // Collection (Application)
    
    // 按钮字节1 (8个按钮位)
    0x05, 0x09,        // Usage Page (Button)
    0x19, 0x01,        // Usage Minimum (1)
    0x29, 0x08,        // Usage Maximum (8)
    0x15, 0x00,        // Logical Minimum (0)
    0x25, 0x01,        // Logical Maximum (1)
    0x95, 0x08,        // Report Count (8)
    0x75, 0x01,        // Report Size (1)
    0x81, 0x02,        // Input (Data,Var,Abs)
    
    // 按钮字节2 (保留位/扩展按钮)
    0x95, 0x08,        // Report Count (8)
    0x75, 0x01,        // Report Size (1)
    0x81, 0x03,        // Input (Const,Var,Abs)
    
    // X位移 (-127 到 +127)
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x30,        // Usage (X)
    0x15, 0x81,        // Logical Minimum (-127)
    0x25, 0x7F,        // Logical Maximum (127)
    0x75, 0x08,        // Report Size (8)
    0x95, 0x01,        // Report Count (1)
    0x81, 0x06,        // Input (Data,Var,Rel)
    
    // X方向向量 (-1, 0)
    0x09, 0x3A,        // Usage (X Tilt)
    0x15, 0xFF,        // Logical Minimum (-1)
    0x25, 0x00,        // Logical Maximum (0)
    0x75, 0x08,        // Report Size (8)
    0x95, 0x01,        // Report Count (1)
    0x81, 0x06,        // Input (Data,Var,Rel)
    
    // Y位移 (-127 到 +127)
    0x09, 0x31,        // Usage (Y)
    0x15, 0x81,        // Logical Minimum (-127)
    0x25, 0x7F,        // Logical Maximum (127)
    0x75, 0x08,        // Report Size (8)
    0x95, 0x01,        // Report Count (1)
    0x81, 0x06,        // Input (Data,Var,Rel)
    
    // Y方向向量 (-1, 0)
    0x09, 0x3B,        // Usage (Y Tilt)
    0x15, 0xFF,        // Logical Minimum (-1)
    0x25, 0x00,        // Logical Maximum (0)
    0x75, 0x08,        // Report Size (8)
    0x95, 0x01,        // Report Count (1)
    0x81, 0x06,        // Input (Data,Var,Rel)
    
    // 垂直滚轮
    0x09, 0x38,        // Usage (Wheel)
    0x15, 0x81,        // Logical Minimum (-127)
    0x25, 0x7F,        // Logical Maximum (127)
    0x75, 0x08,        // Report Size (8)
    0x95, 0x01,        // Report Count (1)
    0x81, 0x06,        // Input (Data,Var,Rel)
    
    // 水平滚轮
    0x05, 0x0C,        // Usage Page (Consumer)
    0x0A, 0x38, 0x02,  // Usage (AC Pan)
    0x15, 0x81,        // Logical Minimum (-127)
    0x25, 0x7F,        // Logical Maximum (127)
    0x75, 0x08,        // Report Size (8)
    0x95, 0x01,        // Report Count (1)
    0x81, 0x06,        // Input (Data,Var,Rel)
    
    0xC0               // End Collection
};

static const tusb_desc_device_t desc_dev = {
    .bLength            = 0x12,
    .bDescriptorType    = 0x01,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = 0x00,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = 64,
    .idVendor           = 0x046D,
    .idProduct          = 0xC088,
    .bcdDevice          = 0x2703,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01
};

static const char* desc_str[] = {
    "\x09\x04",
    "Logitech",
    "G502 HERO SE",
    "910-005631"
};

typedef struct __attribute__((packed)) {
    uint8_t buttons;
    uint8_t buttons2;
    int8_t x;
    int8_t x_d;
    int8_t y;
    int8_t y_d;
    int8_t wheel_v;
    int8_t wheel_h;
} mouse_report_t;

#define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN)
static const uint8_t desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, CONFIG_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    TUD_HID_DESCRIPTOR(0, 4, false, sizeof(hid_report_descriptor), 0x81, 8, 1)
};

// HID回调函数
uint8_t const * tud_hid_descriptor_report_cb(uint8_t instance) {
    return hid_report_descriptor;
}

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, 
                               hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen) {
    return 0;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                          hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize) {
}

bool tud_hid_set_idle_cb(uint8_t instance, uint8_t idle_rate) {
    return true;
}

void tud_hid_set_protocol_cb(uint8_t instance, uint8_t protocol) {
    ESP_LOGI(TAG, "Protocol Changed: %s", protocol == HID_PROTOCOL_BOOT ? "BOOT" : "REPORT");
}

void tud_hid_report_complete_cb(uint8_t instance, uint8_t const* report, uint16_t len) {
}

static void usb_event_cb(tinyusb_event_t *event, void *arg) {
    switch (event->id) {
        case TINYUSB_EVENT_ATTACHED:
            ESP_LOGI(TAG, "USB Connected to Host");
            break;
        case TINYUSB_EVENT_DETACHED:
            ESP_LOGI(TAG, "USB Disconnected from Host");
            break;
        default:
            break;
    }
}

static int8_t calculate_direction(int8_t value) {
    if (value < 0) return -1;
    return 0;
}

// 发送鼠标报告函数
bool send_mouse_report_ex(int8_t x, int8_t y, int8_t wheel_v, int8_t wheel_h, uint8_t buttons) {
    if (!tud_hid_ready()) {
        return false;
    }

    mouse_report_t report = {
        .buttons = buttons & 0xFF,
        .buttons2 = 0x00,
        .x = x,
        .x_d = calculate_direction(x),
        .y = y,
        .y_d = calculate_direction(y),
        .wheel_v = wheel_v,
        .wheel_h = wheel_h
    };
    
    bool result = tud_hid_report(0, &report, sizeof(report));
    return result;
}

// ============================ UART鼠标数据解析部分 ============================

// 鼠标数据包结构
typedef struct __attribute__((packed)) {
    uint8_t header[3];    // 固定包头 57 AB 02
    uint8_t button;       // 按钮状态
    int8_t x_move;        // X轴位移（有符号）
    int8_t y_move;        // Y轴位移（有符号） 
    int8_t wheel;         // 滚轮状态（有符号）
} mouse_packet_t;

/**
 * @brief 初始化UART配置
 */
static void uart_init(void) {
    // 配置UART参数
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // 应用UART参数配置
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    
    // 设置UART引脚
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, 
                                UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    // 安装UART驱动程序
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, 
                                      UART_RX_BUFFER_SIZE, 
                                      UART_TX_BUFFER_SIZE, 
                                      0, NULL, 0));
    
    ESP_LOGI(TAG, "UART初始化完成: 端口=%d, TX引脚=GPIO%d, RX引脚=GPIO%d", 
             UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN);
}

/**
 * @brief 解析鼠标数据包并更新UART鼠标数据
 */
static void parse_uart_mouse_data(uint8_t* data, int length) {
    // 检查数据长度是否是7的倍数
    if (length % 7 != 0) {
        ESP_LOGW(TAG, "UART数据长度%d不是7的倍数", length);
        return;
    }
    
    int packet_count = length / 7;
    //int processed_count = 0;
    
    for (int i = 0; i < packet_count; i++) {

        uint8_t* packet_data = data + i * 7;
        mouse_packet_t packet;
        memcpy(&packet, packet_data, sizeof(mouse_packet_t));
        
        // 验证包头
        if (packet.header[0] != 0x57 || packet.header[1] != 0xAB || packet.header[2] != 0x02) {
            continue;
        }
        
        // 使用互斥锁保护共享数据
        if (xSemaphoreTake(mouse_data_mutex, portMAX_DELAY) == pdTRUE) {
            // UART鼠标数据
            uart_x += packet.x_move;
            uart_y += packet.y_move;
            uart_wheel_v += packet.wheel;
            uart_buttons = packet.button;
            uart_x = (uart_x > 127) ? 127 : (uart_x < -127) ? -127 : uart_x;
            uart_y = (uart_y > 127) ? 127 : (uart_y < -127) ? -127 : uart_y;        
            xSemaphoreGive(mouse_data_mutex);

            //processed_count++;
        }
        /*ESP_LOGI(TAG, "🎯 鼠标报告: X=%d, Y=%d", 
                             uart_x, uart_y );*/
/*        // 调试信息（可选）
        if (processed_count % 10 == 0) { // 每10个包输出一次日志
            ESP_LOGI(TAG, "UART鼠标包: 按钮=0x%02X, X=%d, Y=%d, 滚轮=%d", 
                     packet.button, packet.x_move, packet.y_move, packet.wheel);
        }*/
    }
    
}

// UART接收任务
static void uart_receive_task(void *pvParameters) {
    uint8_t *data = (uint8_t *)malloc(UART_RX_BUFFER_SIZE + 1);
    
    ESP_LOGI(TAG, "开始监听CH9350鼠标UART数据...");
    
    while (1) {
        int length = 0;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_PORT_NUM, (size_t*)&length));
        
        if (length > 0) {
            int bytes_read = uart_read_bytes(UART_PORT_NUM, data, length, pdMS_TO_TICKS(100));
            if (bytes_read > 0) {
                // 解析UART鼠标数据
                parse_uart_mouse_data(data, bytes_read);
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    free(data);
}

// ============================ TCP客户端部分 ============================

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                ESP_LOGI(TAG, "WiFi站点模式启动");
                esp_wifi_connect();
                break;
            case WIFI_EVENT_STA_CONNECTED:
                ESP_LOGI(TAG, "已连接到AP");
                break;
            case WIFI_EVENT_STA_DISCONNECTED: {
                wifi_event_sta_disconnected_t* event = (wifi_event_sta_disconnected_t*) event_data;
                ESP_LOGW(TAG, "WiFi连接断开, 原因: %d", event->reason);
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

static void ip_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data) {
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "获取到IP地址: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void) {
    ESP_LOGI(TAG, "初始化WiFi...");
    
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 优化WiFi配置
    wifi_country_t country = {
        .cc = "CN", .schan = 1, .nchan = 13, .max_tx_power = 20, .policy = WIFI_COUNTRY_POLICY_AUTO
    };
    esp_wifi_set_country(&country);
    esp_wifi_set_ps(WIFI_PS_NONE);

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .scan_method = WIFI_FAST_SCAN,
            .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
            .threshold = {.rssi = -127, .authmode = WIFI_AUTH_WPA2_PSK},
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

// TCP指令解析逻辑
static void parse_tcp_mouse_command(const char* command) {
    // 去除换行符
    char cleaned_cmd[128];
    strncpy(cleaned_cmd, command, sizeof(cleaned_cmd) - 1);
    cleaned_cmd[sizeof(cleaned_cmd) - 1] = '\0';
    
    char *newline = strchr(cleaned_cmd, '\n');
    if (newline) *newline = '\0';
    
    // 解析格式: "buttons, x, y"
    int buttons_val = 0, x_val = 0, y_val = 0;
    int parsed_count = sscanf(cleaned_cmd, "%d, %d, %d", &buttons_val, &x_val, &y_val);
    
    if (parsed_count == 3) {
        if (xSemaphoreTake(mouse_data_mutex, portMAX_DELAY) == pdTRUE) {
            // 更新TCP鼠标数据
            current_buttons = (uint8_t)(buttons_val & 0xFF);
            current_x = (x_val);
            current_y = (y_val);
            
            xSemaphoreGive(mouse_data_mutex);
            
            ESP_LOGI(TAG, "✅ TCP解析成功: 按钮=0x%02X, X=%d, Y=%d", 
                    current_buttons, current_x, current_y);
        }
    } else {
        ESP_LOGW(TAG, "⚠️ TCP指令格式错误: %s", cleaned_cmd);
    }
}

// TCP客户端任务
static void tcp_client_task(void *pvParameters) {
    static char rx_buffer[128];
    static char tx_buffer[256];
    
    ESP_LOGI(TAG, "等待WiFi连接...");
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);

    while (1) {
        struct sockaddr_in server_addr;
        int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) {
            ESP_LOGE(TAG, "创建socket失败");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        // 设置socket参数
        int tcp_no_delay = 1;
        setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &tcp_no_delay, sizeof(tcp_no_delay));
        
        struct timeval timeout = {1, 0};
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(SERVER_PORT);
        inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr);

        ESP_LOGI(TAG, "尝试连接服务器 %s:%d", SERVER_IP, SERVER_PORT);
        
        if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) != 0) {
            ESP_LOGE(TAG, "连接服务器失败");
            close(sock);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        ESP_LOGI(TAG, "成功连接到服务器!");
        
        // 发送连接消息
        int64_t connect_time = esp_timer_get_time();
        snprintf(tx_buffer, sizeof(tx_buffer), "HID_MOUSE_CONNECTED|%lld", connect_time);
        send(sock, tx_buffer, strlen(tx_buffer), 0);

        while (1) {
            // 接收数据
            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            if (len < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    vTaskDelay(10 / portTICK_PERIOD_MS);
                    continue;
                }
                ESP_LOGE(TAG, "接收数据失败");
                break;
            } else if (len == 0) {
                ESP_LOGI(TAG, "服务器断开连接");
                break;
            } else {
                rx_buffer[len] = '\0';
                ESP_LOGI(TAG, "收到TCP指令: %s", rx_buffer);
                
                // 解析并执行TCP鼠标指令
                parse_tcp_mouse_command(rx_buffer);
                
                // 发送响应
                snprintf(tx_buffer, sizeof(tx_buffer), "ACK|%s", rx_buffer);
                if (send(sock, tx_buffer, strlen(tx_buffer), 0) < 0) {
                    ESP_LOGE(TAG, "发送响应失败");
                    break;
                }
            }
        }
        
        close(sock);
        ESP_LOGI(TAG, "TCP连接关闭，准备重连...");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

// ============================ 鼠标数据融合控制部分 ============================

// 鼠标控制任务 - 融合UART和TCP数据
static void mouse_control_task(void *pvParameters) {
    ESP_LOGI(TAG, "鼠标控制任务启动 - UART+TCP数据融合");
    
    while (1) {
        if (tud_hid_ready()) {
            int8_t fused_x = 0, fused_y = 0;
            int8_t fused_wheel_v = 0;
            uint8_t fused_buttons = 0;
            
            // 获取互斥锁保护的数据
            if (xSemaphoreTake(mouse_data_mutex, portMAX_DELAY) == pdTRUE) {
                // 数据融合：TCP数据 + UART数据
                fused_x = current_x + uart_x;
                fused_y = current_y + uart_y;
                fused_wheel_v = current_wheel_v + uart_wheel_v;
                fused_buttons = current_buttons|uart_buttons;
                fused_x = (fused_x > 127) ? 127 : (fused_x < -127) ? -127 : fused_x;
                fused_y = (fused_y > 127) ? 127 : (fused_y < -127) ? -127 : fused_y;
                fused_wheel_v = (fused_wheel_v > 127) ? 127 : (fused_wheel_v < -127) ? -127 : fused_wheel_v;


                // 发送融合后的鼠标报告
                send_mouse_report_ex(fused_x, fused_y, fused_wheel_v, 0, fused_buttons);
                uart_x = uart_y = 0;  
                uart_wheel_v = 0;
                current_x = current_y = 0;      
                current_wheel_v = 0;        
                xSemaphoreGive(mouse_data_mutex);

/*                // 调试输出（可选）
                static int counter = 0;
                if (counter++ % 100 == 0) { // 每100次输出一次
                    ESP_LOGI(TAG, "🎯 数据融合: X=%d(TCP:%d+UART:%d), Y=%d(TCP:%d+UART:%d), 按钮=0x%02X", 
                            fused_x, current_x, uart_x, fused_y, current_y, uart_y, fused_buttons);
                }*/
            }
        }        
        vTaskDelay(10 / portTICK_PERIOD_MS); // 100Hz更新率
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "🚀 ESP32-S3 HID鼠标TCP+UART融合控制器启动");
    int sampling_rate = 10;
    // 创建互斥锁
    mouse_data_mutex = xSemaphoreCreateMutex();
    if (mouse_data_mutex == NULL) {
        ESP_LOGE(TAG, "创建互斥锁失败");
        return;
    }
    
    // 初始化NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 初始化USB HID
    ESP_LOGI(TAG, "初始化USB HID...");
    
    tinyusb_desc_config_t desc_config = {
        .device = &desc_dev,
        .qualifier = NULL,
        .string = desc_str,
        .string_count = 4,
        .full_speed_config = desc_configuration,
        .high_speed_config = NULL
    };
    
    tinyusb_phy_config_t phy_config = {
        .skip_setup = false,
        .self_powered = false,
        .vbus_monitor_io = -1
    };
    
    tinyusb_task_config_t task_config = {
        .size = 4096,
        .priority = 5,
        .xCoreID = 0
    };
    
    tinyusb_config_t config = {
        .port = TINYUSB_PORT_FULL_SPEED_0,
        .phy = phy_config,
        .task = task_config,
        .descriptor = desc_config,
        .event_cb = usb_event_cb,
        .event_arg = NULL
    };
    
    ret = tinyusb_driver_install(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TinyUSB驱动安装失败: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "TinyUSB驱动安装成功");

    // 初始化UART
    uart_init();
    
    // 初始化WiFi
    wifi_init_sta();
    
    // 创建鼠标控制任务（数据融合）
    xTaskCreate(mouse_control_task, "mouse_control", 4096, NULL, 8, &mouse_task_handle);
    
    // 创建UART接收任务
    xTaskCreate(uart_receive_task, "uart_receive", 20480, NULL, 7, NULL);
    
    // 创建TCP客户端任务
    xTaskCreate(tcp_client_task, "tcp_client", 8192, NULL, 6, NULL);

    ESP_LOGI(TAG, "系统初始化完成，等待USB连接和WiFi连接...");
    
    // 主循环 - 处理USB事件
    while (1) {
        tud_task();
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}