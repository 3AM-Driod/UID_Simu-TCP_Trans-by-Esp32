#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

// 定义UART配置参数
#define UART_PORT_NUM         UART_NUM_1      // 使用UART1
#define UART_BAUD_RATE        115200          // 波特率115200
#define UART_RX_PIN           4               // RX引脚GPIO4
#define UART_TX_PIN           5               // TX引脚GPIO5
#define UART_RX_BUFFER_SIZE   1024            // 接收缓冲区大小
#define UART_TX_BUFFER_SIZE   1024            // 发送缓冲区大小

static const char *TAG = "UART_EXAMPLE";

// 鼠标数据包结构
typedef struct {
    uint8_t header[3];    // 固定包头 57 AB 02
    uint8_t button;       // 按钮状态
    int8_t x_move;       // X轴位移（有符号）
    int8_t y_move;       // Y轴位移（有符号） 
    int8_t wheel;        // 滚轮状态（有符号）
} __attribute__((packed)) mouse_packet_t;

/**
 * @brief 初始化UART配置
 */
void uart_init(void)
{
    // 1. 配置UART参数
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,           // 波特率
        .data_bits = UART_DATA_8_BITS,        // 8位数据位
        .parity = UART_PARITY_DISABLE,        // 无奇偶校验
        .stop_bits = UART_STOP_BITS_1,        // 1位停止位
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // 无硬件流控
        .source_clk = UART_SCLK_DEFAULT,      // 默认时钟源
    };
    
    // 应用UART参数配置
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    
    // 2. 设置UART引脚
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, 
                                UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    // 3. 安装UART驱动程序
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, 
                                      UART_RX_BUFFER_SIZE, 
                                      UART_TX_BUFFER_SIZE, 
                                      0, NULL, 0));
    
    ESP_LOGI(TAG, "UART初始化完成: 端口=%d, TX引脚=GPIO%d, RX引脚=GPIO%d", 
             UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN);
}

/**
 * @brief 解析鼠标数据包
 * @param data 原始数据
 * @param length 数据长度
 */
void parse_mouse_data(uint8_t* data, int length, int sampling_rate)
{
    // 检查数据长度是否是7的倍数
    if (length % 7 != 0) {
        ESP_LOGW(TAG, "数据长度%d不是7的倍数，可能数据不完整", length);
        return;
    }
    
    // 参数校验和设置默认值
    if (sampling_rate <= 0) {
        sampling_rate = 1; // 默认处理所有数据包
        ESP_LOGW(TAG, "采样率参数无效，已设置为默认值: %d", sampling_rate);
    }
    
    int packet_count = length / 7;
/*    int processed_packets = 0; // 实际处理的包数量
    int skipped_packets = 0;   // 跳过的包数量*/
        
    for (int i = 0; i < packet_count; i++) {
        // 采样率控制：只处理符合采样条件的包
        if (i % sampling_rate != 0) {
/*            skipped_packets++;
*/            continue; // 跳过不符合采样率的包
        }
        
        uint8_t* packet_data = data + i * 7;
        mouse_packet_t packet;
        memcpy(&packet, packet_data, sizeof(mouse_packet_t));
        
        // 验证包头
        if (packet.header[0] != 0x57 || packet.header[1] != 0xAB || packet.header[2] != 0x02) {
            ESP_LOGE(TAG, "无效的数据包头[包%d]: %02X %02X %02X", 
                    i, packet.header[0], packet.header[1], packet.header[2]);
            continue;
        }
        
        // 解析按钮状态
        const char* button_str = "无按键";
        if (packet.button == 0x01) {
            button_str = "左键按下";
        } else if (packet.button == 0x02) {
            button_str = "右键按下";
        }
        
        // 生成事件描述
        char event_desc[100] = {0};
        
        if (packet.button != 0x00) {
            snprintf(event_desc, sizeof(event_desc), "按键事件: %s", button_str);
        } else if (packet.wheel != 0) {
            snprintf(event_desc, sizeof(event_desc), "滚轮滚动: %s%d", 
                    packet.wheel > 0 ? "向上" : "向下", abs(packet.wheel));
        } else if (packet.x_move != 0 || packet.y_move != 0) {
            snprintf(event_desc, sizeof(event_desc), "鼠标移动: X=%s%d, Y=%s%d", 
                    packet.x_move > 0 ? "+" : "", packet.x_move,
                    packet.y_move > 0 ? "+" : "", packet.y_move);
        } else {
            snprintf(event_desc, sizeof(event_desc), "无事件");
        }
        
        ESP_LOGI(TAG, "鼠标包[%d/%d] - %s (采样率:1/%d)", 
                i + 1, packet_count, event_desc, sampling_rate);
        
/*        processed_packets++;
*/        
    }
    
/*    ESP_LOGI(TAG, "数据解析完成: 处理%d包, 跳过%d包, 处理比例=%.1f%%", 
            processed_packets, skipped_packets, 
            (float)processed_packets / packet_count * 100);*/
}
void uart_receive_task(void *pvParameters)
{
    uint8_t *data = (uint8_t *)malloc(UART_RX_BUFFER_SIZE + 1);
    
    ESP_LOGI(TAG, "开始监听CH9350鼠标数据...");
    
    while (1) {
        int length = 0;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_PORT_NUM, (size_t*)&length));
        
        if (length > 0) {
            int bytes_read = uart_read_bytes(UART_PORT_NUM, data, length, pdMS_TO_TICKS(100));
            if (bytes_read > 0) {
                // 十六进制显示
                char hex_buffer[bytes_read * 3 + 1];
                for (int i = 0; i < bytes_read; i++) {
                    sprintf(hex_buffer + i * 3, "%02X ", data[i]);
                }
/*                ESP_LOGI(TAG, "收到%d字节: %s", bytes_read, hex_buffer);
*/                
                // 解析鼠标数据
                parse_mouse_data(data, bytes_read,10);
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    free(data);
}

/**
 * @brief 应用程序入口点
 */
void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-S3 UART通信示例启动");
    
    // 初始化UART
    uart_init();
    
    // 创建接收任务
    xTaskCreate(uart_receive_task, "uart_receive_task", 20480, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "UART任务创建完成，系统开始运行");
}