#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "tinyusb.h"
#include "tusb.h"
#include "class/hid/hid_device.h"

static const char *TAG = "HID_Mouse";

// 自定义8字节鼠标报告描述符（完全匹配目标设备格式）
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
    0x81, 0x03,        // Input (Const,Var,Abs) - 常量或保留
    
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

// 修正设备描述符
static const tusb_desc_device_t desc_dev = {
    .bLength            = 0x12,
    .bDescriptorType    = 0x01,
    .bcdUSB             = 0x0200,        // USB 2.0
    .bDeviceClass       = 0x00,         // 每个接口指定类
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = 64,
    .idVendor           = 0x046D,       // Espressif VID
    .idProduct          = 0xC088,       // 更改产品ID避免冲突
    .bcdDevice          = 0x2703,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01
};

// 字符串描述符
static const char* desc_str[] = {
    "\x09\x04",        // 语言ID
    "Logitech",                 // 制造商
    "G502 HERO SE",            // 产品名称
    "910-005631"                     // 序列号
};

// 匹配目标鼠标的8字节数据结构
typedef struct __attribute__((packed)) {
    uint8_t buttons;     // 按钮状态 (bit0:左键, bit1:右键, bit2:中键)
    uint8_t buttons2;    // 保留按钮/扩展按钮
    int8_t x;            // X位移 (-127 到 +127)
    int8_t x_d;          // X方向向量 (-1, 0)
    int8_t y;            // Y位移 (-127 到 +127)
    int8_t y_d;          // Y方向向量 (-1, 0)
    int8_t wheel_v;      // 垂直滚轮
    int8_t wheel_h;      // 水平滚轮
} mouse_report_t;

// 使用 TinyUSB 的配置描述符宏
#define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN)
static const uint8_t desc_configuration[] = {
    // 配置描述符
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, CONFIG_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    
    // HID 接口描述符 - 报告长度改为8字节，端点间隔1ms匹配目标设备
    TUD_HID_DESCRIPTOR(0, 4, false, sizeof(hid_report_descriptor), 0x81, 8, 1)
};

// HID 回调函数
uint8_t const * tud_hid_descriptor_report_cb(uint8_t instance)
{
    ESP_LOGI(TAG, "HID Report Descriptor Request");
    return hid_report_descriptor;
}

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, 
                               hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
    ESP_LOGD(TAG, "GET_REPORT Request");
    return 0;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                          hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
    ESP_LOGD(TAG, "SET_REPORT Request");
}

bool tud_hid_set_idle_cb(uint8_t instance, uint8_t idle_rate)
{
    ESP_LOGD(TAG, "SET_IDLE Request - Rate: %d", idle_rate);
    return true;
}

void tud_hid_set_protocol_cb(uint8_t instance, uint8_t protocol)
{
    ESP_LOGI(TAG, "Protocol Changed: %s", protocol == HID_PROTOCOL_BOOT ? "BOOT" : "REPORT");
}

// 报告发送完成回调
void tud_hid_report_complete_cb(uint8_t instance, uint8_t const* report, uint16_t len)
{
    ESP_LOGD(TAG, "Report Sent Successfully");
}

// USB 事件回调
static void usb_event_cb(tinyusb_event_t *event, void *arg)
{
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

// 计算方向向量（根据你的观察，x_d和y_d只包含-1和0）
static int8_t calculate_direction(int8_t value)
{
    if (value < 0) return -1;   // 负向移动对应1
    return 0;                  // 无移动为0
}

// 发送符合目标格式的鼠标报告
bool send_mouse_report_ex(int8_t x, int8_t y, int8_t wheel_v, int8_t wheel_h, uint8_t buttons)
{
    if (!tud_hid_ready()) {
        ESP_LOGW(TAG, "HID not ready");
        return false;
    }

    mouse_report_t report = {
        .buttons = buttons & 0xFF,        // 按钮状态
        .buttons2 = 0x00,                 // 保留按钮字节设为0（根据你的观察）
        .x = x,
        .x_d = calculate_direction(x),    // 计算X方向向量
        .y = y,
        .y_d = calculate_direction(y),    // 计算Y方向向量
        .wheel_v = wheel_v,
        .wheel_h = wheel_h
    };
    
    // 发送8字节报告
    bool result = tud_hid_report(0, &report, sizeof(report));
    
    if (result) {
        ESP_LOGD(TAG, "Report: BTN=0x%02X, BTN2=0x%02X, X=%d, Xd=%d, Y=%d, Yd=%d, WV=%d, WH=%d",
                report.buttons, report.buttons2, report.x, report.x_d, 
                report.y, report.y_d, report.wheel_v, report.wheel_h);
    } else {
        ESP_LOGE(TAG, "Failed to send HID report");
    }
    
    return result;
}

// 兼容原有函数的封装
bool send_mouse_report(int8_t x, int8_t y, int8_t wheel, uint8_t buttons)
{
    return send_mouse_report_ex(x, y, wheel, 0, buttons);
}

// 鼠标演示任务
static void mouse_demo_task(void *arg)
{
    ESP_LOGI(TAG, "Mouse Demo Task Started");
    
    // 等待 USB 设备枚举完成
    int timeout = 0;
    while (!tud_mounted()) {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (timeout++ > 100) { // 10秒超时
            ESP_LOGE(TAG, "USB Enumeration Timeout");
            return;
        }
    }
    
    ESP_LOGI(TAG, "USB Enumeration Completed");
    
    // 等待 HID 设备就绪
    while (!tud_hid_ready()) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    ESP_LOGI(TAG, "HID Device Ready - Starting Mouse Demo");
    ESP_LOGI(TAG, "Device Identity: VID=0x%04X,PID=0x%04X,Class=0x%02X,SubClass=0x%02X,Protocol=0x%02X,Speed=Full",
             0x046D, 0xC08B, 0x00, 0x00, 0x00);
    
    uint32_t counter = 0;
    uint32_t last_log_time = 0;
    
    while (1) {
        if (tud_mounted() && tud_hid_ready()) {
            int8_t move_x = 0, move_y = 0, wheel_v = 0, wheel_h = 0;
            uint8_t buttons = 0;
            
            // 模拟真实鼠标行为 - 方形移动
            switch (counter % 80) {
                case 0:  move_x = 10; break;    // 右移
                case 20: move_y = 10; break;    // 下移
                case 40: move_x = -10; break;   // 左移
                case 60: move_y = -10; break;   // 上移
            }
            

            // 使用新的报告函数发送数据
            if (send_mouse_report_ex(move_x, move_y, wheel_v, wheel_h, buttons)) {
                // 减少日志频率，每2秒记录一次
                uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
                if (current_time - last_log_time > 2000) {
                    ESP_LOGI(TAG, "Mouse active: X=%d, Y=%d, WheelV=%d, WheelH=%d", 
                            move_x, move_y, wheel_v, wheel_h);
                    last_log_time = current_time;
                }
            }
            
            counter++;
            
            // 如果是点击操作，需要在下一个周期释放按钮
            if (buttons != 0) {
                vTaskDelay(pdMS_TO_TICKS(10)); // 短暂延迟模拟点击持续时间
                send_mouse_report_ex(0, 0, 0, 0, 0); // 释放所有按钮
            }
        } else {
            ESP_LOGW(TAG, "USB/HID Not Ready");
            vTaskDelay(pdMS_TO_TICKS(500)); // 未就绪时延长等待时间
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz 更新率，匹配目标设备
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting ESP32-S3 HID Mouse");
    
    // 描述符配置
    tinyusb_desc_config_t desc_config = {
        .device = &desc_dev,
        .qualifier = NULL,
        .string = desc_str,
        .string_count = 4,
        .full_speed_config = desc_configuration,
        .high_speed_config = NULL
    };    
    // PHY 配置
    tinyusb_phy_config_t phy_config = {
        .skip_setup = false,
        .self_powered = false,
        .vbus_monitor_io = -1
    };    
    // 任务配置
    tinyusb_task_config_t task_config = {
        .size = 4096,
        .priority = 5,
        .xCoreID = 0
    };
    // TinyUSB 配置
    tinyusb_config_t config = {
        .port = TINYUSB_PORT_FULL_SPEED_0,
        .phy = phy_config,
        .task = task_config,
        .descriptor = desc_config,
        .event_cb = usb_event_cb,
        .event_arg = NULL
    };   
    // 安装 TinyUSB 驱动
    esp_err_t ret = tinyusb_driver_install(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TinyUSB Driver Installation Failed: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "TinyUSB Driver Installed Successfully");
    
    // 创建鼠标演示任务
    xTaskCreate(mouse_demo_task, "mouse_demo", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "HID Mouse Example Started");
    
    // 主循环
    while (1) {
        tud_task(); // 处理 USB 事件
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}