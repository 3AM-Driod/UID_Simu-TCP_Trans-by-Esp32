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

// 使用 TinyUSB 的标准鼠标报告描述符
static const uint8_t hid_report_descriptor[] = {
    TUD_HID_REPORT_DESC_MOUSE()
};

// 修正设备描述符
static const tusb_desc_device_t desc_dev = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,        // USB 2.0
    .bDeviceClass       = 0x00,         // 每个接口指定类
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = 64,
    .idVendor           = 0x046D,       // Espressif VID
    .idProduct          = 0xC088,       // 更改产品ID避免冲突
    .bcdDevice          = 0x0100,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01
};

// 字符串描述符
static const char* desc_str[] = {
    "\x09\x04",        // 语言ID
    "Logitech",                 // 制造商
    "GPW PRO",            // 产品名称
    "123456"                     // 序列号
};

// 使用 TinyUSB 的配置描述符宏
#define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN)
static const uint8_t desc_configuration[] = {
    // 配置描述符
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, CONFIG_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    
    // HID 接口描述符
    TUD_HID_DESCRIPTOR(0, 4, false, sizeof(hid_report_descriptor), 0x81, 8, 10)
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
    
    // 处理 LED 状态（对于键盘）
    if (report_type == HID_REPORT_TYPE_OUTPUT && bufsize > 0) {
        uint8_t leds = buffer[0];
        ESP_LOGI(TAG, "Keyboard LEDs - NumLock: %d, CapsLock: %d, ScrollLock: %d", 
                 (leds & 1) ? 1 : 0, (leds & 2) ? 1 : 0, (leds & 4) ? 1 : 0);
    }
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

// 发送鼠标报告
bool send_mouse_report(int8_t x, int8_t y, int8_t wheel, uint8_t buttons)
{
    // 使用 TinyUSB 的标准鼠标报告函数
    return tud_hid_mouse_report(0, buttons, x, y, wheel, 0);
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
    
    uint32_t counter = 0;
    
    while (1) {
        if (tud_mounted() && tud_hid_ready()) {
            int8_t move_x = 0, move_y = 0, wheel = 0;
            uint8_t buttons = 0;
            
            // 简单的演示模式
            switch (counter % 20) {
                case 0:  move_x = 5; break;    // 右
                case 5:  move_y = 5; break;    // 下
                case 10: move_x = -5; break;   // 左
                case 15: move_y = -5; break;   // 上
            }
            
/*            // 周期性点击
            if (counter % 40 == 0) {
                buttons = 1; // 左键点击
            }
            
            // 周期性滚轮
            if (counter % 60 == 0) {
                wheel = 5;
            } else if (counter % 60 == 30) {
                wheel = -5;
            }
            */
            if (send_mouse_report(move_x, move_y, wheel, buttons)) {
                ESP_LOGW(TAG, "Mouse Report: X=%d, Y=%d, Wheel=%d, Buttons=0x%02X", 
                         move_x, move_y, wheel, buttons);
            } else {
                ESP_LOGW(TAG, "Failed to Send Mouse Report");
            }
            
            counter++;
        } else {
            ESP_LOGW(TAG, "USB/HID Not Ready");
        }
        
        vTaskDelay(pdMS_TO_TICKS(20)); // 20Hz 更新率
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting ESP32-S3 HID Mouse Example");
    
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