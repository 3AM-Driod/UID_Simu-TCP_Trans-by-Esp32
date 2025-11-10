#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "usb/usb_host.h"
#include "usb/usb_types_ch9.h"
#include "usb/usb_types_stack.h"
#include "usb/usb_helpers.h"
#define USB_B_DESCRIPTOR_TYPE_HID 0x21
static const char *TAG = "USB_MOUSE";
static void parse_configuration_descriptors(const usb_config_desc_t *config_desc);
static void print_interface_class_info(uint8_t class, uint8_t subclass, uint8_t protocol);
static void print_endpoint_info(uint8_t addr, uint8_t attr);
// 设备管理结构体
typedef struct {
    usb_device_handle_t device;
    usb_host_client_handle_t client;
    usb_transfer_t *transfer;
    uint8_t interface_number;
    uint8_t endpoint_address;
    uint16_t max_packet_size;
    bool transfer_active;
    bool device_connected;
    SemaphoreHandle_t mutex;
} mouse_device_t;

static mouse_device_t mouse_dev = {0};

// Mouse data structure
typedef struct __attribute__((packed)) {
    uint8_t button1;
    uint8_t button2;
    int8_t x;
    int8_t x_d;
    int8_t y;
    int8_t y_d;
    int8_t wheel1;
    int8_t wheel2;
} mouse_data_t;

// 传输完成回调函数
static void transfer_cb(usb_transfer_t *transfer)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    ESP_LOGD(TAG, "Status: %d, Actual bytes: %d, Data length: %d", 
             transfer->status, transfer->actual_num_bytes, transfer->num_bytes);
    
    if (xSemaphoreTakeFromISR(mouse_dev.mutex, &xHigherPriorityTaskWoken) == pdTRUE) {
        if (transfer->status == USB_TRANSFER_STATUS_COMPLETED) {
            ESP_LOGD(TAG, "Transfer completed successfully");
            
            if (transfer->actual_num_bytes >= 3) {
                uint8_t button1 = transfer->data_buffer[0];
                uint8_t button2 = transfer->data_buffer[1];
                int8_t x = (int8_t)transfer->data_buffer[2];
                int8_t x_d = (int8_t)transfer->data_buffer[3];
                int8_t y = (int8_t)transfer->data_buffer[4];
                int8_t y_d = (int8_t)transfer->data_buffer[5];
                int8_t wheel1 = (int8_t)transfer->data_buffer[6];
                int8_t wheel2 = (transfer->actual_num_bytes >= 7) ? (int8_t)transfer->data_buffer[7] : 0;
                
                ESP_LOGI(TAG, "Mouse Data: button1=0x%02X,button2=0x%02X, X=%d, x_d=%d, Y=%d, y_d=%d, wheel1=%d, wheel2=%d", 
                        button1,button2, x, x_d, y,y_d,wheel1,wheel2);
            } else {
                ESP_LOGW(TAG, "Short transfer: only %d bytes received", transfer->actual_num_bytes);
            }
            
            // 重新提交传输
            esp_err_t err = usb_host_transfer_submit(transfer);
            if (err == ESP_OK) {
                mouse_dev.transfer_active = true;
                ESP_LOGD(TAG, "Transfer resubmitted successfully");
            } else {
                mouse_dev.transfer_active = false;
                ESP_LOGE(TAG, "Failed to resubmit transfer: %s", esp_err_to_name(err));
            }
        } else {
            mouse_dev.transfer_active = false;
            ESP_LOGE(TAG, "Transfer failed with status: %d", transfer->status);
            
            // 打印所有可能的传输状态
            switch (transfer->status) {
                case USB_TRANSFER_STATUS_COMPLETED: break;
                case USB_TRANSFER_STATUS_ERROR: 
                    ESP_LOGE(TAG, "Transfer error: USB_TRANSFER_STATUS_ERROR"); break;
                case USB_TRANSFER_STATUS_TIMED_OUT: 
                    ESP_LOGE(TAG, "Transfer error: USB_TRANSFER_STATUS_TIMED_OUT"); break;
                case USB_TRANSFER_STATUS_CANCELED: 
                    ESP_LOGE(TAG, "Transfer error: USB_TRANSFER_STATUS_CANCELED"); break;
                case USB_TRANSFER_STATUS_STALL: 
                    ESP_LOGE(TAG, "Transfer error: USB_TRANSFER_STATUS_STALL"); break;
                case USB_TRANSFER_STATUS_NO_DEVICE: 
                    ESP_LOGE(TAG, "Transfer error: USB_TRANSFER_STATUS_NO_DEVICE"); break;
                case USB_TRANSFER_STATUS_OVERFLOW: 
                    ESP_LOGE(TAG, "Transfer error: USB_TRANSFER_STATUS_OVERFLOW"); break;
                default: 
                    ESP_LOGE(TAG, "Transfer error: Unknown status %d", transfer->status); break;
            }
        }
        xSemaphoreGiveFromISR(mouse_dev.mutex, &xHigherPriorityTaskWoken);
    } else {
        ESP_LOGE(TAG, "Failed to take mutex in transfer callback");
    }
    
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

// 清理设备资源
static void cleanup_device(void)
{
    if (xSemaphoreTake(mouse_dev.mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        if (mouse_dev.transfer) {
            usb_host_transfer_free(mouse_dev.transfer);
            mouse_dev.transfer = NULL;
        }
        
        if (mouse_dev.device) {
            if (mouse_dev.client) {
                usb_host_interface_release(mouse_dev.client, mouse_dev.device, mouse_dev.interface_number);
            }
            usb_host_device_close(mouse_dev.client, mouse_dev.device);
            mouse_dev.device = NULL;
        }
        
        mouse_dev.transfer_active = false;
        mouse_dev.device_connected = false;
        xSemaphoreGive(mouse_dev.mutex);
    }
}


// 增强版：打印完整的设备描述符信息
static void print_device_info(usb_device_handle_t device)
{
    esp_err_t err;
    const usb_device_desc_t *dev_desc;

    // 1. 获取并打印设备描述符 (Device Descriptor)
    err = usb_host_get_device_descriptor(device, &dev_desc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get device descriptor: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "=== USB Device Descriptor ===");
    ESP_LOGI(TAG, "bLength: 0x%02X", dev_desc->bLength);
    ESP_LOGI(TAG, "bDescriptorType: 0x%02X", dev_desc->bDescriptorType);
    ESP_LOGI(TAG, "bcdUSB: 0x%04X", dev_desc->bcdUSB);
    ESP_LOGI(TAG, "bDeviceClass: 0x%02X", dev_desc->bDeviceClass);
    ESP_LOGI(TAG, "bDeviceSubClass: 0x%02X", dev_desc->bDeviceSubClass);
    ESP_LOGI(TAG, "bDeviceProtocol: 0x%02X", dev_desc->bDeviceProtocol);
    ESP_LOGI(TAG, "bMaxPacketSize0: %d", dev_desc->bMaxPacketSize0);
    ESP_LOGI(TAG, "idVendor: 0x%04X", dev_desc->idVendor);
    ESP_LOGI(TAG, "idProduct: 0x%04X", dev_desc->idProduct);
    ESP_LOGI(TAG, "bcdDevice: 0x%04X", dev_desc->bcdDevice);
    ESP_LOGI(TAG, "iManufacturer: 0x%02X", dev_desc->iManufacturer);
    ESP_LOGI(TAG, "iProduct: 0x%02X", dev_desc->iProduct);
    ESP_LOGI(TAG, "iSerialNumber: 0x%02X", dev_desc->iSerialNumber);
    ESP_LOGI(TAG, "bNumConfigurations: %d", dev_desc->bNumConfigurations);
    
    // 2. 获取设备速度信息
    usb_device_info_t dev_info;
    if (usb_host_device_info(device, &dev_info) == ESP_OK) {
        const char* speed_str = "Unknown";
        switch (dev_info.speed) {
            case USB_SPEED_LOW: speed_str = "Low (1.5 Mbps)"; break;
            case USB_SPEED_FULL: speed_str = "Full (12 Mbps)"; break;
            case USB_SPEED_HIGH: speed_str = "High (480 Mbps)"; break;
            // USB3.0速度可能需要根据实际支持的ESP32-S3版本添加
        }
        ESP_LOGI(TAG, "Device Speed: %s", speed_str);
    }

    // 3. 获取并打印配置描述符、接口描述符、端点描述符
    for (int config_index = 0; config_index < dev_desc->bNumConfigurations; config_index++) {
        const usb_config_desc_t *config_desc;
		esp_err_t err = usb_host_get_active_config_descriptor(device, &config_desc);
		if (err != ESP_OK) {
		    ESP_LOGE(TAG, "Failed to get config descriptor: %s", esp_err_to_name(err));
		    continue;
		}

        ESP_LOGI(TAG, "=== Configuration Descriptor [%d] ===", config_index);
        ESP_LOGI(TAG, "bLength: 0x%02X", config_desc->bLength);
        ESP_LOGI(TAG, "bDescriptorType: 0x%02X", config_desc->bDescriptorType);
        ESP_LOGI(TAG, "wTotalLength: %d bytes", config_desc->wTotalLength);
        ESP_LOGI(TAG, "bNumInterfaces: %d", config_desc->bNumInterfaces);
        ESP_LOGI(TAG, "bConfigurationValue: 0x%02X", config_desc->bConfigurationValue);
        ESP_LOGI(TAG, "iConfiguration: 0x%02X", config_desc->iConfiguration);
        ESP_LOGI(TAG, "bmAttributes: 0x%02X", config_desc->bmAttributes);
        ESP_LOGI(TAG, "bMaxPower: %d mA", config_desc->bMaxPower * 2); // 转换为mA

        // 递归解析配置描述符内的接口和端点描述符
        parse_configuration_descriptors(config_desc);
    }
}

// 递归解析配置描述符中的接口和端点描述符
static void parse_configuration_descriptors(const usb_config_desc_t *config_desc)
{
    const uint8_t *desc_ptr = (const uint8_t *)config_desc;
    uint16_t total_length = config_desc->wTotalLength;
    int interface_index = -1;
    int endpoint_index = 0;

    while (total_length > 0) {
        if (desc_ptr[0] == 0) break; // 描述符长度不能为0

        uint8_t desc_len = desc_ptr[0];
        uint8_t desc_type = desc_ptr[1];

        switch (desc_type) {
            case USB_B_DESCRIPTOR_TYPE_INTERFACE: {
                const usb_intf_desc_t *intf = (const usb_intf_desc_t *)desc_ptr;
                interface_index++;
                endpoint_index = 0; // 新接口，端点索引重置

                ESP_LOGI(TAG, "  --- Interface Descriptor [%d] ---", interface_index);
                ESP_LOGI(TAG, "  bLength: 0x%02X", intf->bLength);
                ESP_LOGI(TAG, "  bDescriptorType: 0x%02X", intf->bDescriptorType);
                ESP_LOGI(TAG, "  bInterfaceNumber: %d", intf->bInterfaceNumber);
                ESP_LOGI(TAG, "  bAlternateSetting: 0x%02X", intf->bAlternateSetting);
                ESP_LOGI(TAG, "  bNumEndpoints: %d", intf->bNumEndpoints);
                ESP_LOGI(TAG, "  bInterfaceClass: 0x%02X", intf->bInterfaceClass);
                ESP_LOGI(TAG, "  bInterfaceSubClass: 0x%02X", intf->bInterfaceSubClass);
                ESP_LOGI(TAG, "  bInterfaceProtocol: 0x%02X", intf->bInterfaceProtocol);
                ESP_LOGI(TAG, "  iInterface: 0x%02X", intf->iInterface);

                // 打印接口类型的可读信息
                print_interface_class_info(intf->bInterfaceClass, intf->bInterfaceSubClass, intf->bInterfaceProtocol);
                break;
            }

            case USB_B_DESCRIPTOR_TYPE_ENDPOINT: {
                const usb_ep_desc_t *ep = (const usb_ep_desc_t *)desc_ptr;
                endpoint_index++;

                ESP_LOGI(TAG, "    -> Endpoint Descriptor [%d]", endpoint_index);
                ESP_LOGI(TAG, "    bLength: 0x%02X", ep->bLength);
                ESP_LOGI(TAG, "    bDescriptorType: 0x%02X", ep->bDescriptorType);
                ESP_LOGI(TAG, "    bEndpointAddress: 0x%02X", ep->bEndpointAddress);
                ESP_LOGI(TAG, "    bmAttributes: 0x%02X", ep->bmAttributes);
                ESP_LOGI(TAG, "    wMaxPacketSize: %d", ep->wMaxPacketSize);
                ESP_LOGI(TAG, "    bInterval: %d", ep->bInterval);

                // 打印端点类型的可读信息
                print_endpoint_info(ep->bEndpointAddress, ep->bmAttributes);
                break;
            }

            case USB_B_DESCRIPTOR_TYPE_HID: {
                // 如果是HID设备，还可以解析HID描述符
                ESP_LOGI(TAG, "  [HID Descriptor Found]");
                // 可以在这里添加HID描述符的详细解析
                break;
            }

            default:
                // 其他类型的描述符（如类特定描述符）
                ESP_LOGI(TAG, "  [Other Descriptor Type: 0x%02X]", desc_type);
                break;
        }

        total_length -= (total_length >= desc_len) ? desc_len : total_length;
        desc_ptr += desc_len;
    }
}

// 打印接口类的可读信息
static void print_interface_class_info(uint8_t class, uint8_t subclass, uint8_t protocol)
{
    const char *class_str = "Unknown";
    switch (class) {
        case 0x01: class_str = "Audio"; break;
        case 0x03: class_str = "HID (Human Interface Device)"; break;
        case 0x08: class_str = "Mass Storage"; break;
        case 0x0E: class_str = "Video"; break;
        case 0xFF: class_str = "Vendor Specific"; break;
    }
    ESP_LOGI(TAG, "  Interface Class: %s (0x%02X)", class_str, class);
}

// 打印端点的可读信息
static void print_endpoint_info(uint8_t addr, uint8_t attr)
{
    const char *direction = (addr & 0x80) ? "IN" : "OUT";
    const char *type = "Unknown";
    
    switch (attr & 0x03) {
        case 0x00: type = "Control"; break;
        case 0x01: type = "Isochronous"; break;
        case 0x02: type = "Bulk"; break;
        case 0x03: type = "Interrupt"; break;
    }
    
    ESP_LOGI(TAG, "    Endpoint: %s, Type: %s, Address: 0x%02X", direction, type, addr);
}

// 查找HID鼠标接口和端点
static bool find_mouse_interface(const usb_config_desc_t *config_desc, 
                                uint8_t *interface_num, 
                                const usb_ep_desc_t **ep_desc)
{
    const uint8_t *desc_ptr = (const uint8_t *)config_desc;
    uint16_t total_length = config_desc->wTotalLength;
    const usb_intf_desc_t *found_intf = NULL;
    const usb_ep_desc_t *found_ep = NULL;
    
    // 第一遍：查找HID鼠标接口
    while (total_length > 0 && desc_ptr[0] > 0) {
        uint8_t desc_len = desc_ptr[0];
        uint8_t desc_type = desc_ptr[1];
        
        if (desc_type == USB_B_DESCRIPTOR_TYPE_INTERFACE) {
            const usb_intf_desc_t *intf = (const usb_intf_desc_t *)desc_ptr;
            
            // 检查是否是HID鼠标接口
            if (intf->bInterfaceClass == USB_CLASS_HID &&
                intf->bInterfaceSubClass == 0x01 && // Boot Interface
                intf->bInterfaceProtocol == 0x02) { // Mouse
                
                found_intf = intf;
                *interface_num = intf->bInterfaceNumber;
                ESP_LOGI(TAG, "Found HID Mouse interface: number=%d, endpoints=%d",
                         intf->bInterfaceNumber, intf->bNumEndpoints);
                break;
            }
        }
        total_length -= desc_len;
        desc_ptr += desc_len;
    }
    
    if (!found_intf) {
        ESP_LOGW(TAG, "No HID mouse interface found");
        return false;
    }
    
    // 第二遍：查找中断IN端点
    total_length = config_desc->wTotalLength;
    desc_ptr = (const uint8_t *)config_desc;
    
    while (total_length > 0 && desc_ptr[0] > 0) {
        uint8_t desc_len = desc_ptr[0];
        uint8_t desc_type = desc_ptr[1];
        
        if (desc_type == USB_B_DESCRIPTOR_TYPE_ENDPOINT) {
            const usb_ep_desc_t *ep = (const usb_ep_desc_t *)desc_ptr;
            
            // 检查是否是IN端点且为中断传输
            if ((ep->bEndpointAddress & 0x80) && // IN端点
                ((ep->bmAttributes & 0x03) == 0x03)) { // 中断传输
                
                found_ep = ep;
                *ep_desc = ep;
                ESP_LOGI(TAG, "Found Interrupt IN endpoint: address=0x%02X, max packet=%d, interval=%d",
                         ep->bEndpointAddress, ep->wMaxPacketSize, ep->bInterval);
                break;
            }
        }
        total_length -= desc_len;
        desc_ptr += desc_len;
    }
    
    return (found_ep != NULL);
}

// 初始化鼠标设备
static bool init_mouse_device(usb_device_handle_t device, usb_host_client_handle_t client)
{
    // 获取配置描述符
    const usb_config_desc_t *config_desc;
    if (usb_host_get_active_config_descriptor(device, &config_desc) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get configuration descriptor");
        return false;
    }
    
    // 查找鼠标接口和端点
    uint8_t interface_num;
    const usb_ep_desc_t *ep_desc;
    if (!find_mouse_interface(config_desc, &interface_num, &ep_desc)) {
        return false;
    }
    
    // 声明接口
    esp_err_t err = usb_host_interface_claim(client, device, interface_num, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to claim interface: %s", esp_err_to_name(err));
        return false;
    }
    
    // 分配传输
    usb_transfer_t *transfer;
    err = usb_host_transfer_alloc(ep_desc->wMaxPacketSize, 0, &transfer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to allocate transfer: %s", esp_err_to_name(err));
        usb_host_interface_release(client, device, interface_num);
        return false;
    }
    
    // 配置传输
    transfer->device_handle = device;
    transfer->bEndpointAddress = ep_desc->bEndpointAddress;
    transfer->callback = transfer_cb;
    transfer->context = NULL;
    transfer->num_bytes = ep_desc->wMaxPacketSize;
    
    // 提交传输
    err = usb_host_transfer_submit(transfer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to submit transfer: %s", esp_err_to_name(err));
        usb_host_transfer_free(transfer);
        usb_host_interface_release(client, device, interface_num);
        return false;
    }
    
    // 保存设备信息
    if (xSemaphoreTake(mouse_dev.mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        mouse_dev.device = device;
        mouse_dev.interface_number = interface_num;
        mouse_dev.endpoint_address = ep_desc->bEndpointAddress;
        mouse_dev.max_packet_size = ep_desc->wMaxPacketSize;
        mouse_dev.transfer = transfer;
        mouse_dev.transfer_active = true;
        mouse_dev.device_connected = true;
        xSemaphoreGive(mouse_dev.mutex);
    }
    
    ESP_LOGI(TAG, "HID Mouse successfully initialized");
    return true;
}

// 客户端事件回调
static void client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg)
{
    switch (event_msg->event) {
        case USB_HOST_CLIENT_EVENT_NEW_DEV:
            ESP_LOGI(TAG, "New USB device detected, address: %d", event_msg->new_dev.address);
            
            if (mouse_dev.device_connected) {
                ESP_LOGW(TAG, "Mouse device already connected, ignoring new device");
                return;
            }
            
            usb_device_handle_t device;
            esp_err_t err = usb_host_device_open(mouse_dev.client, event_msg->new_dev.address, &device);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to open device: %s", esp_err_to_name(err));
                return;
            }
            
            ESP_LOGI(TAG, "Device opened successfully");
            print_device_info(device);
            
            // 尝试初始化为鼠标设备
            if (init_mouse_device(device, mouse_dev.client)) {
                ESP_LOGI(TAG, "Mouse device initialized successfully");
            } else {
                ESP_LOGW(TAG, "Device is not a mouse or initialization failed");
                usb_host_device_close(mouse_dev.client, device);
            }
            break;
            
        case USB_HOST_CLIENT_EVENT_DEV_GONE:
            ESP_LOGI(TAG, "Device disconnected");
            cleanup_device();
            break;
    }
}


// USB主机初始化
void usb_host_init(void)
{
    // 初始化设备结构体
    mouse_dev.mutex = xSemaphoreCreateMutex();
    if (!mouse_dev.mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }
    
    // USB主机配置
    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    
    esp_err_t err = usb_host_install(&host_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install USB host: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "USB host library installed successfully");
    
    // 客户端配置
    const usb_host_client_config_t client_config = {
        .is_synchronous = false,
        .max_num_event_msg = 5,
        .async = {
            .client_event_callback = client_event_cb,
            .callback_arg = NULL
        }
    };
    
    err = usb_host_client_register(&client_config, &mouse_dev.client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register USB client: %s", esp_err_to_name(err));
        usb_host_uninstall();
        return;
    }
    ESP_LOGI(TAG, "USB client registered successfully");
}

// 主任务
void app_main(void)
{
    ESP_LOGI(TAG, "Starting USB Mouse Reader");
    // 初始化USB主机
    usb_host_init();
    
    if (!mouse_dev.client) {
        ESP_LOGE(TAG, "USB host initialization failed");
        return;
    }
    
    ESP_LOGI(TAG, "Please insert a USB mouse into the ESP32-S3 USB port");
    ESP_LOGI(TAG, "Waiting for device connection...");
    
    uint32_t event_flags;
    
    while (1) {

        // 处理USB库事件
		esp_err_t err = usb_host_lib_handle_events(pdMS_TO_TICKS(10), &event_flags);
		if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
		    ESP_LOGE(TAG, "usb_host_lib_handle_events failed: %s", esp_err_to_name(err));
		    vTaskDelay(pdMS_TO_TICKS(100));
		    continue;
		}
        // 处理客户端事件
        err = usb_host_client_handle_events(mouse_dev.client, pdMS_TO_TICKS(10));
        if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "usb_host_client_handle_events failed: %s", esp_err_to_name(err));
        }
/*        // 定期状态检查和恢复
        TickType_t current_tick = xTaskGetTickCount();
        if (current_tick - last_status_check > pdMS_TO_TICKS(2000)) {
            if (mouse_dev.device_connected) {
                ESP_LOGI(TAG, "Mouse connected, transfer active: %s", 
                        mouse_dev.transfer_active ? "YES" : "NO");
                
                // 检查并恢复传输
                check_and_recover_transfer();
            } else {
                ESP_LOGI(TAG, "Waiting for mouse device...");
            }
            last_status_check = current_tick;
        }
*/
        // 短暂延迟避免忙等待
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}