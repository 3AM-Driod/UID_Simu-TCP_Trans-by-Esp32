#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"
#include "driver/ledc.h"

static const char *TAG = "RGB_Breathing";
#define BLINK_GPIO CONFIG_BLINK_GPIO

// PWM配置参数
#define LEDC_CHANNEL_RED     LEDC_CHANNEL_0
#define LEDC_CHANNEL_GREEN   LEDC_CHANNEL_1
#define LEDC_CHANNEL_BLUE    LEDC_CHANNEL_2
#define LEDC_RESOLUTION      LEDC_TIMER_13_BIT
#define LEDC_FREQ            5000
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_TIMER          LEDC_TIMER_0

// 可调参数
#define MAX_BRIGHTNESS_PERCENT 10  // 最大亮度百分比
#define COLOR_CYCLE_MS 8000         // 完整颜色循环周期(毫秒)
#define SIGMOID_STEEPNESS 8.0       // Sigmoid曲线陡度

#ifdef CONFIG_BLINK_LED_STRIP

static led_strip_handle_t led_strip;
static float color_hue = 0.0;        // 色相 (0-360度)
static float breath_position = -6.0; // 呼吸位置

// HSV转RGB函数
static void hsv_to_rgb(float h, float s, float v, uint8_t *r, uint8_t *g, uint8_t *b) {
    h = fmodf(h, 360.0f);
    if (h < 0) h += 360.0f;
    
    float c = v * s;
    float x = c * (1 - fabsf(fmodf(h / 60.0f, 2) - 1));
    float m = v - c;
    
    float r_, g_, b_;
    
    if (h < 60) {
        r_ = c; g_ = x; b_ = 0;
    } else if (h < 120) {
        r_ = x; g_ = c; b_ = 0;
    } else if (h < 180) {
        r_ = 0; g_ = c; b_ = x;
    } else if (h < 240) {
        r_ = 0; g_ = x; b_ = c;
    } else if (h < 300) {
        r_ = x; g_ = 0; b_ = c;
    } else {
        r_ = c; g_ = 0; b_ = x;
    }
    
    *r = (uint8_t)((r_ + m) * 255);
    *g = (uint8_t)((g_ + m) * 255);
    *b = (uint8_t)((b_ + m) * 255);
}

// Sigmoid函数
static float sigmoid(float x) {
    return 1.0f / (1.0f + expf(-x));
}

static void set_rgb_led(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness) {
    // 应用全局亮度控制
    r = (uint8_t)(r * brightness / 255);
    g = (uint8_t)(g * brightness / 255);
    b = (uint8_t)(b * brightness / 255);
    
    led_strip_set_pixel(led_strip, 0, r, g, b);
    led_strip_refresh(led_strip);
}

static void configure_led(void) {
    ESP_LOGI(TAG, "彩色呼吸灯配置完成!");
    ESP_LOGI(TAG, "最大亮度: %d%%, 颜色循环周期: %dms", MAX_BRIGHTNESS_PERCENT, COLOR_CYCLE_MS);
    
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1,
    };
    
#if CONFIG_BLINK_LED_STRIP_BACKEND_RMT
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
#elif CONFIG_BLINK_LED_STRIP_BACKEND_SPI
    led_strip_spi_config_t spi_config = {
        .spi_bus = SPI2_HOST,
        .flags.with_dma = true,
    };
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
#else
#error "不支持的LED灯带后端"
#endif
    
    led_strip_clear(led_strip);
}

#elif CONFIG_BLINK_LED_GPIO

// 对于GPIO LED，我们使用PWM实现彩色呼吸效果
static void set_rgb_led(uint32_t r_duty, uint32_t g_duty, uint32_t b_duty) {
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_RED, r_duty);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_GREEN, g_duty);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_BLUE, b_duty);
    
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_RED);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_GREEN);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_BLUE);
}

// HSV转RGB函数
static void hsv_to_rgb_pwm(float h, float s, float v, uint32_t max_duty, 
                          uint32_t *r_duty, uint32_t *g_duty, uint32_t *b_duty) {
    h = fmodf(h, 360.0f);
    if (h < 0) h += 360.0f;
    
    float c = v * s;
    float x = c * (1 - fabsf(fmodf(h / 60.0f, 2) - 1));
    float m = v - c;
    
    float r, g, b;
    
    if (h < 60) {
        r = c; g = x; b = 0;
    } else if (h < 120) {
        r = x; g = c; b = 0;
    } else if (h < 180) {
        r = 0; g = c; b = x;
    } else if (h < 240) {
        r = 0; g = x; b = c;
    } else if (h < 300) {
        r = x; g = 0; b = c;
    } else {
        r = c; g = 0; b = x;
    }
    
    *r_duty = (uint32_t)((r + m) * max_duty);
    *g_duty = (uint32_t)((g + m) * max_duty);
    *b_duty = (uint32_t)((b + m) * max_duty);
}

// Sigmoid函数
static float sigmoid(float x) {
    return 1.0f / (1.0f + expf(-x));
}

static void configure_led(void) {
    ESP_LOGI(TAG, "RGB PWM呼吸灯配置完成!");
    ESP_LOGI(TAG, "最大亮度: %d%%, 颜色循环周期: %dms", MAX_BRIGHTNESS_PERCENT, COLOR_CYCLE_MS);

    // 配置PWM定时器
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_RESOLUTION,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // 配置红色通道
    ledc_channel_config_t ledc_channel_red = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_RED,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = BLINK_GPIO,  // 实际应用中需要为RGB分别定义GPIO
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_red));

    // 配置绿色通道（需要不同的GPIO）
    ledc_channel_config_t ledc_channel_green = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_GREEN,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = BLINK_GPIO,  // 需要修改为实际GPIO
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_green));

    // 配置蓝色通道（需要不同的GPIO）
    ledc_channel_config_t ledc_channel_blue = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_BLUE,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = BLINK_GPIO,  // 需要修改为实际GPIO
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_blue));
}

#else
#error "不支持的LED类型"
#endif

void app_main(void) {
    configure_led();
    
#ifdef CONFIG_BLINK_LED_GPIO
    const uint32_t max_duty = (1 << LEDC_RESOLUTION) - 1;
    const uint32_t adjusted_max_duty = (uint32_t)(max_duty * MAX_BRIGHTNESS_PERCENT / 100.0f);
#else
    const uint8_t adjusted_max_brightness = (uint8_t)(255 * MAX_BRIGHTNESS_PERCENT / 100.0f);
#endif

    float color_hue = 0.0f;           // 色相 (0-360度)
    float breath_position = -SIGMOID_STEEPNESS;
    int breath_direction = 1;
    
    // 计算步长
    const TickType_t step_delay = pdMS_TO_TICKS(30); // 每30ms更新一次
    const int total_steps = COLOR_CYCLE_MS / 30;
    const float hue_step = 360.0f / total_steps;      // 色相步长
    const float position_step = (2 * SIGMOID_STEEPNESS) / total_steps;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        // 更新色相（颜色循环）
        color_hue += hue_step;
        if (color_hue >= 360.0f) {
            color_hue = 0.0f;
        }
        
        // 更新呼吸位置
        breath_position += breath_direction * position_step;
        if (breath_position >= SIGMOID_STEEPNESS) {
            breath_position = SIGMOID_STEEPNESS;
            breath_direction = -1;
        } else if (breath_position <= -SIGMOID_STEEPNESS) {
            breath_position = -SIGMOID_STEEPNESS;
            breath_direction = 1;
        }
        
        // 计算亮度系数 (0.0 - 1.0)
        float brightness_factor = sigmoid(breath_position);

#ifdef CONFIG_BLINK_LED_GPIO
        // GPIO RGB LED模式
        uint32_t r_duty, g_duty, b_duty;
        hsv_to_rgb_pwm(color_hue, 1.0f, brightness_factor, adjusted_max_duty, 
                      &r_duty, &g_duty, &b_duty);
        
        set_rgb_led(r_duty, g_duty, b_duty);
        
        ESP_LOGI(TAG, "色相: %.1f°, 亮度: %.3f, RGB占空比: %lu,%lu,%lu", 
                 color_hue, brightness_factor, r_duty, g_duty, b_duty);
#else
        // 可寻址LED模式
        uint8_t r, g, b;
        hsv_to_rgb(color_hue, 1.0f, 1.0f, &r, &g, &b); // 先获取最大亮度颜色
        
        // 应用呼吸亮度
        uint8_t brightness = (uint8_t)(brightness_factor * adjusted_max_brightness);
        set_rgb_led(r, g, b, brightness);
        
        ESP_LOGI(TAG, "色相: %.1f°, 亮度: %.3f, RGB原始: %d,%d,%d, 实际亮度: %d", 
                 color_hue, brightness_factor, r, g, b, brightness);
#endif

        vTaskDelayUntil(&xLastWakeTime, step_delay);
    }
}