#include "PWM.h"

#include <stdio.h>
#include "driver/ledc.h"
#include "esp_err.h"

// PWM引脚定义
#define PWM1_GPIO          42    // PWM1 输出GPIO
#define PWM2_GPIO          6     // PWM2 输出GPIO
#define PWM3_GPIO          15    // PWM3 输出GPIO
#define PWM4_GPIO          38    // PWM4 输出GPIO

// PWM定时器定义
#define PWM1_TIMER         LEDC_TIMER_0    // PWM1 定时器
#define PWM2_TIMER         LEDC_TIMER_1    // PWM2 定时器
#define PWM3_TIMER         LEDC_TIMER_2    // PWM3 定时器
#define PWM4_TIMER         LEDC_TIMER_3    // PWM4 定时器

// PWM通道定义
#define PWM1_CHANNEL       LEDC_CHANNEL_0  // PWM1 通道
#define PWM2_CHANNEL       LEDC_CHANNEL_1  // PWM2 通道
#define PWM3_CHANNEL       LEDC_CHANNEL_2  // PWM3 通道
#define PWM4_CHANNEL       LEDC_CHANNEL_3  // PWM4 通道

// PWM参数配置
#define PWM_MAX_DUTY       1000             // PWM占空比最大值
#define PWM_MODE           LEDC_LOW_SPEED_MODE  // PWM低速模式
#define PWM_RESOLUTION     LEDC_TIMER_8_BIT     // 8位分辨率
#define PWM_FREQ_HZ        1000                 // PWM频率(Hz)

/**
 * @brief 初始化PWM模块
 * @note 配置4个PWM通道，使用8位分辨率和1kHz频率，初始占空比为0
 */
void PWM_init(void)
{
    // PWM1 定时器配置
    ledc_timer_config_t PWM1_timer = {
        .speed_mode = PWM_MODE,         // 低速模式
        .timer_num = PWM1_TIMER,        // 定时器0
        .duty_resolution = PWM_RESOLUTION,  // 8位分辨率
        .freq_hz = PWM_FREQ_HZ,         // 频率1kHz
        .clk_cfg = LEDC_AUTO_CLK        // 自动选择时钟源
    };
    ESP_ERROR_CHECK(ledc_timer_config(&PWM1_timer));

    // PWM2 定时器配置
    ledc_timer_config_t PWM2_timer = {
        .speed_mode = PWM_MODE,         // 低速模式
        .timer_num = PWM2_TIMER,        // 定时器1
        .duty_resolution = PWM_RESOLUTION,  // 8位分辨率
        .freq_hz = PWM_FREQ_HZ,         // 频率1kHz
        .clk_cfg = LEDC_AUTO_CLK        // 自动选择时钟源
    };
    ESP_ERROR_CHECK(ledc_timer_config(&PWM2_timer));

    // PWM3 定时器配置
    ledc_timer_config_t PWM3_timer = {
        .speed_mode = PWM_MODE,         // 低速模式
        .timer_num = PWM3_TIMER,        // 定时器2
        .duty_resolution = PWM_RESOLUTION,  // 8位分辨率
        .freq_hz = PWM_FREQ_HZ,         // 频率1kHz
        .clk_cfg = LEDC_AUTO_CLK        // 自动选择时钟源
    };
    ESP_ERROR_CHECK(ledc_timer_config(&PWM3_timer));

    // PWM4 定时器配置
    ledc_timer_config_t PWM4_timer = {
        .speed_mode = PWM_MODE,         // 低速模式
        .timer_num = PWM4_TIMER,        // 定时器3
        .duty_resolution = PWM_RESOLUTION,  // 8位分辨率
        .freq_hz = PWM_FREQ_HZ,         // 频率1kHz
        .clk_cfg = LEDC_AUTO_CLK        // 自动选择时钟源
    };
    ESP_ERROR_CHECK(ledc_timer_config(&PWM4_timer));

    // PWM1 通道配置
    ledc_channel_config_t PWM1_channel = {
        .speed_mode = PWM_MODE,         // 低速模式
        .channel = PWM1_CHANNEL,        // 通道0
        .timer_sel = PWM1_TIMER,        // 选择定时器0
        .intr_type = LEDC_INTR_DISABLE, // 禁用中断
        .gpio_num = PWM1_GPIO,          // GPIO 42
        .duty = 0,                      // 初始占空比为0
        .hpoint = 0                     // 高电平点为0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&PWM1_channel));

    // PWM2 通道配置
    ledc_channel_config_t PWM2_channel = {
        .speed_mode = PWM_MODE,         // 低速模式
        .channel = PWM2_CHANNEL,        // 通道1
        .timer_sel = PWM2_TIMER,        // 选择定时器1
        .intr_type = LEDC_INTR_DISABLE, // 禁用中断
        .gpio_num = PWM2_GPIO,          // GPIO 6
        .duty = 0,                      // 初始占空比为0
        .hpoint = 0                     // 高电平点为0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&PWM2_channel));

    // PWM3 通道配置
    ledc_channel_config_t PWM3_channel = {
        .speed_mode = PWM_MODE,         // 低速模式
        .channel = PWM3_CHANNEL,        // 通道2
        .timer_sel = PWM3_TIMER,        // 选择定时器2
        .intr_type = LEDC_INTR_DISABLE, // 禁用中断
        .gpio_num = PWM3_GPIO,          // GPIO 15
        .duty = 0,                      // 初始占空比为0
        .hpoint = 0                     // 高电平点为0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&PWM3_channel));

    // PWM4 通道配置
    ledc_channel_config_t PWM4_channel = {
        .speed_mode = PWM_MODE,         // 低速模式
        .channel = PWM4_CHANNEL,        // 通道3
        .timer_sel = PWM4_TIMER,        // 选择定时器3
        .intr_type = LEDC_INTR_DISABLE, // 禁用中断
        .gpio_num = PWM4_GPIO,          // GPIO 38
        .duty = 0,                      // 初始占空比为0
        .hpoint = 0                     // 高电平点为0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&PWM4_channel));
}

/**
 * @brief 设置电机PWM值
 * @param[in] MOTO1_PWM 电机1 PWM值 (0-1000)
 * @param[in] MOTO2_PWM 电机2 PWM值 (0-1000)
 * @param[in] MOTO3_PWM 电机3 PWM值 (0-1000)
 * @param[in] MOTO4_PWM 电机4 PWM值 (0-1000)
 * @note 输入值会被限制在0-1000范围，并转换为8位分辨率(0-255)
 */
void Moto_Pwm(float MOTO1_PWM, float MOTO2_PWM, float MOTO3_PWM, float MOTO4_PWM)
{
    // 限制PWM值在0-1000范围内
    if (MOTO1_PWM > PWM_MAX_DUTY) MOTO1_PWM = PWM_MAX_DUTY;
    if (MOTO2_PWM > PWM_MAX_DUTY) MOTO2_PWM = PWM_MAX_DUTY;
    if (MOTO3_PWM > PWM_MAX_DUTY) MOTO3_PWM = PWM_MAX_DUTY;
    if (MOTO4_PWM > PWM_MAX_DUTY) MOTO4_PWM = PWM_MAX_DUTY;
    if (MOTO1_PWM < 0.0f) MOTO1_PWM = 0.0f;
    if (MOTO2_PWM < 0.0f) MOTO2_PWM = 0.0f;
    if (MOTO3_PWM < 0.0f) MOTO3_PWM = 0.0f;
    if (MOTO4_PWM < 0.0f) MOTO4_PWM = 0.0f;

    // 设置各通道PWM占空比 (转换为8位分辨率)
    ledc_set_duty(PWM_MODE, PWM1_CHANNEL, (uint32_t)(MOTO1_PWM * 0.255f));
    ledc_set_duty(PWM_MODE, PWM2_CHANNEL, (uint32_t)(MOTO2_PWM * 0.255f));
    ledc_set_duty(PWM_MODE, PWM3_CHANNEL, (uint32_t)(MOTO3_PWM * 0.255f));
    ledc_set_duty(PWM_MODE, PWM4_CHANNEL, (uint32_t)(MOTO4_PWM * 0.255f));

    // 更新各通道PWM值
    ledc_update_duty(PWM_MODE, PWM1_CHANNEL);
    ledc_update_duty(PWM_MODE, PWM2_CHANNEL);
    ledc_update_duty(PWM_MODE, PWM3_CHANNEL);
    ledc_update_duty(PWM_MODE, PWM4_CHANNEL);
}