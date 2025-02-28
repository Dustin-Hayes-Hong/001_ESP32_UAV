#include "LED.h"
#include "Data_declaration.h"
#include "VBAT.h"//电池电压读取头文件
#include "IMU.h"	//姿态解算头文件
/**
 * @brief LED任务，周期性控制LED状态
 * @param pvParameter 任务参数（未使用）
 */
void led_task(void *pvParameters) {
    // 定义常量和变量
    const TickType_t LED_PERIOD = pdMS_TO_TICKS(50); // LED状态更新周期：50ms
    const uint32_t VBAT_THRESHOLD = 380;             // 电池电压阈值：3.8V
    const uint8_t RC_CHECK_INTERVAL = 100;           // RC连接检查间隔计数（100 * 50ms = 5s）

    bool is_led2_blinking = false;                   // 黄灯闪烁状态
    uint8_t rc_check_counter = 0;                    // RC状态检查计数器
    TickType_t last_wake_time = xTaskGetTickCount(); // 上次唤醒时间

    while (1) {
        vTaskDelayUntil(&last_wake_time, LED_PERIOD); // 周期性延迟
        if(init_ok){
            // 黄灯控制：电池电压检测
        if (VBAT <= VBAT_THRESHOLD) {
            is_led2_blinking = !is_led2_blinking;     // 电压低时闪烁
            gpio_set_level(LED2, is_led2_blinking);
        } else {
            gpio_set_level(LED2, 0);                  // 电压正常时常亮
        }

        // 绿灯控制：RC连接状态检测
        if (rc_check_counter >= RC_CHECK_INTERVAL) {
            rc_check_counter = 0;                     // 重置计数器
            if (state.rc_link) {
                gpio_set_level(LED1, 0);              // RC连接时绿灯亮
            } else {
                gpio_set_level(LED1, 1);              // RC断开时绿灯灭
                state.isRCLocked = false;             // 更新锁定状态
            }
            state.rc_link = false;                    // 重置RC连接状态
        } else {
            rc_check_counter++;                       // 计数器递增
        }
        }
        
    }
}

/**
 * @brief 初始化LED相关的GPIO并启动LED任务
 */
void led_init(void) {
    // 定义GPIO配置结构体
    gpio_config_t led_config = {
        .mode = GPIO_MODE_OUTPUT,           // 设置为输出模式
        .intr_type = GPIO_INTR_DISABLE,     // 禁用中断
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // 禁用下拉
        .pull_up_en = GPIO_PULLUP_DISABLE,     // 禁用上拉
        .pin_bit_mask = (1ULL << LED1) | (1ULL << LED2) | (1ULL << LED3) // 配置所有LED引脚
    };

    // 应用GPIO配置
    gpio_config(&led_config);

    // LED启动自检序列
    const uint32_t DELAY_SHORT = pdMS_TO_TICKS(500);  // 短延时：500ms
    const uint32_t DELAY_LONG = pdMS_TO_TICKS(1000);  // 长延时：1000ms

    // 所有LED点亮
    gpio_set_level(LED1, 1);
    gpio_set_level(LED2, 1);
    gpio_set_level(LED3, 1);
    vTaskDelay(DELAY_SHORT);

    // 依次熄灭LED
    gpio_set_level(LED3, 0);
    vTaskDelay(DELAY_SHORT);
    gpio_set_level(LED1, 0);
    vTaskDelay(DELAY_SHORT);
    gpio_set_level(LED2, 0);
    vTaskDelay(DELAY_SHORT);

    // 再次点亮并保持1秒
    gpio_set_level(LED1, 1);
    gpio_set_level(LED2, 1);
    gpio_set_level(LED3, 1);
    vTaskDelay(DELAY_LONG);

    // 创建LED任务
    xTaskCreate(led_task, "led_task", 5120, NULL, tskIDLE_PRIORITY, NULL);
}