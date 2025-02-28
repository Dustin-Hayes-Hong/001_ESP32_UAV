/**
 * @file VBAT.h
 * @brief 电池电压监测模块，提供 ADC 初始化和电压读取功能
 * @details 
 * 本文件实现了一个基于 ESP-IDF 的电池电压监测模块，使用 ADC1 通道读取电压值。
 * 支持线性拟合校准方案，适用于 ESP32S3 或类似硬件。
 * 注意：需要在 esp-idf 配置中启用 ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED。
 * @author [Your Name]
 * @date [Current Date]
 */

 #include "VBAT.h"
 #include "Data_declaration.h"
 #include "esp_adc/adc_oneshot.h"       // ADC 一次性读取接口
 #include "esp_adc/adc_cali.h"          // ADC 校准接口
 #include "esp_adc/adc_cali_scheme.h"   // ADC 校准方案
 #include "freertos/FreeRTOS.h"         // FreeRTOS 任务支持
 #include "freertos/task.h"             // FreeRTOS 任务管理
 #include "esp_err.h"                   // ESP 错误检查
 
 // ADC 配置常量
 #define ADC1_VBAT_CHANNEL ADC_CHANNEL_3  // GPIO4 对应的 ADC1 通道，用于电池电压检测
 
 // 全局变量
 static adc_oneshot_unit_handle_t adc1_handle;  // ADC1 单元句柄，用于一次性读取
 static adc_cali_handle_t adc_cali_handle;      // ADC 校准句柄，用于原始值转电压
 int VBAT = 0;                                  // 电池电压（单位：毫伏），全局共享变量
 
 /**
  * @brief 初始化电池电压监测模块
  * @details 
  * 配置 ADC1 单元和通道，使用线性拟合校准方案，并创建电压读取任务。
  * @return esp_err_t 初始化结果，ESP_OK 表示成功
  */
 int VBAT_init(void)
 {
     // 初始化 ADC1 单元
     adc_oneshot_unit_init_cfg_t init_config1 = {
         .unit_id = ADC_UNIT_1,         // 使用 ADC1
         .ulp_mode = ADC_ULP_MODE_DISABLE, // 禁用超低功耗模式
     };
     ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
 
     // 配置 ADC 通道
     adc_oneshot_chan_cfg_t config = {
         .bitwidth = ADC_BITWIDTH_DEFAULT, // 使用默认位宽（通常 12 位）
         .atten = ADC_ATTEN_DB_12,         // 衰减设置为 12dB，适用于 0-3.9V 输入范围
     };
     ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC1_VBAT_CHANNEL, &config));
 
    //  // 配置 ADC 校准（线性拟合方案）
    //  adc_cali_line_fitting_config_t cali_config = {
    //      .unit_id = ADC_UNIT_1,         // ADC1 单元
    //      .atten = ADC_ATTEN_DB_12,      // 衰减设置，与通道配置一致
    //      .bitwidth = ADC_BITWIDTH_DEFAULT, // 默认位宽
    //      .default_vref = 3300,          // 默认参考电压（单位：毫伏），可根据硬件调整
    //  };
    //  ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_config, &adc_cali_handle));
 
    //  // 检查 EFUSE 中的校准值
    //  adc_cali_line_fitting_efuse_val_t cali_val = ADC_CALI_LINE_FITTING_EFUSE_VAL_DEFAULT_VREF;
    //  adc_cali_scheme_line_fitting_check_efuse(&cali_val);
 
     // 创建电池电压读取任务
     xTaskCreate(vbat_task, "VBAT_task", 2048, NULL, 5, NULL);
 
     return ESP_OK;
 }
 
 /**
  * @brief 读取电池电压
  * @details 
  * 从指定 ADC 通道读取原始值，并转换为校准后的电压值（单位：毫伏）。
  * @return float 电池电压（单位：毫伏）
  */
 float read_vbat(void)
 {
     int adc_raw = 0;
     int voltage = 0;
 
     // 读取 ADC 原始值
     ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC1_VBAT_CHANNEL, &adc_raw));
 
     // 将原始值转换为电压（单位：毫伏）
    //  ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, adc_raw, &voltage));
 
     VBAT = voltage; // 更新全局变量
     return (float)voltage; // 返回毫伏单位，避免除以 1000（根据需求调整）
 }
 
 /**
  * @brief 电池电压监测任务
  * @details 
  * 周期性读取电池电压并打印，用于调试或监控。
  * @param pvParameters 任务参数（未使用）
  * @return 无（无限循环）
  */
 void vbat_task(void *pvParameters)
 {
     while (true) {
         if (init_ok) {
             float vbat = read_vbat();
             printf("Battery Voltage: %.2f V\n", vbat / 1000.0f); // 转换为伏特并打印
         }
         vTaskDelay(1000 / portTICK_PERIOD_MS); // 每秒读取一次
     }
 }
 