#include "VBAT.h"

//说实话有个 battery_monitor  组件，但还是想自己完成新api的移植（一种植物）
//https://www.youtube.com/watch?v=4nnCbbZbdk0

#include "hal/adc_types.h"
#include "esp_adc/adc_oneshot.h"  // 一次性读取
#include "esp_adc/adc_cali.h"   // ADC校准
#include "esp_adc/adc_cali_scheme.h"  // ADC校准方案
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// ADC所接的通道  GPIO4 if ADC1  = ADC1_CHANNEL_3
#define ADC1_VBAT_CHANNEL ADC_CHANNEL_3  //GPIO4

// ADC曲线拟合，需要在“*\v5.4\esp-idf\components\esp_adc\esp32s3\include\adc_cali_schemes.h"中定义#define ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED 1

adc_oneshot_unit_handle_t adc1_handle;  // 一次性读取句柄
adc_cali_handle_t adc_cali_handle;      // ADC校准句柄

int VBAT_init(void)
{
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));  // 创建一次性读取单元

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC1_VBAT_CHANNEL, &config));  // 配置通道

    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle);

    return ESP_OK;
}

// 读取电池电压
float read_vbat() {
    int adc_raw, voltage;
    adc_raw = adc_oneshot_read(adc1_handle, ADC1_VBAT_CHANNEL, &voltage);
    adc_cali_raw_to_voltage(adc_cali_handle, adc_raw, &voltage);
    return voltage / 1000.0f; // 转换为伏特
}

// 电池电压读取任务
void vbat_task(void *pvParameters) {
    while (true) {
        float vbat = read_vbat();
        printf("Battery Voltage: %.2f V\n", vbat);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}