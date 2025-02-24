#include "UART.h"

/**
 * @brief 初始化 UART1
 * @param baud 波特率
 * @note  配置串口参数、安装驱动并设置引脚
 */
void uart1_init(int baud) {
    printf("*******************串口初始化开始*******************\n");

    // 配置串口参数
    const uart_config_t uart_config = {
        .baud_rate = baud,                // 波特率
        .data_bits = UART_DATA_8_BITS,    // 数据位：8位
        .parity = UART_PARITY_DISABLE,    // 奇偶校验：禁用
        .stop_bits = UART_STOP_BITS_1,    // 停止位：1位
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // 硬件流控：禁用
        .source_clk = UART_SCLK_APB       // 时钟源：APB
    };

    // 打印配置信息
    printf("串口号: %d\n", UART_NUM_0);
    printf("波特率: %d\n", uart_config.baud_rate);
    printf("数据位: 8位\n");
    printf("奇偶校验: 禁用\n");
    printf("停止位: 1位\n");
    printf("硬件流控: 禁用\n");
    printf("接收缓冲区大小: %d\n", RX_BUF_SIZE * 2);
    printf("发送缓冲区大小: 0\n");
    printf("事件队列大小: 0\n");
    printf("TXD引脚: %d\n", TXD_PIN);
    printf("RXD引脚: %d\n", RXD_PIN);

    // 安装串口驱动
    esp_err_t ret = uart_driver_install(UART_NUM_0, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    printf("串口驱动安装 %s\n", ret == ESP_OK ? "成功" : "失败");

    // 配置串口参数
    ret = uart_param_config(UART_NUM_0, &uart_config);
    printf("串口参数配置 %s\n", ret == ESP_OK ? "成功" : "失败");

    // 设置引脚
    ret = uart_set_pin(UART_NUM_0, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    printf("串口引脚设置 %s\n", ret == ESP_OK ? "成功" : "失败");

    printf("*******************串口初始化结束*******************\n");
}

/**
 * @brief 发送数据通过UART
 * @param data 要发送的字符串
 * @return 发送的字节数，错误时返回负值
 */
int sendData(const char *data) {
    if (data == NULL) return -1; // 输入检查
    int len = strlen(data);
    int txBytes = uart_write_bytes(UART_NUM_0, data, len);
    // ESP_LOGI("UART", "Wrote %d bytes", txBytes); // 调试用，可启用
    return txBytes;
}

/**
 * @brief UART发送任务
 * @param arg 任务参数（未使用）
 * @note  每2秒发送一次数据
 */
void tx_task(void *arg) {
    (void)arg; // 避免未使用警告
    while (1) {
        sendData("设置\n"); // 发送数据并换行
        vTaskDelay(2000 / portTICK_PERIOD_MS); // 延时2秒
    }
}

/**
 * @brief UART接收任务
 * @param arg 任务参数（未使用）
 * @note  循环接收数据并回显
 */
void rx_task(void *arg) {
    (void)arg; // 避免未使用警告
    uint8_t *data = (uint8_t *)malloc(RX_BUF_SIZE + 1);
    if (data == NULL) return; // 内存分配失败检查

    while (1) {
        int rxBytes = uart_read_bytes(UART_NUM_0, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = '\0'; // 确保字符串终止
            uart_write_bytes(UART_NUM_0, (char *)data, rxBytes); // 回显数据
            // ESP_LOGI("RX_TASK", "Read %d bytes: '%s'", rxBytes, data); // 调试用
        }
    }
    free(data); // 释放内存（理论上不会到达此处）
}



