#ifndef __UART_H
#define __UART_H

#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"

static const int RX_BUF_SIZE = 1024; // 接收缓冲区大小（字节）

#define TXD_PIN 43 // 发送引脚 (GPIO 43)
#define RXD_PIN 44 // 接收引脚 (GPIO 44)

/**
 * @brief 初始化UART1
 * @param baud 波特率
 */
void uart1_init(int baud);

/**
 * @brief UART发送任务
 * @param arg 任务参数
 */
void tx_task(void *arg);

/**
 * @brief UART接收任务
 * @param arg 任务参数
 */
void rx_task(void *arg);

/**
 * @brief 发送数据
 * @param data 要发送的数据字符串
 * @return 发送的字节数，错误时返回负值
 */
int sendData(const char *data);

#endif //__UART_H
