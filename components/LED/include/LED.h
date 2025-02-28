#ifndef __LED_H
#define __LED_H

#include <stdio.h>              //标准输入输出
#include <stdbool.h>
#include <unistd.h>



#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"//RTOS头文件
#include "freertos/task.h"    //RTOS头文件



#define LED1  GPIO_NUM_16 //连接信号灯
#define LED2  GPIO_NUM_17 //电量低信号灯
#define LED3  GPIO_NUM_18 //状态指示灯

extern void led_init(void);
#endif // __LED_H