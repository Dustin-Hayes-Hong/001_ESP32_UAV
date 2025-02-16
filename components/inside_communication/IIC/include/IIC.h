#ifndef __IIC_H
#define __IIC_H

#include <stdio.h>              //标准输入输出
#include <string.h>             //字符串操作
#include "driver/i2c_master.h"  //I2C驱动

#include "freertos/FreeRTOS.h"  //FreeRTOS
#include "freertos/task.h"      //FreeRTOS任务

#define I2C_MASTER_SCL_IO        1      					//IIC时钟线GPIO端口号 1  19
#define I2C_MASTER_SDA_IO        2      					//IIC数据线GPIO端口号 2   18
#define I2C_MASTER_NUM           0      					//IIC端口号(0或1)
#define I2C_MASTER_FREQ_HZ       1000000 					//IIC速率
#define I2C_MASTER_TX_BUF_SIZE   0      					//IIC发送缓冲区
#define I2C_MASTER_RX_BUF_SIZE   0      					//IIC接送缓冲区
#define I2C_MASTER_TIMEOUT_MS    1000 / portTICK_RATE_MS	//IIC超时时间

extern i2c_master_dev_handle_t dev1_handle; // 外部引用 dev1 配置
extern i2c_master_dev_handle_t dev2_handle; // 外部引用 dev2 配置

void I2C_Init(void);
int i2c_write(i2c_master_dev_handle_t handle,uint8_t *read_buffer,size_t read_size);
int i2c_read(i2c_master_dev_handle_t handle,uint8_t *write_buffer,size_t write_size);
int i2c_write_read(i2c_master_dev_handle_t handle,uint8_t *write_buffer,size_t write_size,uint8_t *read_buffer,size_t read_size);
int i2c_write_reg(i2c_master_dev_handle_t handle,uint8_t reg_addr,uint8_t data);

#endif // __IIC_H