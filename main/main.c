#include <stdio.h>
#include "IIC.h"    //先配置I2C，以便读取传感器数据
#include "MPU6050.h"
#include "SPL06-001.h"
#include "VBAT.h"   //配置完传感器，可以配置一下电池电压检测和LED灯
#include "LED.h"
void app_main(void)
{

    I2C_Init();
    mpu6050_test();
    mpu6050_init();
    mpu6050_read_data(&sensorData);
    spl06_init();
}