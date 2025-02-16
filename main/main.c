#include <stdio.h>
#include "IIC.h"
#include "MPU6050.h"

void app_main(void)
{

    I2C_Init();
    mpu6050_test();
    mpu6050_init();
    mpu6050_read_data(&sensorData);
}