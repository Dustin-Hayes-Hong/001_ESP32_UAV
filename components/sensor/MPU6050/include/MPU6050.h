#ifndef __MPU6050_H
#define __MPU6050_H

#include <stdio.h>              //标准输入输出
#include <string.h>             //字符串操作

#define MPU6050_SENSOR_ADDR                 0x68        //MPU6050传感器地址
#define MPU6050_WHO_AM_I_REG_ADDR           0x75        //MPU6050传感器寄存器地址，通过读取这个地址来验证连接是否正常

typedef struct
{
		int16_t X;
		int16_t Y;
		int16_t Z;

} INT16_XYZ;

typedef struct
{
	float pressure;
	float temperature;
	float asl;
} baro_t;

typedef struct
{

		float X;
		float Y;
		float Z;

} FLOAT_XYZ;

typedef struct
{
	INT16_XYZ acc_n;		//加速度原始数值
	INT16_XYZ gyro_n;	//陀螺仪原始数值
	INT16_XYZ mag_n; 	//磁力计原始数值
	baro_t baro;	//气压计原始数值

	FLOAT_XYZ acc_f;		//加速度原始数值
	FLOAT_XYZ gyro_f;	//陀螺仪原始数值

} sensorData_t;

extern sensorData_t sensorData;

extern int mpu6050_test(void);
extern int mpu6050_init(void);
extern int mpu6050_read_data(sensorData_t *data);
int mpu6050_offset(INT16_XYZ *value, INT16_XYZ *offset, uint16_t sensivity);
#endif // __MPU6050_H