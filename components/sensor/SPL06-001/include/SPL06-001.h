#ifndef __SPL06_001_H
#define __SPL06_001_H

#include <stdio.h>              //标准输入输出
#include <string.h>             //字符串操作

#define SPL06_001_SENSOR_ADDR                 0x10        //SPL06-001传感器地址
#define SPL06_001_WHO_AM_I_REG_ADDR           0x0D        //SPL06-001传感器寄存器地址，通过读取这个地址来验证连接是否正常
#define SPL06_COEFFICIENT_CALIB_REG		      0x10        //校准系数寄存器地址

#define SPL06_PRESSURE_MSB_REG			(0x00) //气压计数据寄存器地址

#define SPL06_CALIB_COEFFICIENT_LENGTH	(18)    //校准系数长度
#define SPL06_DATA_FRAME_SIZE			(6)     //数据帧长度

//测量速率是指每秒刷新多少次结果（测量速度）
//过采样率是指每次结果需要采集多少次数据（测量精度）
//这里的0-7分别代表着 1 2 4 8 16 32 64 128

#define Pressure_measurement_rate		   (4)     //气压测量速率   每秒4次
#define Pressure_oversampling_rate		   (6)      //气压过采样率   每次64

#define Temperature_measurement_rate	   (4)      //温度测量速率   每秒4次
#define Temperature_oversampling_rate	   (6)      //温度过采样率   每次64

typedef struct
{
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;
} spl06CalibCoefficient_t;  //SPL06-001校准系数

typedef struct {
    float h;        // 海拔高度 (米)
    float v;        // 垂直速度 (米/秒)  
} alt_bro;

extern int spl06_init(void);

#endif // __SPL06_001_H