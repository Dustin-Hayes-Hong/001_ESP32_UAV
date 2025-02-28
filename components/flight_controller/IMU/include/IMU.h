#ifndef __IMU_H
#define __IMU_H

#include <stdio.h>
#include <stdbool.h>
#include "math.h"
#include "MPU6050.h"

#define G      9.80665f            // m/s^2
#define RadtoDeg    57.324841f        //弧度到角度 (弧度 * 180/3.1415)
#define DegtoRad    0.0174533f        //角度到弧度 (角度 * 3.1415/180)

typedef struct
{
    uint32_t timestamp; // 时间戳，记录数据生成的时间（单位：毫秒）

    float roll;  // 横滚角（单位：度）
    float pitch; // 俯仰角（单位：度）
    float yaw;   // 偏航角（单位：度）
} attitude_t;

typedef struct
{
    attitude_t attitude; // 飞行姿态（欧拉角表示）
    // quaternion_t attitudeQuaternion; // 飞行姿态（四元数表示）
    // point_t position;    // 飞行器位置
    // velocity_t velocity; // 飞行器速度
    // acc_t acc;           // 角速度
    bool rc_link;        // 遥控器连接状态（true表示已连接）
    int airplane_mode;   // 飞行模式（数值对应特定模式）
    bool isRCLocked;     // 遥控器锁定状态（true表示已锁定）
} state_t;

extern state_t state;          //状态

extern void prepare_data(sensorData_t *sensor_data);
extern void imu_update(sensorData_t *sensor_data, state_t *state);

#endif
