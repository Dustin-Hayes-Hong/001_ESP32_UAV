#include "flight_control.h"
#include "Data_declaration.h"

#include <stdio.h>
#include <stdbool.h>

#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <sys/time.h>

#include <inttypes.h>

#include "MPU6050.h"


#include "PID.h"

#include "PWM.h"


#include "SPL06-001.h"

setpoint_t setpoint; 

// 系统时间相关变量
struct timeval tv_now;         // 当前时间结构体，用于获取系统时间
int64_t time1_us = 0;          // 上次时间戳（微秒），用于时间差计算
int64_t time_us = 0;           // 当前时间戳（微秒），用于时间差计算

// 控制相关全局变量
float Pre_THROTTLE = 0.0f;     // 上一次油门值，用于平滑处理（未使用，可优化）
float THROTTLE = 0.0f;         // 当前油门值（未直接使用，可优化）
float Moto_PWM_1 = 0.0f;       // 电机1 PWM 输出值
float Moto_PWM_2 = 0.0f;       // 电机2 PWM 输出值
float Moto_PWM_3 = 0.0f;       // 电机3 PWM 输出值
float Moto_PWM_4 = 0.0f;       // 电机4 PWM 输出值
uint8_t SI24R1_Controlflag = 1;// SI24R1 控制标志（未使用，可优化）
uint8_t Airplane_Enable = 0;   // 飞机使能标志（未使用，可优化）

int mun = 0;                   // 电机启动计数器，用于低油门时逐步启动
float rc_yaw = 0.0f;           // 遥控器输入的偏航角累积值
float control_yaw = 0.0f;      // 控制参考偏航角，用于解锁时清零
bool lock_p = false;           // 解锁状态标志，记录是否刚解锁


/*****************************************************************************
 * 函数名：Control
 * 功能：飞行器姿态与电机控制核心函数，基于角度环和角速度环 PID 控制
 * 参数：
 *   att_in: 系统状态输入（包含姿态数据）
 *   gyr_in: 传感器数据输入（包含陀螺仪数据）
 *   rc_in: 遥控器设定值输入（包含油门和目标角度）
 * 返回值：无
 *****************************************************************************/
void Control(state_t *att_in, sensorData_t *gyr_in, setpoint_t *rc_in)
{
    attitude_t Measure_Angle;  // 当前测量角度
    attitude_t Target_Angle;   // 目标设定角度

    // 获取当前姿态角度并调整偏航角基准
    Measure_Angle.roll = att_in->attitude.roll;
    Measure_Angle.pitch = att_in->attitude.pitch;
    Measure_Angle.yaw = att_in->attitude.yaw - control_yaw;

    // 将遥控器输入值（通常 1000-2000）转换为角度（±范围）
    Target_Angle.roll = (rc_in->attitude.roll - 1500) / 35.0f;
    Target_Angle.pitch = (rc_in->attitude.pitch - 1500) / 35.0f;
    Target_Angle.yaw = (1500 - rc_in->attitude.yaw) / 35.0f;

    // 偏航角微调：当目标偏航角较大时，缓慢调整 rc_yaw
    if (Target_Angle.yaw > 1.0f || Target_Angle.yaw < -1.0f)
        rc_yaw -= Target_Angle.yaw / 100.0f;

    // 角度环 PID 计算（输入：角度，输出：目标角速度）
    PID_Postion_Cal(&PID_ROL_Angle, Target_Angle.roll, Measure_Angle.roll);
    PID_Postion_Cal(&PID_PIT_Angle, Target_Angle.pitch, Measure_Angle.pitch);
    PID_Postion_Cal(&PID_YAW_Angle, Target_Angle.yaw, Measure_Angle.yaw);

    // 角速度环 PID 计算（输入：角度环输出，输出：电机控制量）
    PID_Postion_Cal(&PID_ROL_Rate, PID_ROL_Angle.OutPut, gyr_in->gyro_f.Y * RadtoDeg);
    PID_Postion_Cal(&PID_PIT_Rate, PID_PIT_Angle.OutPut, gyr_in->gyro_f.X * RadtoDeg);
    PID_Postion_Cal(&PID_YAW_Rate, PID_YAW_Angle.OutPut, gyr_in->gyro_f.Z * RadtoDeg);

    // 动力分配，只有在飞机解锁时生效
    if (att_in->isRCLocked)
    {
        // 刚解锁时重置偏航角基准
        if (!lock_p) {
            control_yaw = att_in->attitude.yaw;
            rc_yaw = 0.0f;
            lock_p = true;
        }

        // 油门大于阈值时正常分配动力
        if (rc_in->thrust > 20) {
            Moto_PWM_1 = rc_in->thrust + PID_ROL_Rate.OutPut + PID_PIT_Rate.OutPut - PID_YAW_Rate.OutPut;
            Moto_PWM_2 = rc_in->thrust - PID_ROL_Rate.OutPut + PID_PIT_Rate.OutPut + PID_YAW_Rate.OutPut;
            Moto_PWM_3 = rc_in->thrust - PID_ROL_Rate.OutPut - PID_PIT_Rate.OutPut - PID_YAW_Rate.OutPut;
            Moto_PWM_4 = rc_in->thrust + PID_ROL_Rate.OutPut - PID_PIT_Rate.OutPut + PID_YAW_Rate.OutPut;
        } else {
            // 低油门时逐步启动电机，避免突然全停
            mun++;
            Moto_PWM_1 = (mun >= 1)  ? 5.0f : 0.0f;
            Moto_PWM_2 = (mun >= 10) ? 5.0f : 0.0f;
            Moto_PWM_3 = (mun >= 20) ? 5.0f : 0.0f;
            Moto_PWM_4 = (mun >= 30) ? 5.0f : 0.0f;
            if (mun >= 40) mun = 40; // 限制计数器上限
        }
    } else {
        // 未解锁时停止所有电机并重置计数器
        Moto_PWM_1 = 0.0f;
        Moto_PWM_2 = 0.0f;
        Moto_PWM_3 = 0.0f;
        Moto_PWM_4 = 0.0f;
        mun = 0;
        lock_p = false;
    }

    // 输出 PWM 信号到电机
    Moto_Pwm(Moto_PWM_1, Moto_PWM_2, Moto_PWM_3, Moto_PWM_4);
}

/*****************************************************************************
 * 函数名：printf_time1_us
 * 功能：打印系统时间或时间差（微秒），用于调试和性能分析
 * 参数：
 *   set: 0 表示记录初始时间，1 表示计算并打印时间差
 * 返回值：无
 *****************************************************************************/
void printf_time1_us(uint8_t set)
{
    gettimeofday(&tv_now, NULL);
    if (set == 0) {
        time1_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
    } else {
        time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
        printf("%" PRId64 "       ", time_us - time1_us);
    }
}

/*****************************************************************************
 * 函数名：angle_control_Task
 * 功能：姿态控制任务，周期性更新传感器数据并执行控制
 * 参数：
 *   pvParameter: 任务参数（未使用）
 * 返回值：无
 *****************************************************************************/
void angle_control_Task(void *pvParameter)
{
    const TickType_t period = 10; // 任务周期（ticks），约 10ms（取决于系统时钟）
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&lastWakeTime, period); // 确保固定周期运行
        if(init_ok){
            //Height_Get();                   // 获取高度数据
            mpu6050_read_data(&sensorData); // 读取 MPU6050 原始数据
            prepare_data(&sensorData);      // 数据滤波和单位转换
            imu_update(&sensorData, &state); // 通过四元数计算欧拉角
            Control(&state, &sensorData, &setpoint); // 执行姿态控制

        // 可选调试输出（已注释）
        // printf_time1_us(1);
        // printf("roll:=%0.2f pitch:=%0.2f yaw:=%0.2f\n", state.attitude.roll, state.attitude.pitch, state.attitude.yaw);
    }
    }
}

/*****************************************************************************
 * 函数名：control_init
 * 功能：初始化控制任务，创建姿态控制线程
 * 参数：无
 * 返回值：无
 *****************************************************************************/
void control_init(void)
{
    xTaskCreate(angle_control_Task, "angle_control_Task", 1024 * 8, NULL, 24, NULL);
}