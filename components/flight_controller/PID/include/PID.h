#include <stdio.h>
#include "stdbool.h"



typedef struct PID
{
    float P;            // 比例系数 (Proportional)，控制比例项的增益
    float I;            // 积分系数 (Integral)，控制积分项的增益
    float D;            // 微分系数 (Derivative)，控制微分项的增益
    
    float Error;        // 当前误差，目标值与实际测量值之差，用于比例项计算
    float Integral;     // 积分项累计值，误差随时间的累加，用于积分控制
    float Differ;       // 微分项值，当前误差与前一次误差的差值，用于微分控制
    
    float PreError;     // 前一次误差值，用于计算微分项
    float PrePreError;  // 前前次误差值，可用于更高阶微分计算（视算法需求）
    
    float Ilimit;       // 积分分离阈值，当测量值超出此范围时禁用积分
    float Irang;        // 积分限幅值，限制积分项的最大和最小范围，防止过调
    
    float Pout;         // 比例项输出，P * Error 的计算结果
    float Iout;         // 积分项输出，I * Integral 的计算结果（受积分分离控制）
    float Dout;         // 微分项输出，D * Differ 的计算结果
    
    float OutPut;       // 总输出，Pout + Iout + Dout 的和，作为最终控制量
    
    uint8_t Ilimit_flag; // 积分分离标志，0表示禁用积分，1表示启用积分
} PID_TYPE;

extern int roll_trim;    // 横滚角（Roll）遥控补偿值，用于校正横滚轴的零点偏移
extern int pitch_trim;   // 俯仰角（Pitch）遥控补偿值，用于校正俯仰轴的零点偏移
extern int yaw_trim;     // 航向角（Yaw）遥控补偿值，用于校正航向轴的零点偏移

// 角度环PID结构体实例，用于控制飞行器的角度稳定
extern PID_TYPE PID_ROL_Angle;  // 横滚角（Roll）角度环PID参数，控制横滚角度的稳定
extern PID_TYPE PID_PIT_Angle;  // 俯仰角（Pitch）角度环PID参数，控制俯仰角度的稳定
extern PID_TYPE PID_YAW_Angle;  // 航向角（Yaw）角度环PID参数，控制航向角度的稳定

// 角速度环PID结构体实例，用于控制飞行器的角速度稳定
extern PID_TYPE PID_ROL_Rate;   // 横滚角（Roll）角速度环PID参数，控制横滚轴的角速度
extern PID_TYPE PID_PIT_Rate;   // 俯仰角（Pitch）角速度环PID参数，控制俯仰轴的角速度
extern PID_TYPE PID_YAW_Rate;   // 航向角（Yaw）角速度环PID参数，控制航向轴的角速度

// 高度环PID结构体实例，用于控制飞行器的高度稳定
extern PID_TYPE PID_ALT_Rate;   // 高度速率环PID参数，控制高度变化速率的稳定
extern PID_TYPE PID_ALT;        // 高度环PID参数，控制绝对高度的稳定

extern void PidParameter_init(void);
extern void PID_Postion_Cal(PID_TYPE* PID, float target, float measure);