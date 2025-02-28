#include "PID.h"
#include "IMU.h"

int roll_trim;    // 横滚角（Roll）遥控补偿值，用于校正横滚轴的零点偏移
int pitch_trim;   // 俯仰角（Pitch）遥控补偿值，用于校正俯仰轴的零点偏移
int yaw_trim;     // 航向角（Yaw）遥控补偿值，用于校正航向轴的零点偏移

// 角度环PID结构体实例，用于控制飞行器的角度稳定
PID_TYPE PID_ROL_Angle;  // 横滚角（Roll）角度环PID参数，控制横滚角度的稳定
PID_TYPE PID_PIT_Angle;  // 俯仰角（Pitch）角度环PID参数，控制俯仰角度的稳定
PID_TYPE PID_YAW_Angle;  // 航向角（Yaw）角度环PID参数，控制航向角度的稳定

// 角速度环PID结构体实例，用于控制飞行器的角速度稳定
PID_TYPE PID_ROL_Rate;   // 横滚角（Roll）角速度环PID参数，控制横滚轴的角速度
PID_TYPE PID_PIT_Rate;   // 俯仰角（Pitch）角速度环PID参数，控制俯仰轴的角速度
PID_TYPE PID_YAW_Rate;   // 航向角（Yaw）角速度环PID参数，控制航向轴的角速度

// 高度环PID结构体实例，用于控制飞行器的高度稳定
PID_TYPE PID_ALT_Rate;   // 高度速率环PID参数，控制高度变化速率的稳定
PID_TYPE PID_ALT;        // 高度环PID参数，控制绝对高度的稳定
/*****************************************************************************
 * 函数名：void PID_Postion_Cal(PID_TYPE* PID, float target, float measure)
 * 功能：位置式PID控制算法，用于计算控制输出
 * 参数：
 *       PID: PID参数结构体指针，包含P、I、D参数及相关变量
 *       target: 目标设定值
 *       measure: 当前实际测量值
 * 返回值：无
 * 备注：适用于角度环和角速度环的通用PID控制
 *****************************************************************************/
void PID_Postion_Cal(PID_TYPE* PID, float target, float measure)
{
    // 计算当前误差
    PID->Error = target - measure;
    // 计算微分项（当前误差与前一次误差之差）
    PID->Differ = PID->Error - PID->PreError;

    // 计算比例项输出
    PID->Pout = PID->P * PID->Error;
    // 计算积分项输出（带积分分离标志）
    PID->Iout = PID->Ilimit_flag * PID->I * PID->Integral;
    // 计算微分项输出
    PID->Dout = PID->D * PID->Differ;

    // 总输出 = 比例 + 积分 + 微分
    PID->OutPut = PID->Pout + PID->Iout + PID->Dout;

    // 飞机解锁且推力足够时才进行积分运算，防止起飞前过调
    if (state.isRCLocked == true )
    {
        // 积分分离：当测量值超出限制范围时禁用积分
        if (measure > PID->Ilimit || measure < -PID->Ilimit)
        {
            PID->Ilimit_flag = 0;  // 停止积分
        }
        else
        {
            PID->Ilimit_flag = 1;  // 启用积分
            PID->Integral += PID->Error;  // 误差累加进行积分
            
            // 积分限幅，防止积分项过大
            if (PID->Integral > PID->Irang)
                PID->Integral = PID->Irang;
            if (PID->Integral < -PID->Irang)
                PID->Integral = -PID->Irang;
        }
    }
    else
    {
        PID->Integral = 0;  // 未解锁时清零积分
    }

    // 保存当前误差作为下次计算的前次误差
    PID->PreError = PID->Error;
}

/*****************************************************************************
 * 函数名：void PidParameter_init(void)
 * 功能：初始化PID控制参数及相关变量
 * 参数：无
 * 返回值：无
 * 备注：包含姿态控制（ROLL/PITCH/YAW）和高度控制的PID参数配置
 *****************************************************************************/
void PidParameter_init(void)
{
    // 遥控器补偿值初始化
    roll_trim = 200;   // 横滚角补偿
    pitch_trim = 0;    // 俯仰角补偿
    yaw_trim = 0;      // 航向角补偿

    // ROLL轴参数（横滚）
    PID_ROL_Rate.Ilimit_flag = 0;  // 角速度环积分分离标志
    PID_ROL_Rate.Ilimit = 150;     // 角速度环积分分离范围
    PID_ROL_Rate.Irang = 1200;     // 角速度环积分限幅值
    PID_ROL_Rate.P = 1.700;        // 角速度环比例系数
    PID_ROL_Rate.I = 0.020;        // 角速度环积分系数
    PID_ROL_Rate.D = 0.600f;       // 角速度环微分系数
    
    PID_ROL_Angle.Ilimit_flag = 0; // 角度环积分分离标志
    PID_ROL_Angle.Ilimit = 35;     // 角度环积分分离范围
    PID_ROL_Angle.Irang = 200;     // 角度环积分限幅值
    PID_ROL_Angle.P = 1.500f;      // 角度环比例系数
    PID_ROL_Angle.I = 0.020f;      // 角度环积分系数
    PID_ROL_Angle.D = 0.050f;      // 角度环微分系数

    // PITCH轴参数（俯仰）
    PID_PIT_Rate.Ilimit_flag = 0;  // 角速度环积分分离标志
    PID_PIT_Rate.Ilimit = 150;     // 角速度环积分分离范围
    PID_PIT_Rate.Irang = 1200;     // 角速度环积分限幅值
    PID_PIT_Rate.P = 1.700;        // 角速度环比例系数
    PID_PIT_Rate.I = 0.020f;       // 角速度环积分系数
    PID_PIT_Rate.D = 0.600f;       // 角速度环微分系数
    
    PID_PIT_Angle.Ilimit_flag = 0; // 角度环积分分离标志
    PID_PIT_Angle.Ilimit = 35;     // 角度环积分分离范围
    PID_PIT_Angle.Irang = 200;     // 角度环积分限幅值
    PID_PIT_Angle.P = 1.500f;      // 角度环比例系数
    PID_PIT_Angle.I = 0.020f;      // 角度环积分系数
    PID_PIT_Angle.D = 0.050f;      // 角度环微分系数

    // YAW轴参数（航向）
    PID_YAW_Rate.Ilimit_flag = 0;  // 角速度环积分分离标志
    PID_YAW_Rate.Ilimit = 150;     // 角速度环积分分离范围
    PID_YAW_Rate.Irang = 1200;     // 角速度环积分限幅值
    PID_YAW_Rate.P = 5.000f;       // 角速度环比例系数
    PID_YAW_Rate.I = 0.200f;       // 角速度环积分系数
    PID_YAW_Rate.D = 0.050f;       // 角速度环微分系数
    
    PID_YAW_Angle.Ilimit_flag = 0; // 角度环积分分离标志
    PID_YAW_Angle.Ilimit = 35;     // 角度环积分分离范围
    PID_YAW_Angle.Irang = 200;     // 角度环积分限幅值
    PID_YAW_Angle.P = 0.200f;      // 角度环比例系数
    PID_YAW_Angle.I = 0.020f;      // 角度环积分系数
    PID_YAW_Angle.D = 0.000f;      // 角度环微分系数

    // 高度控制参数
    PID_ALT_Rate.Ilimit_flag = 0;  // 高度速率环积分分离标志
    PID_ALT_Rate.Ilimit = 0;       // 高度速率环积分分离范围
    PID_ALT_Rate.Irang = 0;        // 高度速率环积分限幅值
    PID_ALT_Rate.P = 0.100f;       // 高度速率环比例系数
    PID_ALT_Rate.I = 0.001f;       // 高度速率环积分系数
    PID_ALT_Rate.D = 0.010f;       // 高度速率环微分系数
    
    PID_ALT.Ilimit_flag = 0;       // 高度环积分分离标志
    PID_ALT.Ilimit = 100;          // 高度环积分分离范围
    PID_ALT.Irang = 200;           // 高度环积分限幅值
    PID_ALT.P = 0.210f;            // 高度环比例系数
    PID_ALT.I = 0.000f;            // 高度环积分系数
    PID_ALT.D = 0.600f;            // 高度环微分系数
}