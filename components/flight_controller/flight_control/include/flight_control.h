#include <stdint.h>
#include "IMU.h"

/*****************************************************************************
 * 结构体名：vec3_s
 * 功能：通用三轴向量结构体，用于表示位置、速度、加速度等物理量
 *****************************************************************************/
typedef struct vec3_s {
    uint32_t timestamp; // 时间戳（单位：通常为毫秒或微秒，视系统需求）
    float x;            // X 轴分量
    float y;            // Y 轴分量
    float z;            // Z 轴分量
} vec3_s;

/*****************************************************************************
 * 枚举名：mode_e
 * 功能：定义控制模式的枚举类型，用于指定各轴的控制方式
 *****************************************************************************/
typedef enum {
    MODE_DISABLE = 0,  // 关闭模式：不启用控制
    MODE_ABS,          // 绝对值模式：控制到绝对位置或角度
    MODE_VELOCITY      // 速率模式：控制速度或角速度
} mode_e;

/*****************************************************************************
 * 结构体名：mode_b
 * 功能：存储各轴和姿态的控制模式配置
 *****************************************************************************/
typedef struct {
    mode_e x;      // X 轴控制模式
    mode_e y;      // Y 轴控制模式
    mode_e z;      // Z 轴控制模式
    mode_e roll;   // 横滚角控制模式
    mode_e pitch;  // 俯仰角控制模式
    mode_e yaw;    // 偏航角控制模式
} mode_b;

// 类型别名定义，便于区分不同物理量的使用场景
typedef vec3_s point_t;     // 位置（单位：米）
typedef vec3_s velocity_t;  // 速度（单位：米/秒）
typedef vec3_s acc_t;       // 加速度（单位：米/秒²）

/*****************************************************************************
 * 结构体名：setpoint_t
 * 功能：飞行器设定点结构体，包含目标姿态、位置、速度及控制模式
 *****************************************************************************/
typedef struct {
    attitude_t attitude;       // 目标姿态（单位：度）
    attitude_t attitudeRate;   // 目标角速度（单位：度/秒）
    point_t position;          // 目标位置（单位：米）
    velocity_t velocity;       // 目标速度（单位：米/秒）
    mode_b mode;               // 各轴控制模式配置
    float aux1;                // 辅助通道 1（用途待定，例如开关或额外控制）
    float aux2;                // 辅助通道 2
    float aux3;                // 辅助通道 3
    float aux4;                // 辅助通道 4
    float aux5;                // 辅助通道 5
    float aux6;                // 辅助通道 6
    float thrust;              // 油门设定值（单位：通常为 PWM 或百分比）
} setpoint_t;

extern setpoint_t setpoint; 

extern void control_init(void);