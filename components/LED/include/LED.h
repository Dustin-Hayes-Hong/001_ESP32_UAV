#ifndef __LED_H
#define __LED_H

#include <stdio.h>              //标准输入输出
#include <stdbool.h>
#include <unistd.h>

#define LED1  GPIO_NUM_16 //连接信号灯
#define LED2  GPIO_NUM_17 //电量低信号灯
#define LED3  GPIO_NUM_18 //状态指示灯

typedef struct
{
	uint32_t timestamp;	//时间戳

	float roll;
	float pitch;
	float yaw;
} attitude_t;

typedef struct
{
	attitude_t attitude;//欧拉角
	//quaternion_t attitudeQuaternion;//四元数
	//point_t position; //位置
	//velocity_t velocity;//速度
	//acc_t acc; //角速度
	bool rc_link;
	int airplane_mode; //飞行器当时模式
	bool isRCLocked;//遥控器锁定
} state_t;

#endif // __LED_H