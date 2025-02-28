/**
 * @file remote_control.c
 * @brief 遥控器数据解码与发送模块
 * @details 
 * 本模块实现无人机与遥控器之间的数据通信功能，包括接收遥控器指令（解码）和发送飞行器状态数据。
 * 使用 UDP 协议进行数据传输，支持姿态、油门和电池电压的实时交互。
 * @author [Your Name]
 * @date [Current Date]
 */
#include "remote_control.h"
#include "Data_declaration.h"
#include "UART.h"
#include "UDP.h"
#include "IMU.h"
#include "flight_control.h"
#include "VBAT.h"

#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "PID.h"
#include "SPL06-001.h"

// 数据发送缓冲区
uint8_t rc_data_to_send[50]; // 发送数据缓存，初始化为 0，避免未定义行为

// 数据拆分宏定义，用于将多字节数据（如 float、uint32_t）拆分为单独字节
#define BYTE0(dwTemp)  (*((uint8_t *)(&dwTemp)))      // 低字节
#define BYTE1(dwTemp)  (*((uint8_t *)(&dwTemp) + 1))  // 次低字节
#define BYTE2(dwTemp)  (*((uint8_t *)(&dwTemp) + 2))  // 次高字节
#define BYTE3(dwTemp)  (*((uint8_t *)(&dwTemp) + 3))  // 高字节

// 全局变量
static int isRCLocked_p = 0; // 上一次解锁状态，用于检测上升沿

/**
 * @brief 解码遥控器发送的数据
 * @details 
 * 解析接收到的数据包，提取姿态、油门和解锁状态，并应用遥控器补偿值。
 * 数据格式：帧头(0xBB 0xBB) + 数据长度 + 数据 + 校验和
 * @param data 接收到的数据缓冲区
 */
void rc_data_decode(char *data)
{
	uint8_t sum = 0; // 校验和
	uint16_t temp;   // 临时变量，用于字节合并

	// 检查帧头（0xBB 0xBB）
	if (data[0] != 0xBB || data[1] != 0xBB) {
		return; // 帧头错误，丢弃数据
	}

	// 计算校验和（帧头 + 数据长度 + 数据）
	for (int i = 0; i < data[3] + 4; i++) {
		sum += (uint8_t)data[i];
	}
	if (sum != (uint8_t)data[data[3] + 4]) {
		return; // 校验失败，丢弃数据
	}

	state.rc_link = true; // 遥控器连接状态置为有效

	// 解码姿态数据（16 位，高字节在前）
	temp = ((uint8_t)data[4] << 8) | (uint8_t)data[5];
	setpoint.attitude.roll = temp + roll_trim;

	temp = ((uint8_t)data[6] << 8) | (uint8_t)data[7];
	setpoint.attitude.pitch = (1500 - (temp - 1500)) + pitch_trim; // 反转并校准

	temp = ((uint8_t)data[8] << 8) | (uint8_t)data[9];
	setpoint.attitude.yaw = temp + yaw_trim;

	temp = ((uint8_t)data[10] << 8) | (uint8_t)data[11];
	setpoint.thrust = temp;

	// 检测解锁信号的上升沿
	if (data[25] == 1 && isRCLocked_p == 0) {
		state.isRCLocked = true; // 上升沿触发解锁
	}
	if (data[25] == 0) {
		state.isRCLocked = false; // 下降沿触发锁定
	}
	isRCLocked_p = data[25]; // 更新前次状态
}

/**
 * @brief 将浮点数转换为 32 位无符号整数
 * @details 
 * 将 float 的内存表示直接转换为 uint32_t，用于字节序转换。
 * @param dat 输入浮点数
 * @return uint32_t 转换后的 32 位整数
 */
static uint32_t float_to_u32(float dat)
{
	union {
		float f;
		uint32_t u;
	} converter;
	converter.f = dat;
	return converter.u;
}

/**
 * @brief 向遥控器发送飞行器状态数据
 * @details 
 * 将姿态角、高度和电池电压打包并通过 UDP 发送，数据格式为：
 * 帧头(0xBB 0xBB) + 功能字(0x01) + 数据长度 + 数据 + 校验和
 * @param angle_rol 横滚角 (deg)
 * @param angle_pit 俯仰角 (deg)
 * @param angle_yaw 偏航角 (deg)
 * @param alt 高度 (m)
 * @param vbat 电池电压 (mV)
 */
void RC_Send_Status(float angle_rol, float angle_pit, float angle_yaw, float alt, uint32_t vbat)
{
	uint8_t cnt = 0;  // 数据缓冲区索引
	uint8_t sum = 0;  // 校验和
	uint32_t temp;    // 临时变量

	// 填充帧头和功能字
	rc_data_to_send[cnt++] = 0xBB; // 帧头第 1 字节
	rc_data_to_send[cnt++] = 0xBB; // 帧头第 2 字节
	rc_data_to_send[cnt++] = 0x01; // 功能字，表示状态数据
	rc_data_to_send[cnt++] = 0;    // 数据长度，稍后填充

	// 填充横滚角
	temp = float_to_u32(angle_rol);
	rc_data_to_send[cnt++] = BYTE3(temp);
	rc_data_to_send[cnt++] = BYTE2(temp);
	rc_data_to_send[cnt++] = BYTE1(temp);
	rc_data_to_send[cnt++] = BYTE0(temp);

	// 填充俯仰角
	temp = float_to_u32(angle_pit);
	rc_data_to_send[cnt++] = BYTE3(temp);
	rc_data_to_send[cnt++] = BYTE2(temp);
	rc_data_to_send[cnt++] = BYTE1(temp);
	rc_data_to_send[cnt++] = BYTE0(temp);

	// 填充偏航角
	temp = float_to_u32(angle_yaw);
	rc_data_to_send[cnt++] = BYTE3(temp);
	rc_data_to_send[cnt++] = BYTE2(temp);
	rc_data_to_send[cnt++] = BYTE1(temp);
	rc_data_to_send[cnt++] = BYTE0(temp);

	// 填充高度
	temp = float_to_u32(alt);
	rc_data_to_send[cnt++] = BYTE3(temp);
	rc_data_to_send[cnt++] = BYTE2(temp);
	rc_data_to_send[cnt++] = BYTE1(temp);
	rc_data_to_send[cnt++] = BYTE0(temp);

	// 填充电池电压
	temp = vbat;
	rc_data_to_send[cnt++] = BYTE3(temp);
	rc_data_to_send[cnt++] = BYTE2(temp);
	rc_data_to_send[cnt++] = BYTE1(temp);
	rc_data_to_send[cnt++] = BYTE0(temp);

	// 计算数据长度（总长度 - 帧头/功能字/长度字节）
	rc_data_to_send[3] = cnt - 4;

	// 计算校验和
	for (int i = 0; i < cnt; i++) {
		sum += rc_data_to_send[i];
	}
	rc_data_to_send[cnt++] = sum;

	// 通过 UDP 发送数据
	UDP_write_rc(rc_data_to_send, cnt);
}

/**
 * @brief 遥控器数据发送任务
 * @details 
 * 周期性（10ms）发送飞行器状态数据给遥控器，仅在系统初始化完成后执行。
 * @param pvParameters 任务参数（未使用）
 */
void remote_control_task(void *pvParameters)
{
	const TickType_t period = 10; // 任务周期 10ms
	TickType_t last_wake_time = xTaskGetTickCount();

	while (1) {
		vTaskDelayUntil(&last_wake_time, period);
		if (init_ok) {
			RC_Send_Status(state.attitude.roll, state.attitude.pitch, state.attitude.yaw, ALT_BRO.h, VBAT);
		}
	}
}

/**
 * @brief 初始化遥控器通信模块
 * @details 
 * 创建遥控器数据发送任务，设置堆栈大小为 4096 字节，优先级为 6。
 */
void remote_control_init(void)
{
	xTaskCreate(remote_control_task, "remote_control_task", 4096, NULL, 6, NULL);
}
