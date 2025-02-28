#include "IIC.h"    //先配置I2C，以便读取传感器数据
#include "MPU6050.h"
#include "SPL06-001.h"
#include "VBAT.h"   //配置完传感器，可以配置一下电池电压检测和LED灯
#include "LED.h"
#include "PWM.h"    //配置PWM输出用于控制电机，接着是姿态解算，以及PID
#include "Data_declaration.h"
#include "WIFI.h"
#include "UDP.h"
#include "remote_control.h"
#include "anotc.h"
#include "flight_control.h"

void app_main(void)
{

    I2C_Init();
	PWM_init();//PWM初始化
	VBAT_init();//电池电压检测初始化

	mpu6050_init();//MPU6050姿态传感器初始化
	//spl06_init(); //气压计初始化
	led_init();//LED灯初始化

	WIFI_Init();//初始化WiFi
	UDP_init();//UDP通信初始化
	anotc_Init();//匿名上位机初始化
	remote_control_init();//遥控器初始化

	control_init();//姿态控制初始化

	init_ok = true;//初始化完成置1让所有任务开始运行
}