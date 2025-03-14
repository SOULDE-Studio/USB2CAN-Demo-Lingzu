// # Copyright (c) 2023-2025 TANGAIR 
// # SPDX-License-Identifier: Apache-2.0
#ifndef __Tangair_usb2can__
#define __Tangair_usb2can__

#include <assert.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <string>
#include <sys/mman.h>
#include <sys/timerfd.h>
#include <thread>
#include <sched.h>
#include <unistd.h>
#include "usb_can.h"


// 辅助函数
float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x, float x_min, float x_max, int bits);

// 灵足电机,此处为RS04参数
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -15.0f
#define V_MAX 15.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -120.0f
#define T_MAX 120.0f



#define PI (3.1415926f)

typedef struct
{
	uint8_t id;
	uint8_t mode;
	uint16_t exdata;
	uint8_t res;

	float position;
	float speed;
	float kp;
	float kd;
	float torque;

	// 位置限制
	float max_position;
	float min_position;


} Motor_CAN_Send_Struct;

typedef struct
{
	uint8_t master_id;
	uint8_t motor_id;
	uint8_t fault_message;
	/*未标�????
	bit5: HALL 编码故障,10000,16
	bit4: 磁编码故�????,01000,8
	bit3: 过温,00100,4
	bit2: 过流,00010,2
	bit1: 欠压故障,00001,1*/
	uint8_t motor_state;
	/*  0 : Reset 模式[复位]
		1 : Cali 模式[标定]
		2 : Motor 模式[运行]*/
	uint8_t mode;

	uint16_t current_position; //[0~65535]对应(-4π~4π)
	uint16_t current_speed;	   //[0~65535]对应(-15rad/s~15rad/s)
	uint16_t current_torque;   //[0~65535]对应�????-120Nm~120Nm�????
	uint16_t current_temp;	   // 当前温度：Temp(摄氏度）*10

	float current_position_f; //[0~65535]对应(-4π~4π)
	float current_speed_f;	  //[0~65535]对应(-15rad/s~15rad/s)与rpm的换算关系为：1rad/s=9.55rpm
	float current_torque_f;	  //[0~65535]对应�????-120Nm~120Nm�????
	float current_temp_f;	  // 当前温度：Temp(摄氏度）*10

	

} Motor_CAN_Recieve_Struct;

typedef struct
{
	Motor_CAN_Send_Struct ID_1_motor_send, ID_2_motor_send, ID_3_motor_send;
	Motor_CAN_Recieve_Struct ID_1_motor_recieve, ID_2_motor_recieve, ID_3_motor_recieve;

} USB2CAN_CAN_Bus_Struct;



class Tangair_usb2can
{
public:
	
	bool all_thread_done_;
	bool running_;

	void Spin();

  

	Tangair_usb2can();

	~Tangair_usb2can();

  
	// CAN设备0
	int USB2CAN0_;
	std::thread _CAN_RX_device_0_thread;
	void CAN_RX_device_0_thread();


	// CAN设备1
	int USB2CAN1_;
	std::thread _CAN_RX_device_1_thread;
	void CAN_RX_device_1_thread();

    int can_dev0_rx_count;
	int can_dev0_rx_count_thread;
	int can_dev1_rx_count;
	int can_dev1_rx_count_thread;


	//can发送测试线程
	std::thread _CAN_TX_test_thread;
	//can发送测试线程函数
	void CAN_TX_test_thread();


    int speed_input;
    //can键盘输入线程
	std::thread _keyborad_input;
	//can发送测试线程函数
	void keyborad_input();




	/*****************************************************************************************************/
	/*********************************       ***电机相关***      ***********************************************/
	/*****************************************************************************************************/
	// 电机基本操作变量
	FrameInfo txMsg_CAN = {
		.canID = 0,
		.frameType = EXTENDED,
		.dataLength = 8,
	};

	uint8_t Data_CAN[8];
	Motor_CAN_Send_Struct Motor_Data_Single;

	//接收暂存
	Motor_CAN_Recieve_Struct CAN_DEV0_RX;
	Motor_CAN_Recieve_Struct CAN_DEV1_RX;


	// 腿部数据
	USB2CAN_CAN_Bus_Struct USB2CAN0_CAN_Bus_1; // 模块0，can1
	USB2CAN_CAN_Bus_Struct USB2CAN0_CAN_Bus_2;  // 模块0，can2
	USB2CAN_CAN_Bus_Struct USB2CAN1_CAN_Bus_1;  // 模块1，can1
	USB2CAN_CAN_Bus_Struct USB2CAN1_CAN_Bus_2;	  // 模块1，can2



	//初始化
	void USB2CAN_CAN_Bus_inti_set(USB2CAN_CAN_Bus_Struct* Leg_Data);
	void USB2CAN_CAN_Bus_Init();

	void Motor_Enable(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data);

	void Motor_Disable(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data);

	void Motor_Zore(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data);

	void CAN_Send_Control(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data); // 运控�??,CAN1=CAN_TX_MAILBOX0,CAN2=CAN_TX_MAILBOX1

	void Motor_Passive_SET(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data);

	void ENABLE_ALL_MOTOR(int delay_us);

	void DISABLE_ALL_MOTOR(int delay_us);

	void ZERO_ALL_MOTOR(int delay_us);

	void PASSIVE_ALL_MOTOR(int delay_us);

	void CAN_TX_ALL_MOTOR(int delay_us);

private:


};

#endif

// # Copyright (c) 2023-2025 TANGAIR 
// # SPDX-License-Identifier: Apache-2.0
