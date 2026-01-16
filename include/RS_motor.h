#ifndef __RS_motor__
#define __RS_motor__

#include "usb_can.h"
#include <vector>
#include <cstring>
#include <thread>
#include <chrono>
#include <iostream>

// 灵足电机,此处为RS00参数，其他电机请自行修改或参考电机调试工具上显示的参数
//https://can.robotsfan.com/motor/robstride/desktop.html
#define P_MIN -12.57f
#define P_MAX 12.57f
#define V_MIN -33.0f
#define V_MAX 33.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -14.0f
#define T_MAX 14.0f

// 辅助函数
float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x, float x_min, float x_max, int bits);

typedef struct
{
	uint8_t master_id;
	uint8_t motor_id; // => 模块ID
	uint8_t fault_message;
	uint8_t motor_state;
	/*  0 : Reset 模式[复位]
		1 : Cali 模式[标定]
		2 : Motor 模式[运行]*/
	uint8_t mode;

	uint16_t current_position; //[0~65535]对应(-4π~4π)
	uint16_t current_speed;	   //[0~65535]对应(-33rad/s~33rad/s)
	uint16_t current_torque;   //[0~65535]对应(-14Nm~14Nm)
	uint16_t current_temp;	   // 当前温度：Temp(摄氏度）*10

	float current_position_f = 0.0f; //[0~65535]对应(-4π~4π)
	float current_speed_f = 0.0f;	  //[0~65535]对应(-33rad/s~33rad/s)与rpm的换算关系为：1rad/s=9.55rpm
	float current_torque_f;	  //[0~65535]对应(-14Nm~14Nm)
	float current_temp_f;	  // 当前温度：Temp(摄氏度）*10

	float motor_init_position = 0.0f;
	bool motor_is_initialized = false; //电机位置是否初始化标志位（位置初始化：防止电机初始化时转圈）
	bool motor_miscount = false;

} RS_Motor_Struct; // RS电机接收结构体

// 电机扩展CAN帧ID结构体
typedef struct ID_CAN_Struct{
	uint8_t id;
	uint16_t exdata; // 主机ID
	uint8_t mode;
} ID_CAN_Struct;

// 电机运控模式发送结构体
typedef struct Motor_PDControl_Struct{
	float Feedforward_Torque;
	float Tar_Position;
	float Tar_Velocity;
	float Kp;
	float Kd;
} Motor_PDControl_Struct;

class RS_Motor
{
public:

	// 电机ID获取函数
	void Get_MotorID(int32_t dev, uint8_t channel, uint32_t motor_id);

	// 设置电机ID函数
	void Set_MotorID(int32_t dev, uint8_t channel, uint32_t old_motor_id, uint32_t new_motor_id);

    // 电机使能和失能函数
	void Motor_Enable(int32_t dev, uint8_t channel, uint32_t motor_id);

	void Motor_Disable(int32_t dev, uint8_t channel, uint32_t motor_id);

	// 电机模式切换函数
	void Motor_Mode_Change(int32_t dev, uint8_t channel, uint32_t motor_id , uint8_t mode);

	// 电机PP模式相关函数
	void PP_Vel_Max_Set(int32_t dev, uint8_t channel, uint32_t motor_id , float velmax);

	void PP_Acc_Set(int32_t dev, uint8_t channel, uint32_t motor_id , float acc);

	void PP_Mode_Set(int32_t dev, uint8_t channel, uint32_t motor_id, float velmax, float acc, int delay_us);

	void PP_Angle_Set(int32_t dev, uint8_t channel, uint32_t motor_id , float angle);

	// 电机速度模式相关函数
	void VEL_Current_Max_Set(int32_t dev, uint8_t channel, uint32_t motor_id , float currentmax);

	void VEL_Acc_Max_Set(int32_t dev, uint8_t channel, uint32_t motor_id , float accmax);

	void VEL_Mode_Set(int32_t dev, uint8_t channel, uint32_t motor_id , float currentmax, float accmax, int delay_us);

	void VEL_Vel_Set(int32_t dev, uint8_t channel, uint32_t motor_id , float speed);

	// 电机零位设置函数
	void Motor_Zero_Set(int32_t dev, uint8_t channel, uint32_t motor_id);

	// 电机PD模式运行相关函数
	void Motor_PD_Control(int32_t dev, uint8_t channel, uint32_t motor_id, Motor_PDControl_Struct *Motor_PDControl, float position);

	// 电机读取当前角度函数
	float Angle_Read(int32_t dev, uint8_t channel, uint32_t motor_id);

    // 电机基本操作变量
	FrameInfo txMsg_CAN_Motor = {
		.canID = 0,
		.frameType = EXTENDED,
		.dataLength = 8,
	};

	// 电机ID结构体
	ID_CAN_Struct ID_CAN = {
		.id = 0x01,
		.exdata = 0xfd,
		.mode = 0,
	};

    // CAN帧数据域
	uint8_t Data_CAN[8] = {0, 0, 0, 0, 0, 0, 0, 0};

private:

};

#endif
