#ifndef __Hipnuc_IMU__
#define __Hipnuc_IMU__

#include "usb_can.h"
#include <eigen3/Eigen/Geometry>
#include <chrono>
#include <thread>
#include <iostream>

typedef struct
{
	uint8_t IMU_id; // --> 模块ID + 0x20
	
    float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // 四元数(w, x, y, z)

} Hipnuc_IMU_Struct; // RS电机接收结构体

class Hipnuc_IMU 
{
public:

    void IMU_Set_SYNC_Mode(int32_t dev, uint8_t channel, uint8_t imu_id);

	void IMU_Send_SYNC(int32_t dev, uint8_t channel);

	void IMU_Get_init_quat(int32_t dev);

	void IMU_Get_offset_quat(Eigen::Quaternionf q_desired);

	void IMU_quat_correct(Hipnuc_IMU_Struct &IMU_data);

    // IMU基本操作变量
	FrameInfo txMsg_CAN_IMU = {
		.canID = 0,
		.frameType = STANDARD,
		.dataLength = 8,
	};

    // CAN帧数据域
	uint8_t Data_CAN[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    float Q_meas_init[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // IMU初始测量四元数
	
    Eigen::Quaternionf Q_offset{Eigen::Quaternionf::Identity()};
    
private:

};

#endif