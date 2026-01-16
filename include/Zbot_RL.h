#ifndef __Zbot_RL__
#define __Zbot_RL__

#include "RS_motor.h"
#include "Hipnuc_IMU.h"
#include <queue>
#include <mutex>
#include <unistd.h> 
#include <thread>
#include <fstream> 
#include <iomanip> 
#include <iostream>
#include <algorithm>
#include <signal.h>
// Optional libtorch include; use -DUSE_LIBTORCH OFF when building without LibTorch
#ifdef USE_LIBTORCH
#include <torch/script.h>
#endif

#define PI (3.1415926f)

#define Delay_0us 0
#define Delay_75us 75
#define Delay_100us 100
#define Delay_150us 150
#define Delay_300us 300
#define Delay_500us 500
#define Delay_1000us 1000
#define Delay_10000us 10000

#define PD_MODE 0
#define PP_MODE 1

// 读取csv文件，返回二维float向量
std::vector<std::vector<float>> loadCSV_invec(const std::string& path);

typedef struct
{
	uint8_t module_id; // 模块ID
	
    RS_Motor_Struct Motor_Recieve; // 电机部分

	Hipnuc_IMU_Struct IMU_Recieve; // IMU部分

} Module_CAN_Recieve_Struct; // zbot模块接收结构体

typedef struct
{
	std::vector<Module_CAN_Recieve_Struct> Module_CAN_Recieve; 

} USB2CAN_Dev_Struct; // USB转CAN设备接收结构体（包含多个zbot模块的数据）

class Zbot_RL
{ 
public:

    void Spin();

	Zbot_RL();

	~Zbot_RL();

    void ALL_Motor_ENABLE(int delay_us);

	void ALL_Motor_DISABLE(int delay_us);

    void ALL_Motor_PP_Mode_Set(int delay_us);

    void ALL_Motor_PP_Angle_Set(int delay_us, std::vector<float> motor_angles);

	void ALL_Motor_PP_Init(std::vector<float> motor_angles);

    void ALL_Motor_Zero_Set(int delay_us);

    void ALL_Motor_PD_Init(std::vector<float> motor_angles);

	void ALL_Motor_PD_Control(int delay_us, std::vector<float> motor_angles);

    // 清理接收缓存函数
	void Read_Clear(int dev, int num);

	// 获取当前时间戳
	float getTimestamp();

private:

    // ************************************************ 线程标志位 ************************************************ //
	
	bool all_thread_done_;
	bool running_;

	// ************************************************ USB2CAN设备 ************************************************ //
	
	int USB2CAN0_;

	// ************************************************ 初始化参数 ************************************************ //

	int Motor_Ctrl_Mode;
	
	std::vector<float> zero_angles = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; // 初始角度--摆直
	// std::vector<float> init_angles = {0.312f, 0.837f, -2.02f, 2.02f, -0.837f, -0.312f}; // 初始角度--站立
	std::vector<float> init_angles = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	float range = 1.0f * PI;
	// std::vector<float> lower_limit = {0.312f - range, 0.837f - range, -2.02f - range, 2.02f - range, -0.837f - range, -0.312f - range}; // 电机运行范围
	// std::vector<float> upper_limit = {0.312f + range, 0.837f + range, -2.02f + range, 2.02f + range, -0.837f + range, -0.312f + range}; // 电机运行范围
	std::vector<float> lower_limit = {- range, - range, - range, - range, - range, - range}; // 电机运行范围
	std::vector<float> upper_limit = {range, range, range, range, range, range}; // 电机运行范围

	std::vector<float> Q_meas_init = {1.0f, 0.0f, 0.0f, 0.0f}; // IMU初始测量四元数
	// Eigen::Quaternionf Q_desired{0.6003f, -0.6003f, -0.3735f, -0.3739f}; // (w, x, y, z) // 注意Eigen中四元数赋值的顺序，实数w在首；但是实际上它的内部存储顺序是[x y z w]
	Eigen::Quaternionf Q_desired{0.7070f, 0.0f, -0.7070f, 0.0f}; // (w, x, y, z) // 注意Eigen中四元数赋值的顺序，实数w在首；但是实际上它的内部存储顺序是[x y z w]
    Eigen::Quaternionf Q_offset{Eigen::Quaternionf::Identity()};

	bool motor_zero_set_already; // 是否进行过零点设置

    Motor_PDControl_Struct Zbot1234_RL_PD = {
        .Feedforward_Torque = 0.0f,
		.Tar_Position = 0.0f,
		.Tar_Velocity = 0.0f,
		.Kp = 20.0f,
		.Kd = 2.0f,
    };
	Motor_PDControl_Struct Zbot5_RL_PD = {
        .Feedforward_Torque = 0.0f,
		.Tar_Position = 0.0f,
		.Tar_Velocity = 0.0f,
		.Kp = 20.0f,
		.Kd = 2.0f,
    };
	Motor_PDControl_Struct Zbot6_RL_PD = {
        .Feedforward_Torque = 0.0f,
		.Tar_Position = 0.0f,
		.Tar_Velocity = 0.0f,
		.Kp = 20.0f,
		.Kd = 2.0f,
    };
	// Motor_PDControl_Struct Zbot5_RL_PD = {
    //     .Feedforward_Torque = 0.0f,
	// 	.Tar_Position = 0.0f,
	// 	.Tar_Velocity = 0.0f,
	// 	.Kp = 25.0f,
	// 	.Kd = 3.0f,
    // };
	// Motor_PDControl_Struct Zbot6_RL_PD = {
    //     .Feedforward_Torque = 0.0f,
	// 	.Tar_Position = 0.0f,
	// 	.Tar_Velocity = 0.0f,
	// 	.Kp = 70.0f,
	// 	.Kd = 5.0f,
    // };

	// ************************************************ 接收线程相关变量和成员 ************************************************ //
	
	std::thread _CAN_RX_device_0_thread;
	void CAN_RX_device_0_thread();

    int can_dev0_rx_count;
	int can_dev0_rx_count_thread;

	// 统一暂存ID
	uint8_t TEMP_ID = 0;

    // CAN转USB设备-接收数据结构体，每个结构体对应不同接收线程，包含6个模块的电机和IMU数据
	USB2CAN_Dev_Struct DEV0_RX = { std::vector<Module_CAN_Recieve_Struct>(6) }; // 包含ID:1～6 的模块数据

	// ************************************************ 发送线程相关变量和成员 ************************************************ //
	
	std::thread _CAN_TX_thread;
	void CAN_TX_thread();

	std::vector<float> motor_angles = init_angles; // 发送到电机的目标角度（rad）

	// ************************************************ 策略线程相关变量和成员 ************************************************ //

	std::thread _strategy_thread;
	void Strategy_thread();

	std::vector<float> out_last = {-1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f}; // 策略需要的输入--上一次的策略输出百分比

	std::vector<float> position_output = init_angles; // 策略计算的输出--电机绝对位置（rad）

#ifdef USE_LIBTORCH
	// LibTorch 模型（TorchScript）
	std::shared_ptr<torch::jit::script::Module> policy_model;
	bool model_loaded = false;
	// at::Tensor output_tensor = torch::zeros({6});
	at::Tensor relative_tensor = torch::tensor({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f});
#endif

	// 读取csv_invec相关变量
	std::vector<std::vector<float>> csv_data;
	int csv_index = 0;

	// ************************************************ 键盘交互线程相关变量和成员 ************************************************ //
	std::thread _keyborad_input;
	void keyborad_input();

	// float command_input = 1.5f; // laydown
	float command_input = 1.0f; // walking


	// ************************************************ 日志线程相关变量和成员 ************************************************ //

	std::thread _log_thread;
	void Log_thread();

	struct LogFrame {
		float timestamp = 0.0;
		float imu_quat[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // (w,x,y,z)
		float motor_pos[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
		float motor_vel[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	};

	std::queue<LogFrame> log_rx_queue;
	std::queue<LogFrame> log_tx_queue;
	std::queue<LogFrame> log_strategy_queue;

	// ************************************************ 线程锁 ************************************************ //

	std::mutex mutex_DEV0_RX;
	std::mutex mutex_command_input;
	std::mutex mutex_position_output; 
	std::mutex mutex_log_tx_queue;
	std::mutex mutex_log_rx_queue;
	std::mutex mutex_log_strategy_queue;

};

#endif
