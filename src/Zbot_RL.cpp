#include "Zbot_RL.h"

static class Hipnuc_IMU IMU;
static class RS_Motor Motor;

static time_t start_tp = 0;

// 主函数循环
void Zbot_RL::Spin()
{
    while (all_thread_done_ != true)
    {
        sleep(1); // 延时1s
    }
    printf("~ ALL Exit ~\n");
}

/// @brief 构造函数，初始化
/// @return
Zbot_RL::Zbot_RL()
{
    running_ = true;
    all_thread_done_ = false;
    motor_zero_set_already = true; // 注意是否进行过零点设置
    Motor_Ctrl_Mode = PD_MODE; // 选择电机控制模式

    std::cout << std::endl
            << "RUN Zbot_RL.cpp" << std::endl
            << std::endl;

    USB2CAN0_ = openUSBCAN("/dev/USB2CAN0");
    if (USB2CAN0_ == -1)
        std::cout << std::endl
                << "USB2CAN0 open INcorrect!!!" << std::endl;
    else
        std::cout << std::endl
                << "USB2CAN0 opened ,num=" << USB2CAN0_ << std::endl;

    // 启动成功
    std::cout << std::endl
            << "USB2CAN   NODE INIT__OK   by TANGAIR" << std::endl
            << std::endl
            << std::endl;

    // ********************************************************************** 初 始 化 ********************************************************************** //
    {
        if (motor_zero_set_already == false) // 电机未进行过零点设置 // 若未进行零点设置，切记将TX线程的运动控制代码注释
        {
            ALL_Motor_Zero_Set(Delay_1000us);
            sleep(1);
            ALL_Motor_ENABLE(Delay_1000us);
            sleep(1);
            Read_Clear(USB2CAN0_, 12); // 清理接收缓存
        }
        else if (motor_zero_set_already == true) // 电机进行过零点设置
        {
            if (Motor_Ctrl_Mode == PP_MODE)
            {
                ALL_Motor_PP_Init(init_angles); // 使能电机，切换到PP模式，运动到指定角度
                Read_Clear(USB2CAN0_, 30); // 清理接收缓存
            }

            else if (Motor_Ctrl_Mode == PD_MODE)
            {
                ALL_Motor_PD_Init(init_angles);
                Read_Clear(USB2CAN0_, 12); // 清理接收缓存
            }
        }

        IMU.IMU_Set_SYNC_Mode(USB2CAN0_, 1, 0x21); // 设置IMU为SYNC模式
        sleep(0.5);

        for (size_t i = 0; i < 5; i++) // 采集多次初始IMU四元数数据
        {
            IMU.IMU_Send_SYNC(USB2CAN0_, 1);
            IMU.IMU_Get_init_quat(USB2CAN0_);
        }
        IMU.IMU_Get_offset_quat(Q_desired); // 计算校正四元数，Q_desired为期望初始四元数
        sleep(1);
    }
    // ********************************************************************** 创 立 线 程 ********************************************************************** //
    // 读取csv作为策略输入
    csv_data = loadCSV_invec("../figures_data/data/obs_env_csv1219/v2_obs.csv");
    csv_index = 0;

    // 获取当前系统时间戳
    std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> tpMill =
        std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
    start_tp = tpMill.time_since_epoch().count();

    // 创建CAN接收线程，设备1
    _CAN_RX_device_0_thread = std::thread(&Zbot_RL::CAN_RX_device_0_thread, this);

    // CAN发送线程
    _CAN_TX_thread = std::thread(&Zbot_RL::CAN_TX_thread, this);

    // 键盘输入线程
    _keyborad_input = std::thread(&Zbot_RL::keyborad_input, this);
    
    // 策略线程（50Hz）
    _strategy_thread = std::thread(&Zbot_RL::Strategy_thread, this);

    // 记录线程（500Hz）
    _log_thread = std::thread(&Zbot_RL::Log_thread, this);

#ifdef USE_LIBTORCH
    // 尝试加载模型（可选），路径可在运行时替换
    try {
        policy_model = std::make_shared<torch::jit::script::Module>(torch::jit::load("/home/rain/libtorch/export_model/policy_standupsymmetry0.5.pt"));
        model_loaded = true;
        std::cout << "Policy model loaded." << std::endl;
    } catch (const std::exception &e) {
        model_loaded = false;
        std::cout << "Warning: failed to load policy model: " << e.what() << std::endl;
    }
#endif
    
}

/// @brief 析构函数
Zbot_RL::~Zbot_RL()
{

    running_ = false;

    /*注销线程*/

    // can接收设备0
    _CAN_RX_device_0_thread.join();

    //can发送测试线程
    _CAN_TX_thread.join();
    //键盘输入线程
    _keyborad_input.join();
    // 策略线程停止并join
    if (_strategy_thread.joinable())
        _strategy_thread.join();

    _log_thread.join();
    // 失能电机
    ALL_Motor_DISABLE(100);

    // 关闭设备
    closeUSBCAN(USB2CAN0_);

    all_thread_done_ = true;
}

/// @brief can设备0，接收线程函数
void Zbot_RL::CAN_RX_device_0_thread()
{
    can_dev0_rx_count = 0;
    can_dev0_rx_count_thread=0;
    
    while (running_)
    {

        uint8_t channel;
        FrameInfo info_rx;
        uint8_t data_rx[8] = {0};
        // LogFrame log_rx_data;

        can_dev0_rx_count_thread++;

        // 有数据不会阻塞，若无数据则等待1s
        int recieve_re = readUSBCAN(USB2CAN0_, &channel, &info_rx, data_rx, 1e6);

        // 接收到数据
        if (recieve_re != -1)
        {
            can_dev0_rx_count++;

            if (info_rx.frameType == STANDARD) // 读取IMU数据
            {
                TEMP_ID = ((info_rx.canID) & 0xfff) - 0x480;
                uint8_t index = TEMP_ID - 0x21;  // 将ID映射到0-2的索引,DEV0_RX包含ID21-ID23的IMU数据

                {
                    std::lock_guard<std::mutex> lock(mutex_DEV0_RX);
                    // 收到的数据低字节在前
                    DEV0_RX.Module_CAN_Recieve[index].IMU_Recieve.quat[0] = static_cast<float>(static_cast<int16_t>((data_rx[1] << 8) | data_rx[0])) / 10000.0f;
                    DEV0_RX.Module_CAN_Recieve[index].IMU_Recieve.quat[1] = static_cast<float>(static_cast<int16_t>((data_rx[3] << 8) | data_rx[2])) / 10000.0f;
                    DEV0_RX.Module_CAN_Recieve[index].IMU_Recieve.quat[2] = static_cast<float>(static_cast<int16_t>((data_rx[5] << 8) | data_rx[4])) / 10000.0f;
                    DEV0_RX.Module_CAN_Recieve[index].IMU_Recieve.quat[3] = static_cast<float>(static_cast<int16_t>((data_rx[7] << 8) | data_rx[6])) / 10000.0f;
                }
            }

            else if (info_rx.frameType == EXTENDED) // 读取电机数据
            {
                TEMP_ID = (info_rx.canID >> 8) & 0xff;
                uint8_t index = TEMP_ID - 0x01;  // 将ID映射到0-2的索引,DEV0_RX包含ID01-ID03的电机数据

                {
                    std::lock_guard<std::mutex> lock(mutex_DEV0_RX);
                    // 解码
                    DEV0_RX.Module_CAN_Recieve[index].Motor_Recieve.master_id = (info_rx.canID) & 0xff;
                    DEV0_RX.Module_CAN_Recieve[index].Motor_Recieve.motor_id = (info_rx.canID >> 8) & 0xff;
                    DEV0_RX.Module_CAN_Recieve[index].Motor_Recieve.fault_message = (info_rx.canID >> 16) & 0x3f;
                    DEV0_RX.Module_CAN_Recieve[index].Motor_Recieve.motor_state = (info_rx.canID >> 22) & 0x03;
                    DEV0_RX.Module_CAN_Recieve[index].Motor_Recieve.mode = (info_rx.canID >> 24) & 0x1f;

                    if (DEV0_RX.Module_CAN_Recieve[index].Motor_Recieve.mode == 0x02)
                    {
                        // 收到的数据高字节在前
                        DEV0_RX.Module_CAN_Recieve[index].Motor_Recieve.current_position = (data_rx[0] << 8) | (data_rx[1]);
                        DEV0_RX.Module_CAN_Recieve[index].Motor_Recieve.current_speed = (data_rx[2] << 8) | (data_rx[3]);
                        DEV0_RX.Module_CAN_Recieve[index].Motor_Recieve.current_torque = (data_rx[4] << 8) | (data_rx[5]);
                        DEV0_RX.Module_CAN_Recieve[index].Motor_Recieve.current_temp = (data_rx[6] << 8) | (data_rx[7]);

                        // 转换
                        DEV0_RX.Module_CAN_Recieve[index].Motor_Recieve.current_position_f = -uint_to_float(DEV0_RX.Module_CAN_Recieve[index].Motor_Recieve.current_position, (P_MIN), (P_MAX), 16); // 电机顺时针为角度增加，所以加负号
                        DEV0_RX.Module_CAN_Recieve[index].Motor_Recieve.current_speed_f = -uint_to_float(DEV0_RX.Module_CAN_Recieve[index].Motor_Recieve.current_speed, (V_MIN), (V_MAX), 16); // 电机顺时针为角度增加，所以加负号
                        DEV0_RX.Module_CAN_Recieve[index].Motor_Recieve.current_torque_f = -uint_to_float(DEV0_RX.Module_CAN_Recieve[index].Motor_Recieve.current_torque, (T_MIN), (T_MAX), 16); // 电机顺时针为角度增加，所以加负号
                        DEV0_RX.Module_CAN_Recieve[index].Motor_Recieve.current_temp_f = (float)DEV0_RX.Module_CAN_Recieve[index].Motor_Recieve.current_temp / 10;
                    }
                }
            }

            // float logtime_now = getTimestamp();

            // log_rx_data.timestamp = logtime_now;
            // {
            //     std::lock_guard<std::mutex> lock(mutex_DEV0_RX);

            //     log_rx_data.motor_pos[0] = DEV0_RX.Module_CAN_Recieve[0].Motor_Recieve.current_position_f;
            //     log_rx_data.motor_pos[1] = DEV0_RX.Module_CAN_Recieve[1].Motor_Recieve.current_position_f;
            //     log_rx_data.motor_pos[2] = DEV0_RX.Module_CAN_Recieve[2].Motor_Recieve.current_position_f;
            //     log_rx_data.motor_pos[3] = DEV0_RX.Module_CAN_Recieve[3].Motor_Recieve.current_position_f;
            //     log_rx_data.motor_pos[4] = DEV0_RX.Module_CAN_Recieve[4].Motor_Recieve.current_position_f;
            //     log_rx_data.motor_pos[5] = DEV0_RX.Module_CAN_Recieve[5].Motor_Recieve.current_position_f;

            //     log_rx_data.motor_vel[0] = DEV0_RX.Module_CAN_Recieve[0].Motor_Recieve.current_speed_f;
            //     log_rx_data.motor_vel[1] = DEV0_RX.Module_CAN_Recieve[1].Motor_Recieve.current_speed_f;
            //     log_rx_data.motor_vel[2] = DEV0_RX.Module_CAN_Recieve[2].Motor_Recieve.current_speed_f;
            //     log_rx_data.motor_vel[3] = DEV0_RX.Module_CAN_Recieve[3].Motor_Recieve.current_speed_f;
            //     log_rx_data.motor_vel[4] = DEV0_RX.Module_CAN_Recieve[4].Motor_Recieve.current_speed_f;
            //     log_rx_data.motor_vel[5] = DEV0_RX.Module_CAN_Recieve[5].Motor_Recieve.current_speed_f;
            // }

            // {
            //     std::lock_guard<std::mutex> lock(mutex_log_rx_queue);
            //     log_rx_queue.push(log_rx_data);
            // }
        }
    }
    std::cout << "CAN_RX_device_0_thread  Exit~~" << std::endl;
}

// can发送线程函数
void Zbot_RL::CAN_TX_thread()
{
    // 发送计数
    uint32_t tx_count = 0;
    LogFrame log_tx_data;

    while (running_)
    {
        // CAN发送计数
        tx_count++;

        IMU.IMU_Send_SYNC(USB2CAN0_, 1); // 发送IMU同步帧
        std::this_thread::sleep_for(std::chrono::microseconds(Delay_300us)); // 单位us

        // 使用策略线程计算得到的输出
        {
            std::lock_guard<std::mutex> lock(mutex_position_output); //线程锁
            motor_angles[0] = position_output[0];
            motor_angles[1] = position_output[1];
            motor_angles[2] = position_output[2];
            motor_angles[3] = position_output[3];
            motor_angles[4] = position_output[4];
            motor_angles[5] = position_output[5];
        }

        for (size_t i = 0; i < motor_angles.size(); ++i) {
            motor_angles[i] = std::clamp(motor_angles[i], lower_limit[i], upper_limit[i]);
        }

        if (Motor_Ctrl_Mode == PD_MODE)
        {
            ALL_Motor_PD_Control(Delay_300us, motor_angles);
            // ALL_Motor_PD_Control(Delay_300us, init_angles);
        }
        else if (Motor_Ctrl_Mode == PP_MODE)
        {
            ALL_Motor_PP_Angle_Set(Delay_300us, motor_angles);
            // ALL_Motor_PP_Angle_Set(Delay_300us, init_angles);
        }
    
        float time_now = getTimestamp();

        // 打印数据
        if (tx_count % 1000 == 0)
        {
            {
                std::lock_guard<std::mutex> lock(mutex_command_input);
                std::cout << " 键盘输入: " << command_input << std::endl;
            }
            {
                std::lock_guard<std::mutex> lock(mutex_position_output);
                std::cout << " 网络输出: " << position_output[0] << ", " << position_output[1] << ", " << position_output[2] << ", " 
                                      << position_output[3] << ", " << position_output[4] << ", " << position_output[5] << std::endl;
            }

            std::cout << " 输入电机1的角度: " << motor_angles[0] << std::endl;
            std::cout << " 输入电机2的角度: " << motor_angles[1] << std::endl;
            std::cout << " 输入电机3的角度: " << motor_angles[2] << std::endl;
            std::cout << " 输入电机4的角度: " << motor_angles[3] << std::endl;
            std::cout << " 输入电机5的角度: " << motor_angles[4] << std::endl;
            std::cout << " 输入电机6的角度: " << motor_angles[5] << std::endl;

            {
                std::lock_guard<std::mutex> lock(mutex_DEV0_RX);
                std::cout << " 电机1当前角度: " << DEV0_RX.Module_CAN_Recieve[0].Motor_Recieve.current_position_f << std::endl;
                std::cout << " 电机2当前角度: " << DEV0_RX.Module_CAN_Recieve[1].Motor_Recieve.current_position_f << std::endl;
                std::cout << " 电机3当前角度: " << DEV0_RX.Module_CAN_Recieve[2].Motor_Recieve.current_position_f << std::endl;
                std::cout << " 电机4当前角度: " << DEV0_RX.Module_CAN_Recieve[3].Motor_Recieve.current_position_f << std::endl;
                std::cout << " 电机5当前角度: " << DEV0_RX.Module_CAN_Recieve[4].Motor_Recieve.current_position_f << std::endl;
                std::cout << " 电机6当前角度: " << DEV0_RX.Module_CAN_Recieve[5].Motor_Recieve.current_position_f << std::endl;
            }

            std::cout << " 发送次数:               " << tx_count << std::endl
                      << " CAN转USB设备0 接收次数: " << can_dev0_rx_count << std::endl
                      << " TIME:                  " << time_now << "s" << std::endl
                      << " ***************************************************************** " << std::endl;
        }
    }

    //程序终止时的提示信息
    std::cout << "CAN_TX_test_thread  Exit~~" << std::endl;
    std::cout << std::endl
              << "----------------请输入任意数字，按回车，以结束键盘进程  ----------------------" << std::endl
              << std::endl;
}

// 键盘输入线程
void Zbot_RL::keyborad_input()
{
    while (running_)
    {
        std::cin >> command_input; // 键盘输入电机速度限制 // 此处不加线程锁，没有必要
    }
    std::cout << "keyborad_input_thread  Exit~~" << std::endl;
}

// 策略线程：以50Hz运行，读取 command_input 与 zbot 状态，计算 position_output
void Zbot_RL::Strategy_thread()
{
    const std::chrono::milliseconds period(20); // 50Hz
    // const std::chrono::milliseconds period(100); // 10Hz
    LogFrame log_strategy_data;

    while (running_)
    {
        auto t0 = std::chrono::steady_clock::now();

        // 读取必要数据并计算输出
        float input_copy;
        std::vector<float> cur_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        std::vector<float> cur_vel = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        float cur_quat_w = Q_desired.w();
        float cur_quat_x = Q_desired.x();
        float cur_quat_y = Q_desired.y();
        float cur_quat_z = Q_desired.z();

        {
            {
                std::lock_guard<std::mutex> lock(mutex_command_input); 
                input_copy = command_input;
            }
            {
                std::lock_guard<std::mutex> lock(mutex_DEV0_RX);
                IMU.IMU_quat_correct(DEV0_RX.Module_CAN_Recieve[0].IMU_Recieve); // 四元数校正

                cur_quat_w = DEV0_RX.Module_CAN_Recieve[0].IMU_Recieve.quat[0];
                cur_quat_x = DEV0_RX.Module_CAN_Recieve[0].IMU_Recieve.quat[1];
                cur_quat_y = DEV0_RX.Module_CAN_Recieve[0].IMU_Recieve.quat[2];
                cur_quat_z = DEV0_RX.Module_CAN_Recieve[0].IMU_Recieve.quat[3];

                std::cout << " IMU四元数: [" << DEV0_RX.Module_CAN_Recieve[0].IMU_Recieve.quat[0] << "," << DEV0_RX.Module_CAN_Recieve[0].IMU_Recieve.quat[1] << "," << 
                                               DEV0_RX.Module_CAN_Recieve[0].IMU_Recieve.quat[2] << "," << DEV0_RX.Module_CAN_Recieve[0].IMU_Recieve.quat[3] << "]" << std::endl;

                cur_pos[0] = DEV0_RX.Module_CAN_Recieve[0].Motor_Recieve.current_position_f;
                cur_pos[1] = DEV0_RX.Module_CAN_Recieve[1].Motor_Recieve.current_position_f;
                cur_pos[2] = DEV0_RX.Module_CAN_Recieve[2].Motor_Recieve.current_position_f;
                cur_pos[3] = DEV0_RX.Module_CAN_Recieve[3].Motor_Recieve.current_position_f;
                cur_pos[4] = DEV0_RX.Module_CAN_Recieve[4].Motor_Recieve.current_position_f;
                cur_pos[5] = DEV0_RX.Module_CAN_Recieve[5].Motor_Recieve.current_position_f;

                cur_vel[0] = DEV0_RX.Module_CAN_Recieve[0].Motor_Recieve.current_speed_f;
                cur_vel[1] = DEV0_RX.Module_CAN_Recieve[1].Motor_Recieve.current_speed_f;
                cur_vel[2] = DEV0_RX.Module_CAN_Recieve[2].Motor_Recieve.current_speed_f;
                cur_vel[3] = DEV0_RX.Module_CAN_Recieve[3].Motor_Recieve.current_speed_f;
                cur_vel[4] = DEV0_RX.Module_CAN_Recieve[4].Motor_Recieve.current_speed_f;
                cur_vel[5] = DEV0_RX.Module_CAN_Recieve[5].Motor_Recieve.current_speed_f;
            }
        }

#ifdef USE_LIBTORCH
        if (model_loaded && policy_model)
        {
            try {
                // 构造输入tensor：模型接受输入
                // std::vector<float> invec = {cur_quat_w, cur_quat_x, cur_quat_y, cur_quat_z, 
                //                             cur_pos[0] - init_angles[0], cur_pos[1] - init_angles[1], cur_pos[2] - init_angles[2], cur_pos[3] - init_angles[3], cur_pos[4] - init_angles[4], cur_pos[5] - init_angles[5],
                //                             cur_vel[0], cur_vel[1], cur_vel[2], cur_vel[3], cur_vel[4], cur_vel[5],
                //                             out_last[0], out_last[1], out_last[2], out_last[3], out_last[4], out_last[5],
                //                             input_copy
                //                            };
                std::vector<float> invec = {cur_quat_w, cur_quat_x, cur_quat_y, cur_quat_z, 
                                            cur_pos[0] - init_angles[0], cur_pos[1] - init_angles[1], cur_pos[2] - init_angles[2], cur_pos[3] - init_angles[3], cur_pos[4] - init_angles[4], cur_pos[5] - init_angles[5],
                                            cur_vel[0], cur_vel[1], cur_vel[2], cur_vel[3], cur_vel[4], cur_vel[5],
                                            out_last[0], out_last[1], out_last[2], out_last[3], out_last[4], out_last[5]
                                           };

                // ---- 读取CSV作为策略输入 ----
                // std::vector<float> invec;
                // if (csv_index < csv_data.size()) {
                //     invec = csv_data[csv_index++];
                // } else {
                //     // 播放完就停在最后一行 // 停不住：虽然out不变,relative_tensor一直增加
                //     // invec = csv_data.back();
                //     return;  // 直接退出当前策略线程函数
                // }

                at::Tensor input_tensor = torch::from_blob(invec.data(), {1, (long)invec.size()}).clone();
                // 前向推理
                std::vector<torch::jit::IValue> inputs;
                inputs.push_back(input_tensor);
                at::Tensor out = policy_model->forward(inputs).toTensor().squeeze().tanh();

                out_last.assign(out.data_ptr<float>(), 
                                out.data_ptr<float>() + out.numel());
                relative_tensor += out * input_copy * 0.02f * PI;
                relative_tensor = relative_tensor.clip(-1.0f * PI, 1.0f * PI);
                // at::Tensor类型没有toVector
                std::vector<float> relative_vec(relative_tensor.data_ptr<float>(), 
                                              relative_tensor.data_ptr<float>() + relative_tensor.numel());
                {
                    std::lock_guard<std::mutex> lock(mutex_position_output);
                    for (size_t i = 0; i < relative_vec.size(); ++i) {
                        position_output[i] = relative_vec[i] + init_angles[i];
                    }

                    float time_now = getTimestamp();
                    log_strategy_data.timestamp = time_now;
                    for (size_t i = 0; i < 6; ++i) {
                        // log_strategy_data.motor_pos[i] = cur_pos[i] - init_angles[i];
                        log_strategy_data.motor_pos[i] = position_output[i] - init_angles[i];
                        log_strategy_data.motor_vel[i] = cur_vel[i];
                    }
                    log_strategy_data.imu_quat[0] = cur_quat_w;
                    log_strategy_data.imu_quat[1] = cur_quat_x;
                    log_strategy_data.imu_quat[2] = cur_quat_y;
                    log_strategy_data.imu_quat[3] = cur_quat_z;
                    {
                        std::lock_guard<std::mutex> lock(mutex_log_strategy_queue);
                        log_strategy_queue.push(log_strategy_data);
                    }
                }
                
            } catch (const std::exception &e) {
                // 推理失败则回退到简单策略
                std::lock_guard<std::mutex> lock(mutex_position_output);
                position_output = init_angles;
                std::cout << "Policy inference failed: " << e.what() << std::endl;
            }
        }
        else
#endif
        {
            // 简单回退策略
            std::lock_guard<std::mutex> lock(mutex_position_output);
            position_output = init_angles;
        }

        // 固定周期等待
        auto elapsed = std::chrono::steady_clock::now() - t0;
        if (elapsed < period)
            std::this_thread::sleep_for(period - elapsed);
    }
    std::cout << "Strategy_thread Exit~~" << std::endl;
}

// 日志线程
void Zbot_RL::Log_thread() {
    // ================== 在这里修改存储路径 ==================
    std::string log_path = "../figures_data/data/";
    
    // 确保路径以斜杠结尾
    if (!log_path.empty() && log_path.back() != '/') {
        log_path += '/';
    }
    
    using namespace std::chrono;
    auto start_time = steady_clock::now();
    int file_index = 0;
    std::ofstream strategy_file;

    auto openNewFile = [&](int index) {
        if (strategy_file.is_open()) strategy_file.close();

        std::string strategy_filename = log_path + "log_strategy_" + std::to_string(index) + ".csv";
        strategy_file.open(strategy_filename, std::ios::out);
        if (!strategy_file.is_open()) {
            std::cerr << "无法打开文件: " << strategy_filename << std::endl;
            return;
        }
        // 表头：四元数在前
        strategy_file << "timestamp,imu_w,imu_x,imu_y,imu_z,pos1,pos2,pos3,pos4,pos5,pos6,vel1,vel2,vel3,vel4,vel5,vel6\n";
        
        std::cout << "创建日志文件: " << strategy_filename << std::endl;
    };

    openNewFile(file_index);

    while (running_) {
        auto now = steady_clock::now();
        if (duration_cast<seconds>(now - start_time).count() >= 100) {
            file_index++;
            openNewFile(file_index);
            start_time = now;
        }

        // 批量收集策略队列数据
        std::vector<LogFrame> strategy_batch;
        {
            std::lock_guard<std::mutex> lock(mutex_log_strategy_queue);
            while (!log_strategy_queue.empty()) {
                strategy_batch.push_back(log_strategy_queue.front());
                log_strategy_queue.pop();
            }
        }

        // 批量写入策略日志
        if (!strategy_batch.empty() && strategy_file.is_open()) {
            for (auto &entry : strategy_batch) {
                strategy_file << std::fixed << std::setprecision(6) << entry.timestamp << ",";
                
                // 先写入 IMU 四元数 (w,x,y,z)
                for (int i = 0; i < 4; ++i) {
                    strategy_file << entry.imu_quat[i] << ",";
                }
                
                // 再写入电机位置
                for (int i = 0; i < 6; ++i) {
                    strategy_file << entry.motor_pos[i] << ",";
                }
                
                // 最后写入电机速度（最后一个不加逗号）
                for (int i = 0; i < 6; ++i) {
                    strategy_file << entry.motor_vel[i];
                    if (i < 5) {
                        strategy_file << ",";
                    } else {
                        strategy_file << "\n";
                    }
                }
            }
            strategy_file.flush();  // 确保数据写入磁盘
        }

        // 队列为空则短暂休眠
        if (strategy_batch.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    if (strategy_file.is_open()) strategy_file.close();
    
    std::cout << "日志线程结束，文件保存路径: " << log_path << std::endl;
}

void Zbot_RL::ALL_Motor_ENABLE(int delay_us)
{
    Motor.Motor_Enable(USB2CAN0_, 2, 0x01);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Enable(USB2CAN0_, 1, 0x04);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Enable(USB2CAN0_, 2, 0x02);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Enable(USB2CAN0_, 1, 0x05);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Enable(USB2CAN0_, 2, 0x03);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Enable(USB2CAN0_, 1, 0x06);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
}

void Zbot_RL::ALL_Motor_DISABLE(int delay_us)
{   
    Motor.Motor_Disable(USB2CAN0_, 2, 0x01);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Disable(USB2CAN0_, 1, 0x04);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Disable(USB2CAN0_, 2, 0x02);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Disable(USB2CAN0_, 1, 0x05);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Disable(USB2CAN0_, 2, 0x03);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Disable(USB2CAN0_, 1, 0x06);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
}

void Zbot_RL::ALL_Motor_PP_Mode_Set(int delay_us)
{
    Motor.PP_Mode_Set(USB2CAN0_, 2, 0x01, 20.0f, 30.0f, delay_us);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    Motor.PP_Mode_Set(USB2CAN0_, 1, 0x04, 20.0f, 30.0f, delay_us);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    Motor.PP_Mode_Set(USB2CAN0_, 2, 0x02, 20.0f, 30.0f, delay_us);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    Motor.PP_Mode_Set(USB2CAN0_, 1, 0x05, 20.0f, 30.0f, delay_us);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    Motor.PP_Mode_Set(USB2CAN0_, 2, 0x03, 20.0f, 30.0f, delay_us);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    Motor.PP_Mode_Set(USB2CAN0_, 1, 0x06, 20.0f, 30.0f, delay_us);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));
}

void Zbot_RL::ALL_Motor_PP_Angle_Set(int delay_us, std::vector<float> motor_angles)
{
    Motor.PP_Angle_Set(USB2CAN0_, 2, 0x01, -motor_angles[0]); // 电机顺时针为角度增加，所以加负号
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    Motor.PP_Angle_Set(USB2CAN0_, 1, 0x04, -motor_angles[3]); // 电机顺时针为角度增加，所以加负号
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    Motor.PP_Angle_Set(USB2CAN0_, 2, 0x02, -motor_angles[1]); // 电机顺时针为角度增加，所以加负号
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    Motor.PP_Angle_Set(USB2CAN0_, 1, 0x05, -motor_angles[4]); // 电机顺时针为角度增加，所以加负号
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    Motor.PP_Angle_Set(USB2CAN0_, 2, 0x03, -motor_angles[2]); // 电机顺时针为角度增加，所以加负号
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    Motor.PP_Angle_Set(USB2CAN0_, 1, 0x06, -motor_angles[5]); // 电机顺时针为角度增加，所以加负号
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));
}

void Zbot_RL::ALL_Motor_PP_Init(std::vector<float> motor_angles)
{
    // 所有电机设置为PP模式
    ALL_Motor_PP_Mode_Set(Delay_10000us);
    sleep(0.1);

    // 以PP模式运动到初始位置
    ALL_Motor_PP_Angle_Set(Delay_10000us, motor_angles);
    sleep(3);
}

void Zbot_RL::ALL_Motor_Zero_Set(int delay_us)
{
    Motor.Motor_Zero_Set(USB2CAN0_, 2, 0x01);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Zero_Set(USB2CAN0_, 1, 0x04);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Zero_Set(USB2CAN0_, 2, 0x02);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Zero_Set(USB2CAN0_, 1, 0x05);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Zero_Set(USB2CAN0_, 2, 0x03);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Zero_Set(USB2CAN0_, 1, 0x06);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
}

void Zbot_RL::ALL_Motor_PD_Init(std::vector<float> motor_angles)
{
    ALL_Motor_ENABLE(Delay_1000us); // 使能电机
    sleep(1);
    ALL_Motor_PD_Control(Delay_1000us, motor_angles); // PD模式运动到指定角度
    sleep(3);
}

void Zbot_RL::ALL_Motor_PD_Control(int delay_us, std::vector<float> motor_angles)
{
    auto t = std::chrono::high_resolution_clock::now();//这一句耗时50us

    Motor.Motor_PD_Control(USB2CAN0_, 2, 0x01, &Zbot1234_RL_PD, -motor_angles[0]); // 电机顺时针为角度增加，所以加负号
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);

    Motor.Motor_PD_Control(USB2CAN0_, 1, 0x04, &Zbot1234_RL_PD, -motor_angles[3]); // 电机顺时针为角度增加，所以加负号
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);

    Motor.Motor_PD_Control(USB2CAN0_, 2, 0x02, &Zbot1234_RL_PD, -motor_angles[1]); // 电机顺时针为角度增加，所以加负号
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);

    Motor.Motor_PD_Control(USB2CAN0_, 1, 0x05, &Zbot5_RL_PD, -motor_angles[4]); // 电机顺时针为角度增加，所以加负号
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);

    Motor.Motor_PD_Control(USB2CAN0_, 2, 0x03, &Zbot1234_RL_PD, -motor_angles[2]); // 电机顺时针为角度增加，所以加负号
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);

    Motor.Motor_PD_Control(USB2CAN0_, 1, 0x06, &Zbot6_RL_PD, -motor_angles[5]); // 电机顺时针为角度增加，所以加负号
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
}

void Zbot_RL::Read_Clear(int dev, int num)
{
    uint8_t channel;
    FrameInfo info_rx;
    uint8_t data_rx[8] = {0};

    for (int i = 0; i < num; i++)
    {
        // 有数据不会阻塞，若无数据则等待0.1s
        readUSBCAN(dev, &channel, &info_rx, data_rx, 1e5);
    }
}

float Zbot_RL::getTimestamp() {
    // 获取当前系统时间戳
    std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> tpMill =
        std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
    time_t now_tp = tpMill.time_since_epoch().count(); 
    
    time_t relavitive_tp = now_tp - start_tp;
    float timestamp = relavitive_tp / 1000.0f;

    return timestamp;
}

std::vector<std::vector<float>> loadCSV_invec(const std::string& path) {
    std::vector<std::vector<float>> data;
    std::ifstream file(path);
    std::string line;

    bool is_first_line = true;
    int line_num = 0;

    while (std::getline(file, line)) {
        line_num++;

        // 跳过表头
        if (is_first_line) {
            is_first_line = false;
            continue;
        }

        std::stringstream ss(line);
        std::string cell;

        std::vector<float> row_23;
        int col_index = 0;

        while (std::getline(ss, cell, ',')) {

            // 去掉前后空白和 \r
            cell.erase(0, cell.find_first_not_of(" \t\r"));
            cell.erase(cell.find_last_not_of(" \t\r") + 1);

            // 只处理 2~24 列，即 col_index 1~23
            if (col_index >= 1 && col_index <= 23) {
                if (!cell.empty()) {
                    try {
                        row_23.push_back(std::stof(cell));
                    } catch (...) {
                        std::cout << "Warning: CSV 第 " << line_num
                                  << " 行第 " << (col_index + 1)
                                  << " 列无法转换为数字，值='" << cell
                                  << "'，该行已跳过。\n";
                        row_23.clear();
                        break;
                    }
                } else {
                    row_23.push_back(0.0f);  // 空值给默认0，你可以换成需要的默认值
                }
            }

            col_index++;
        }

        // 必须是 23 列
        if (row_23.size() != 23) {
            std::cout << "Warning: 第 " << line_num
                      << " 行有效列数不是23，已跳过。\n";
            continue;
        }

        data.push_back(row_23);
    }

    return data;
}


