#include "Motor_test.h"

static class Hipnuc_IMU IMU;
static class RS_Motor Motor;

static time_t start_tp = 0;
int motor_ID = 0x05; // 电机ID
float init_angle = 0.0f;

// 主函数循环
void Motor_test::Spin()
{
    while (all_thread_done_ != true)
    {
        sleep(1); // 延时1s
    }
    printf("~ ALL Exit ~\n");
}

/// @brief 构造函数，初始化
/// @return
Motor_test::Motor_test() 
{
    running_ = true;
    all_thread_done_ = false;

    std::cout << std::endl
            << "RUN Motor_test.cpp" << std::endl
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
    
    // Motor.Motor_Enable(USB2CAN0_, 1, motor_ID); // 使能电机1
    // Read_Clear(USB2CAN0_, 5); // 清理5条接收缓存
    // sleep(1);
    // init_angle = - Motor.Angle_Read(USB2CAN0_, 1, motor_ID); // 读取当前电机角度，作为初始角度，电机顺时针为角度增加，所以加负号
    // std::cout << std::endl
    //         << "InitAngle:" << init_angle << std::endl;
    // Motor.Motor_PD_Control(USB2CAN0_, 1, motor_ID, &Motor_test_PD, -init_angle); // 注意：电机顺时针为角度增加，所以加负号
    // sleep(3);
    Motor.Set_MotorID(USB2CAN0_, 1, 0x06, 0x05);
    std::this_thread::sleep_for(std::chrono::microseconds(10000)); // 单位us

    for (int i = 0; i < 256; i++)
    {
        Motor.Get_MotorID(USB2CAN0_, 1, i); // 获取电机ID
        std::this_thread::sleep_for(std::chrono::microseconds(500)); // 单位us
    }
    std::cout << "OVER" << std::endl;

    // ********************************************************************** 创 立 线 程 ********************************************************************** //

    // // 获取当前系统时间戳
    // std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> tpMill =
    //     std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
    // start_tp = tpMill.time_since_epoch().count();

    // // 创建CAN接收线程，设备1
    // _Test_RX_thread = std::thread(&Motor_test::Test_RX_thread, this);

    // // CAN发送线程
    // _Test_TX_thread = std::thread(&Motor_test::Test_TX_thread, this);

    // // 记录线程（50Hz）
    // _Test_Log_thread = std::thread(&Motor_test::Test_Log_thread, this);
}

/// @brief 析构函数
Motor_test::~Motor_test()
{

    running_ = false;

    /*注销线程*/

    // can接收设备0
    _Test_RX_thread.join();

    //can发送测试线程
    _Test_TX_thread.join();

    _Test_Log_thread.join();
    // 失能电机
    Motor.Motor_Disable(USB2CAN0_, 1, motor_ID);

    // 关闭设备
    closeUSBCAN(USB2CAN0_);

    all_thread_done_ = true;
}

/// @brief can设备0，接收线程函数
void Motor_test::Test_RX_thread()
{
    can_dev0_rx_count = 0;
    can_dev0_rx_count_thread=0;
    
    while (running_)
    {
        uint8_t channel;
        FrameInfo info_rx;
        uint8_t data_rx[8] = {0};

        can_dev0_rx_count_thread++;

        // 有数据不会阻塞，若无数据则等待1s
        int recieve_re = readUSBCAN(USB2CAN0_, &channel, &info_rx, data_rx, 1e6);

        // 接收到数据
        if (recieve_re != -1)
        {
            can_dev0_rx_count++;
            
            TEMP_ID = (info_rx.canID >> 8) & 0xff;
            uint8_t index = TEMP_ID - 0x01;  // 将ID映射到0-2的索引,DEV0_RX包含ID01-ID03的电机数据

            {
                // 解码
                Motor_test_receive.master_id = (info_rx.canID) & 0xff;
                Motor_test_receive.motor_id = (info_rx.canID >> 8) & 0xff;
                Motor_test_receive.fault_message = (info_rx.canID >> 16) & 0x3f;
                Motor_test_receive.motor_state = (info_rx.canID >> 22) & 0x03;
                Motor_test_receive.mode = (info_rx.canID >> 24) & 0x1f;

                if (Motor_test_receive.mode == 0x02)
                {
                    // 收到的数据高字节在前
                    Motor_test_receive.current_position = (data_rx[0] << 8) | (data_rx[1]);
                    Motor_test_receive.current_speed = (data_rx[2] << 8) | (data_rx[3]);
                    Motor_test_receive.current_torque = (data_rx[4] << 8) | (data_rx[5]);
                    Motor_test_receive.current_temp = (data_rx[6] << 8) | (data_rx[7]);

                    // 转换
                    Motor_test_receive.current_position_f = -uint_to_float(Motor_test_receive.current_position, (P_MIN), (P_MAX), 16); // 电机顺时针为角度增加，所以加负号
                    Motor_test_receive.current_speed_f = -uint_to_float(Motor_test_receive.current_speed, (V_MIN), (V_MAX), 16); // 电机顺时针为角度增加，所以加负号
                    Motor_test_receive.current_torque_f = -uint_to_float(Motor_test_receive.current_torque, (T_MIN), (T_MAX), 16); // 电机顺时针为角度增加，所以加负号
                    Motor_test_receive.current_temp_f = (float)Motor_test_receive.current_temp / 10;
                }
            }
            
            float logtime_now = getTimestamp();
            {
                std::lock_guard<std::mutex> lock(rx_log_mutex);
                Log_RX.timestamp = logtime_now;
                Log_RX.Log_angle = Motor_test_receive.current_position_f;  // 记录接收的角度
                
                rx_log_queue.push(Log_RX);
            }

            std::cout << "logtime_now:" << Log_RX.timestamp << std::endl;
            std::cout << "RX_angle:" << Log_RX.Log_angle << std::endl;
        }
    }
    std::cout << "CAN_RX_device_0_thread  Exit~~" << std::endl;
}

// can发送线程函数
void Motor_test::Test_TX_thread()
{
    // 发送计数
    uint32_t tx_count = 0;

    // 正弦波参数
    double amplitude = 0.25 * PI;  // 幅值（弧度）
    double frequency = 4;  // 频率（Hz）

    std::chrono::steady_clock::time_point next_cycle = std::chrono::steady_clock::now();

    while (running_)
    {
        // 设置下一个循环开始时间
        next_cycle += std::chrono::milliseconds(20);

        // CAN发送计数
        tx_count++;
        
        // 计算当前时间（秒）：tx_count * 0.02
        double time_in_seconds = tx_count * 0.02;
        
        // 生成正弦波角度：sin(2π * 频率 * 时间)
        double sine_value = amplitude * std::sin(frequency * time_in_seconds);
        TX_angle = init_angle + sine_value;
        
        // 发送控制命令
        Motor.Motor_PD_Control(USB2CAN0_, 1, motor_ID, &Motor_test_PD, -TX_angle); // 注意：电机顺时针为角度增加，所以加负号

        // 记录TX数据
        float logtime_now = getTimestamp();
        {
            std::lock_guard<std::mutex> lock(tx_log_mutex);
            Log_TX.timestamp = logtime_now;
            Log_TX.Log_angle = TX_angle;  // 记录发送的角度

            tx_log_queue.push(Log_TX);
        }

        std::cout << "logtime_now:" << Log_TX.timestamp << std::endl;
        std::cout << "TX_angle:" << Log_TX.Log_angle << std::endl;
        
        // 等待到下一个周期
        std::this_thread::sleep_until(next_cycle);
    }

    //程序终止时的提示信息
    std::cout << "CAN_TX_test_thread  Exit~~" << std::endl;
    std::cout << std::endl
              << "----------------请输入任意数字，按回车，以结束键盘进程  ----------------------" << std::endl
              << std::endl;
}

// 日志线程
void Motor_test::Test_Log_thread()
{
    // 指定CSV存储路径
    std::string csv_path = "../figures_data/data";
    
    // 获取当前时间作为日志文件前缀
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm local_tm = *std::localtime(&now_time);
    
    // 创建时间戳字符串
    std::ostringstream time_ss;
    time_ss << std::put_time(&local_tm, "%Y%m%d_%H%M%S");
    std::string timestamp_str = time_ss.str();
    
    // 文件计数器
    int file_counter_tx = 0;
    int file_counter_rx = 0;
    
    // 文件记录起始时间
    auto file_start_time_tx = std::chrono::steady_clock::now();
    auto file_start_time_rx = std::chrono::steady_clock::now();
    
    // 两个日志文件流
    std::ofstream log_file_tx;
    std::ofstream log_file_rx;
    std::string current_filename_tx;
    std::string current_filename_rx;
    
    // CSV文件列标题
    const std::string header_tx = "Timestamp,TX_Angle\n";
    const std::string header_rx = "Timestamp,RX_Angle\n";
    
    // 数据记录间隔（秒）
    const int LOG_INTERVAL_SECONDS = 100;
    
    // 创建目录（如果不存在）
    std::filesystem::path dir_path(csv_path);
    if (!std::filesystem::exists(dir_path))
    {
        std::filesystem::create_directories(dir_path);
        std::cout << "[Log Thread] Created directory: " << csv_path << std::endl;
    }
    else
    {
        std::cout << "[Log Thread] Using directory: " << csv_path << std::endl;
    }
    
    // 主循环
    while (running_)
    {
        // 处理TX日志数据
        bool has_tx_data = false;
        LogFrame tx_log;
        {
            std::lock_guard<std::mutex> lock(tx_log_mutex);
            if (!tx_log_queue.empty())
            {
                tx_log = tx_log_queue.front();
                tx_log_queue.pop();
                has_tx_data = true;
            }
        }
        
        // 处理RX日志数据
        bool has_rx_data = false;
        LogFrame rx_log;
        {
            std::lock_guard<std::mutex> lock(rx_log_mutex);
            if (!rx_log_queue.empty())
            {
                rx_log = rx_log_queue.front();
                rx_log_queue.pop();
                has_rx_data = true;
            }
        }
        
        // ========== TX文件处理 ==========
        if (has_tx_data)
        {
            // 检查是否需要创建新的TX文件
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed_seconds_tx = std::chrono::duration_cast<std::chrono::seconds>(
                current_time - file_start_time_tx).count();
            
            if (!log_file_tx.is_open() || elapsed_seconds_tx >= LOG_INTERVAL_SECONDS)
            {
                // 关闭旧文件（如果存在）
                if (log_file_tx.is_open())
                {
                    log_file_tx.close();
                    std::cout << "[Log Thread] Closed TX log file: " << current_filename_tx << std::endl;
                }
                
                // 生成新文件名
                std::ostringstream filename_ss;
                filename_ss << csv_path << "/motor_test_TX_" << timestamp_str 
                           << "_" << std::setw(3) << std::setfill('0') << file_counter_tx 
                           << ".csv";
                
                current_filename_tx = filename_ss.str();
                
                // 打开新文件
                log_file_tx.open(current_filename_tx);
                if (!log_file_tx.is_open())
                {
                    std::cerr << "[Log Thread] Failed to open TX log file: " << current_filename_tx << std::endl;
                }
                else
                {
                    // 写入CSV头部
                    log_file_tx << header_tx;
                    log_file_tx.flush();
                    
                    std::cout << "[Log Thread] Created new TX log file: " << current_filename_tx << std::endl;
                    
                    // 重置计时器和计数器
                    file_start_time_tx = current_time;
                    file_counter_tx++;
                }
            }
            
            // 写入TX数据到CSV文件
            if (log_file_tx.is_open())
            {
                log_file_tx << tx_log.timestamp << "," 
                           << tx_log.Log_angle << "\n";
                log_file_tx.flush();
            }
        }
        
        // ========== RX文件处理 ==========
        if (has_rx_data)
        {
            // 检查是否需要创建新的RX文件
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed_seconds_rx = std::chrono::duration_cast<std::chrono::seconds>(
                current_time - file_start_time_rx).count();
            
            if (!log_file_rx.is_open() || elapsed_seconds_rx >= LOG_INTERVAL_SECONDS)
            {
                // 关闭旧文件（如果存在）
                if (log_file_rx.is_open())
                {
                    log_file_rx.close();
                    std::cout << "[Log Thread] Closed RX log file: " << current_filename_rx << std::endl;
                }
                
                // 生成新文件名
                std::ostringstream filename_ss;
                filename_ss << csv_path << "/motor_test_RX_" << timestamp_str 
                           << "_" << std::setw(3) << std::setfill('0') << file_counter_rx 
                           << ".csv";
                
                current_filename_rx = filename_ss.str();
                
                // 打开新文件
                log_file_rx.open(current_filename_rx);
                if (!log_file_rx.is_open())
                {
                    std::cerr << "[Log Thread] Failed to open RX log file: " << current_filename_rx << std::endl;
                }
                else
                {
                    // 写入CSV头部
                    log_file_rx << header_rx;
                    log_file_rx.flush();
                    
                    std::cout << "[Log Thread] Created new RX log file: " << current_filename_rx << std::endl;
                    
                    // 重置计时器和计数器
                    file_start_time_rx = current_time;
                    file_counter_rx++;
                }
            }
            
            // 写入RX数据到CSV文件
            if (log_file_rx.is_open())
            {
                log_file_rx << rx_log.timestamp << "," 
                           << rx_log.Log_angle << "\n";
                log_file_rx.flush();
            }
        }
        
        // 短暂休眠以避免过度占用CPU
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    // 关闭日志文件
    if (log_file_tx.is_open())
    {
        log_file_tx.close();
        std::cout << "[Log Thread] Closed TX log file on thread exit" << std::endl;
    }
    
    if (log_file_rx.is_open())
    {
        log_file_rx.close();
        std::cout << "[Log Thread] Closed RX log file on thread exit" << std::endl;
    }
    
    std::cout << "[Log Thread] Log thread exit" << std::endl;
}

void Motor_test::Read_Clear(int dev, int num)
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

float Motor_test::getTimestamp() {
    // 获取当前系统时间戳
    std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> tpMill =
        std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
    time_t now_tp = tpMill.time_since_epoch().count(); 
    
    time_t relavitive_tp = now_tp - start_tp;
    float timestamp = relavitive_tp / 1000.0f;

    return timestamp;
}

