#include "RS_motor.h"

/// @brief 获取电机ID
/// @param dev
/// @param channel
/// @param motor_id
void RS_Motor::Get_MotorID(int32_t dev, uint8_t channel, uint32_t motor_id)
{
    ID_CAN.id = motor_id;
    ID_CAN.exdata = 0xfd; // 主机ID
    ID_CAN.mode = 0x00;
    uint8_t read_channel;
    FrameInfo info_rx;
    uint8_t data_rx[8] = {0};
    uint8_t recieve_re = 0;
    uint16_t MotorID = 0;

    txMsg_CAN_Motor.canID = ((ID_CAN.id & 0xff) | ((ID_CAN.exdata & 0xffff) << 8) | ((ID_CAN.mode & 0x1f) << 24));

    for (int i = 0; i < 8; i++)
    {
        Data_CAN[i] = 0;
    }
    sendUSBCAN(dev, channel, &txMsg_CAN_Motor, Data_CAN);

    // 有数据不会阻塞，若无数据则等待0.01s
    recieve_re = readUSBCAN(dev, &read_channel, &info_rx, data_rx, 1e4);
    if (recieve_re != -1)
    {
        MotorID = static_cast<uint16_t>((info_rx.canID >> 8) & 0xFFFF);
        if (((info_rx.canID >> 24) & 0x1f) == 0x00) // 确认是ID响应帧
        {
            std::cout << "Motor ID is: " << MotorID << std::endl;
        }
    }
    
}

/// @brief 设置电机ID
/// @param dev 
/// @param channel 
/// @param old_motor_id 
/// @param new_motor_id 
void RS_Motor::Set_MotorID(int32_t dev, uint8_t channel, uint32_t old_motor_id, uint32_t new_motor_id)
{
    ID_CAN.id = old_motor_id;
    ID_CAN.exdata = 0xfd | ((new_motor_id & 0xff) << 8); // 低8位:主机ID=0xfd, 高8位:new_motor_id
    ID_CAN.mode = 0x07;

    txMsg_CAN_Motor.canID = ((ID_CAN.id & 0xff) | ((ID_CAN.exdata & 0xffff) << 8) | ((ID_CAN.mode & 0x1f) << 24));
    sendUSBCAN(dev, channel, &txMsg_CAN_Motor, Data_CAN);
}

/// @brief 电机使能
/// @param dev
/// @param channel
/// @param motor_id
void RS_Motor::Motor_Enable(int32_t dev, uint8_t channel, uint32_t motor_id)
{
    ID_CAN.id = motor_id;
    ID_CAN.exdata = 0xfd; // 主机ID
    ID_CAN.mode = 0x03;

    txMsg_CAN_Motor.canID = ((ID_CAN.id & 0xff) | ((ID_CAN.exdata & 0xffff) << 8) | ((ID_CAN.mode & 0x1f) << 24));

    for (int i = 0; i < 8; i++)
    {
        Data_CAN[i] = 0;
    }
    sendUSBCAN(dev, channel, &txMsg_CAN_Motor, Data_CAN);
}

/// @brief 电机失能
/// @param dev
/// @param channel
/// @param motor_id
void RS_Motor::Motor_Disable(int32_t dev, uint8_t channel, uint32_t motor_id)
{
    ID_CAN.id = motor_id;
    ID_CAN.exdata = 0xfd; // 主机ID
    ID_CAN.mode = 0x04;

    txMsg_CAN_Motor.canID = ((ID_CAN.id & 0xff) | ((ID_CAN.exdata & 0xffff) << 8) | ((ID_CAN.mode & 0x1f) << 24));
    for (int i = 0; i < 8; i++)
    {
        Data_CAN[i] = 0;
    }
    // Data_CAN[0] = 1; // 用于清除故障位
    sendUSBCAN(dev, channel, &txMsg_CAN_Motor, Data_CAN);
}

/// @brief 电机模式切换
/// @param dev
/// @param channel
/// @param motor_id
/// @param mode
void RS_Motor::Motor_Mode_Change(int32_t dev, uint8_t channel, uint32_t motor_id , uint8_t mode)
{
    ID_CAN.id = motor_id;
    ID_CAN.exdata = 0xfd; // 主机ID
    ID_CAN.mode = 0x12;

    txMsg_CAN_Motor.canID = ((ID_CAN.id & 0xff) | ((ID_CAN.exdata & 0xffff) << 8) | ((ID_CAN.mode & 0x1f) << 24));
    for (int i = 0; i < 8; i++)
    {
        Data_CAN[i] = 0;
    }

    Data_CAN[0] = 0x05;
    Data_CAN[1] = 0x70;
    Data_CAN[4] = mode;
    sendUSBCAN(dev, channel, &txMsg_CAN_Motor, Data_CAN);
}

/// @brief 电机PP模式最大速度设置
/// @param dev
/// @param channel
/// @param motor_id
/// @param velmax
void RS_Motor::PP_Vel_Max_Set(int32_t dev, uint8_t channel, uint32_t motor_id , float velmax)
{
    ID_CAN.id = motor_id;
    ID_CAN.exdata = 0xfd; // 主机ID
    ID_CAN.mode = 0x12;

    txMsg_CAN_Motor.canID = ((ID_CAN.id & 0xff) | ((ID_CAN.exdata & 0xffff) << 8) | ((ID_CAN.mode & 0x1f) << 24));
    for (int i = 0; i < 8; i++)
    {
        Data_CAN[i] = 0;
    }
    
    Data_CAN[0] = 0x24;
    Data_CAN[1] = 0x70;
    std::memcpy(&Data_CAN[4], &velmax, sizeof(velmax));
    sendUSBCAN(dev, channel, &txMsg_CAN_Motor, Data_CAN);
}

/// @brief 电机PP模式最大加速度设置
/// @param dev
/// @param channel
/// @param motor_id
/// @param acc
void RS_Motor::PP_Acc_Set(int32_t dev, uint8_t channel, uint32_t motor_id , float acc)
{
    ID_CAN.id = motor_id;
    ID_CAN.exdata = 0xfd; // 主机ID
    ID_CAN.mode = 0x12;

    txMsg_CAN_Motor.canID = ((ID_CAN.id & 0xff) | ((ID_CAN.exdata & 0xffff) << 8) | ((ID_CAN.mode & 0x1f) << 24));
    for (int i = 0; i < 8; i++)
    {
        Data_CAN[i] = 0;
    }
    
    Data_CAN[0] = 0x25;
    Data_CAN[1] = 0x70;
    std::memcpy(&Data_CAN[4], &acc, sizeof(acc));
    sendUSBCAN(dev, channel, &txMsg_CAN_Motor, Data_CAN);
}

/// @brief 电机PP模式初始化封装
/// @param dev
/// @param channel
/// @param motor_id
/// @param velmax
/// @param acc
/// @param delay_us
void RS_Motor::PP_Mode_Set(int32_t dev, uint8_t channel, uint32_t motor_id, float velmax, float acc, int delay_us)
{
    Motor_Mode_Change(dev, channel, motor_id, 1);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor_Enable(dev, channel, motor_id);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    PP_Vel_Max_Set(dev, channel, motor_id, velmax);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    PP_Acc_Set(dev, channel, motor_id, acc);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
}

/// @brief 电机PP模式角度设置（运动）
/// @param dev
/// @param channel
/// @param motor_id
/// @param angle
void RS_Motor::PP_Angle_Set(int32_t dev, uint8_t channel, uint32_t motor_id , float angle)
{
    ID_CAN.id = motor_id;
    ID_CAN.exdata = 0xfd; // 主机ID
    ID_CAN.mode = 0x12;

    txMsg_CAN_Motor.canID = ((ID_CAN.id & 0xff) | ((ID_CAN.exdata & 0xffff) << 8) | ((ID_CAN.mode & 0x1f) << 24));
    for (int i = 0; i < 8; i++)
    {
        Data_CAN[i] = 0;
    }
    Data_CAN[0] = 0x16;
    Data_CAN[1] = 0x70;
    std::memcpy(&Data_CAN[4], &angle, sizeof(angle));
    sendUSBCAN(dev, channel, &txMsg_CAN_Motor, Data_CAN);
}

//////////////////////////////////////////
/// @brief 电机速度模式最大电流设置
/// @param dev
/// @param channel
/// @param motor_id
/// @param cuurrentmax
void RS_Motor::VEL_Current_Max_Set(int32_t dev, uint8_t channel, uint32_t motor_id , float cuurrentmax)
{
    ID_CAN.id = motor_id;
    ID_CAN.exdata = 0xfd; // 主机ID
    ID_CAN.mode = 0x12;

    txMsg_CAN_Motor.canID = ((ID_CAN.id & 0xff) | ((ID_CAN.exdata & 0xffff) << 8) | ((ID_CAN.mode & 0x1f) << 24));
    for (int i = 0; i < 8; i++)
    {
        Data_CAN[i] = 0;
    }
    
    Data_CAN[0] = 0x18;
    Data_CAN[1] = 0x70;
    std::memcpy(&Data_CAN[4], &cuurrentmax, sizeof(cuurrentmax));
    sendUSBCAN(dev, channel, &txMsg_CAN_Motor, Data_CAN);
}

/// @brief 电机速度模式最大加速度设置
/// @param dev
/// @param channel
/// @param motor_id
/// @param accmax
void RS_Motor::VEL_Acc_Max_Set(int32_t dev, uint8_t channel, uint32_t motor_id , float accmax)
{
    ID_CAN.id = motor_id;
    ID_CAN.exdata = 0xfd; // 主机ID
    ID_CAN.mode = 0x12;

    txMsg_CAN_Motor.canID = ((ID_CAN.id & 0xff) | ((ID_CAN.exdata & 0xffff) << 8) | ((ID_CAN.mode & 0x1f) << 24));
    for (int i = 0; i < 8; i++)
    {
        Data_CAN[i] = 0;
    }
    
    Data_CAN[0] = 0x22;
    Data_CAN[1] = 0x70;
    std::memcpy(&Data_CAN[4], &accmax, sizeof(accmax));
    sendUSBCAN(dev, channel, &txMsg_CAN_Motor, Data_CAN);
}

/// @brief 电机速度模式初始化封装
/// @param dev
/// @param channel
/// @param motor_id
/// @param cuurrentmax
/// @param accmax
/// @param delay_us
void RS_Motor::VEL_Mode_Set(int32_t dev, uint8_t channel, uint32_t motor_id, float cuurrentmax, float accmax, int delay_us)
{
    Motor_Mode_Change(dev, channel, motor_id, 2);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor_Enable(dev, channel, motor_id);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    VEL_Current_Max_Set(dev, channel, motor_id, cuurrentmax);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    VEL_Acc_Max_Set(dev, channel, motor_id, accmax);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
}

/// @brief 电机速度模式速度设置（运动）
/// @param dev
/// @param channel
/// @param motor_id
/// @param vel
void RS_Motor::VEL_Vel_Set(int32_t dev, uint8_t channel, uint32_t motor_id , float vel)
{
    ID_CAN.id = motor_id;
    ID_CAN.exdata = 0xfd; // 主机ID
    ID_CAN.mode = 0x12;

    txMsg_CAN_Motor.canID = ((ID_CAN.id & 0xff) | ((ID_CAN.exdata & 0xffff) << 8) | ((ID_CAN.mode & 0x1f) << 24));
    for (int i = 0; i < 8; i++)
    {
        Data_CAN[i] = 0;
    }
    Data_CAN[0] = 0x0A;
    Data_CAN[1] = 0x70;
    std::memcpy(&Data_CAN[4], &vel, sizeof(vel));
    sendUSBCAN(dev, channel, &txMsg_CAN_Motor, Data_CAN);
}

/// @brief 电机零位设置
/// @param dev
/// @param channel
/// @param motor_id
void RS_Motor::Motor_Zero_Set(int32_t dev, uint8_t channel, uint32_t motor_id)
{
    ID_CAN.id = motor_id;
    ID_CAN.exdata = 0xfd; // 主机ID
    ID_CAN.mode = 0x06;

    txMsg_CAN_Motor.canID = ((ID_CAN.id & 0xff) | ((ID_CAN.exdata & 0xffff) << 8) | ((ID_CAN.mode & 0x1f) << 24));
    for (int i = 0; i < 8; i++)
    {
        Data_CAN[i] = 0;
    }
    
    Data_CAN[0] = 0x01;
    sendUSBCAN(dev, channel, &txMsg_CAN_Motor, Data_CAN);
}

/// @brief 电机（运控）PD模式控制
/// @param dev
/// @param channel
/// @param motor_id
/// @param Motor_PDControl
/// @param position
void RS_Motor::Motor_PD_Control(int32_t dev, uint8_t channel, uint32_t motor_id, Motor_PDControl_Struct *Motor_PDControl, float position)
{
    Motor_PDControl->Tar_Position = position;

    ID_CAN.id = motor_id;
    ID_CAN.exdata = float_to_uint(Motor_PDControl->Feedforward_Torque, T_MIN, T_MAX, 16);
    ID_CAN.mode = 0x1;

    txMsg_CAN_Motor.canID = ((ID_CAN.id & 0xff) | ((ID_CAN.exdata & 0xffff) << 8) | ((ID_CAN.mode & 0x1f) << 24));

    Data_CAN[0] = float_to_uint(Motor_PDControl->Tar_Position, P_MIN, P_MAX, 16) >> 8;
    Data_CAN[1] = float_to_uint(Motor_PDControl->Tar_Position, P_MIN, P_MAX, 16);
    Data_CAN[2] = float_to_uint(Motor_PDControl->Tar_Velocity, V_MIN, V_MAX, 16) >> 8;
    Data_CAN[3] = float_to_uint(Motor_PDControl->Tar_Velocity, V_MIN, V_MAX, 16);
    Data_CAN[4] = float_to_uint(Motor_PDControl->Kp, KP_MIN, KP_MAX, 16) >> 8;
    Data_CAN[5] = float_to_uint(Motor_PDControl->Kp, KP_MIN, KP_MAX, 16);
    Data_CAN[6] = float_to_uint(Motor_PDControl->Kd, KD_MIN, KD_MAX, 16) >> 8;
    Data_CAN[7] = float_to_uint(Motor_PDControl->Kd, KD_MIN, KD_MAX, 16);

    sendUSBCAN(dev, channel, &txMsg_CAN_Motor, Data_CAN);
}

float RS_Motor::Angle_Read(int32_t dev, uint8_t channel, uint32_t motor_id)
{
    ID_CAN.id = motor_id;
    ID_CAN.exdata = 0xfd; // 主机ID
    ID_CAN.mode = 0x11;

    uint8_t read_channel;
    FrameInfo info_rx;
    uint8_t data_rx[8] = {0};
    uint32_t uint_value;
    float float_value;

    txMsg_CAN_Motor.canID = ((ID_CAN.id & 0xff) | ((ID_CAN.exdata & 0xffff) << 8) | ((ID_CAN.mode & 0x1f) << 24));
    for (int i = 0; i < 8; i++)
    {
        Data_CAN[i] = 0;
    }
    Data_CAN[0] = 0x16;
    Data_CAN[1] = 0x70;
    
    sendUSBCAN(dev, channel, &txMsg_CAN_Motor, Data_CAN);

    // 有数据不会阻塞，若无数据则等待1s
    readUSBCAN(dev, &read_channel, &info_rx, data_rx, 1e6);

    // 解码
    uint_value = (data_rx[7] << 24) | (data_rx[6] << 16) | (data_rx[5] << 8) | (data_rx[4]);

    // 转换为float
    std::memcpy(&float_value, &uint_value, sizeof(float));

    return float_value;
}

/// @brief 辅助函数
int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max)
        x = x_max;
    else if (x < x_min)
        x = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}
