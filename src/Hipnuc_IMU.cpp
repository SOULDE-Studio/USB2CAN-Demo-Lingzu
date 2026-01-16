#include "Hipnuc_IMU.h"

void Hipnuc_IMU::IMU_Set_SYNC_Mode(int32_t dev, uint8_t channel, uint8_t imu_id)
{
    txMsg_CAN_IMU.canID = 0x600 | (imu_id & 0xFF);
    Data_CAN[0] = 0x2F;
    Data_CAN[1] = 0x03;     // 设置TPDO4，对应四元数输出
    Data_CAN[2] = 0x18;
    Data_CAN[3] = 0x02;
    Data_CAN[4] = 0x01;     // 设置同步模式
    Data_CAN[5] = 0x00;
    Data_CAN[6] = 0x00;
    Data_CAN[7] = 0x00;
    sendUSBCAN(dev, channel, &txMsg_CAN_IMU, Data_CAN);
}

void Hipnuc_IMU::IMU_Send_SYNC(int32_t dev, uint8_t channel)
{
    txMsg_CAN_IMU.canID = 0x80;
    for (uint8_t i = 0; i < 8; i++)
    {
        Data_CAN[i] = 0x00;
    }

    sendUSBCAN(dev, channel, &txMsg_CAN_IMU, Data_CAN);
}

void Hipnuc_IMU::IMU_Get_init_quat(int32_t dev)
{
    uint8_t channel;
    FrameInfo info_rx;
    uint8_t data_rx[8] = {0};
    // 有数据不会阻塞，若无数据则等待1s
    int recieve_re = readUSBCAN(dev, &channel, &info_rx, data_rx, 1e6);

    // 接收到数据
    if (recieve_re != -1)
    {
        if (info_rx.frameType == STANDARD) // 读取IMU数据
        {
            // 收到的数据低字节在前
            Q_meas_init[0] = static_cast<float>(static_cast<int16_t>((data_rx[1] << 8) | data_rx[0])) / 10000.0f;
            Q_meas_init[1] = static_cast<float>(static_cast<int16_t>((data_rx[3] << 8) | data_rx[2])) / 10000.0f;
            Q_meas_init[2] = static_cast<float>(static_cast<int16_t>((data_rx[5] << 8) | data_rx[4])) / 10000.0f;
            Q_meas_init[3] = static_cast<float>(static_cast<int16_t>((data_rx[7] << 8) | data_rx[6])) / 10000.0f;
            std::cout << "Q_meas_init:[" << Q_meas_init[0] << "," << Q_meas_init[1] << "," << Q_meas_init[2] << "," << Q_meas_init[3] << "]" << std::endl;
        }
    }
}

void Hipnuc_IMU::IMU_Get_offset_quat(Eigen::Quaternionf q_desired)
{
    // 创建Eigen四元数 (注意顺序：w, x, y, z)
    Eigen::Quaternionf q_meas(
        Q_meas_init[0],  // w
        Q_meas_init[1],  // x  
        Q_meas_init[2],  // y
        Q_meas_init[3]   // z
    );
    
    // 归一化
    q_meas.normalize();
    q_desired.normalize();
    
    // 计算校正四元数: Q_offset = Q_desired * q_meas.conjugate()
    Q_offset = q_desired * q_meas.conjugate();
    Q_offset.normalize();
    
    std::cout << "Calibration Complete!" << std::endl;
    std::cout << "Q_offset:[" << Q_offset.w() << "," << Q_offset.x() << "," << Q_offset.y() << "," << Q_offset.z() << "]" << std::endl;
}

void Hipnuc_IMU::IMU_quat_correct(Hipnuc_IMU_Struct &IMU_data)
{
    Eigen::Quaternionf q_current(
        IMU_data.quat[0],  // w
        IMU_data.quat[1],  // x  
        IMU_data.quat[2],  // y
        IMU_data.quat[3]   // z
    );
    
    q_current.normalize();
    Eigen::Quaternionf q_corrected = Q_offset * q_current;
    q_corrected.normalize();
    
    // 存回数据
    IMU_data.quat[0] = q_corrected.w();
    IMU_data.quat[1] = q_corrected.x();
    IMU_data.quat[2] = q_corrected.y();
    IMU_data.quat[3] = q_corrected.z();
}
