#ifndef __NEW_IMU_HPP__
#define __NEW_IMU_HPP__

#include "main.h"
#include <stdint.h>

#define IMU_FRAME_SIZE 30  // 30字节数据帧

//刘勋给的高精度Imu
class NewImu {
private:
    // 传感器数据
    float gyro_x_ = 0;    // 角速度X (度/秒)
    float gyro_y_ = 0;    // 角速度Y
    float gyro_z_ = 0;    // 角速度Z
    float acc_x_ = 0;     // 加速度X (mg)
    float acc_y_ = 0;     // 加速度Y
    float acc_z_ = 0;     // 加速度Z
    float pitch_ = 0;     // 俯仰角 (度)
    float roll_ = 0;      // 横滚角
    float yaw_ = 0;       // 偏航角
    float yaw_zero_ = 0;  // 偏航角零点偏移
    float temp_ = 0;      // 温度 (摄氏度)
    
    // 接收缓冲区
    // uint8_t rx_data_;
    // uint8_t rx_buf_[30];  // 完整数据帧30字节
    // uint8_t rx_index_ = 0;
    // bool data_ready_ = false;
    
    UART_HandleTypeDef *huart_ = nullptr;
    
    // CRC校验
    uint16_t CalculateCRC16(const uint8_t *data, uint16_t length);
    
public:
    uint8_t *imu_rx_buf_;  // DMA接收缓冲区
    NewImu() = default;
    ~NewImu() = default;
    
    void Init(UART_HandleTypeDef *huart, uint8_t *imu_rx_buf);
    void Decode();
    void ResetYaw();
    void SetYaw(float target_yaw);
    void StartAutoOutput();
    void StopAutoOutput();
    void GyroCalibration();
    void SaveToFlash();
    
    // 数据获取接口
    float GetGyroX() const { return gyro_x_; }
    float GetGyroY() const { return gyro_y_; }
    float GetGyroZ() const { return gyro_z_; }
    float GetAccX() const { return acc_x_ / 1000.0f; }  // 转换为g
    float GetAccY() const { return acc_y_ / 1000.0f; }
    float GetAccZ() const { return acc_z_ / 1000.0f; }
    float GetPitch() const { return pitch_; }
    float GetRoll() const { return roll_; }
    float GetYaw() const { 
        float yaw = yaw_ - yaw_zero_;
        if(yaw > 180) yaw -= 360;
        else if(yaw < -180) yaw += 360;
        return yaw;
    }
    float GetTemp() const { return temp_; }
};

#endif