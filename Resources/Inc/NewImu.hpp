#ifndef __NEW_IMU_HPP__
#define __NEW_IMU_HPP__

#include "main.h"
#include <stdint.h>

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
    // 上一次的加速度（世界坐标系）
    float last_acc_world_x = 0;
    float last_acc_world_y = 0;
    // 当前速度（世界坐标系）
    float velocity_x = 0;
    float velocity_y = 0;
    // 位移
    float displacement_x = 0;
    float displacement_y = 0;
    // 重力加速度
    const float g = 9.79319433994521f;
    float dt = 0;
    
    // 接收缓冲区
    uint8_t last_rx_data_;
    uint8_t rx_data_;
    uint8_t rx_buf_[38];  // 完整数据帧38字节
    uint8_t rx_index_ = 0;
    bool data_ready_ = false;
    
    UART_HandleTypeDef *huart_ = nullptr;
    
    // CRC校验
    uint16_t CalculateCRC16(const uint8_t *data, uint16_t length);
    
public:
    NewImu() = default;
    ~NewImu() = default;
    
    void Init(UART_HandleTypeDef *huart, float deta_time);
    void Decode();
    void INSUpdate();
    void INSReset();
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
    float GetSpeedX() {return -velocity_y;}
    float GetSpeedY() {return velocity_x;}
};

#endif