#ifndef __IMU_HPP__
#define __IMU_HPP__

#include "main.h"

class Imu
{
private:
    //速度
    float acc_x_ = 0;
    float acc_y_ = 0;
    float acc_z_ = 0;
    //角速度
    float w_x_ = 0;
    float w_y_ = 0;
    float w_z_ = 0;
    //角度
    float roll_ = 0;
    float pitch_ = 0;
    float yaw_ = 0;
    float yaw_zero_ = 0;

    uint8_t rx_data_;
    uint8_t rx_buf_[11];
    uint8_t rx_index_ = 0;  //当前接收位置
    bool Acc_Update_FLAG_ = false;
    bool Palstance_Update_FLAG_ = false;
    bool Angle_Update_FLAG_ = false;

    UART_HandleTypeDef *huart_ = nullptr;  //传感器串口
    
public:
    Imu() {};
    ~Imu() {};
    void Init(UART_HandleTypeDef *huart);
    void Decode();
    void ResetYaw();
    void SetYawZero();
    float GetPitch()
    {
        return pitch_;
    }
    float GetYaw()
    {
        float zeroed_yaw = yaw_ - yaw_zero_;
    
        // 处理角度跨越±180°的情况
        if (zeroed_yaw > 180.0) {
            zeroed_yaw -= 360.0;
        } else if (zeroed_yaw < -180.0) {
            zeroed_yaw += 360.0;
        }
        
        return zeroed_yaw;
    }
    float GetRoll()
    {
        return roll_;
    }
    float GetAccX()
    {
        return acc_x_;
    }
    float GetAccY()
    {
        return acc_y_;
    }
    float GetAccZ()
    {
        return acc_z_;
    }
};

#endif