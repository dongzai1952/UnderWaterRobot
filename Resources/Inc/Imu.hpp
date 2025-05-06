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
    float pich_ = 0;
    float yaw_ = 0;

    uint8_t rx_data_;
    uint8_t rx_buf_[11];
    uint8_t rx_index_ = 0;  //当前接收位置
    bool Acc_Update_FLAG_ = false;
    bool Palstance_Update_FLAG_ = false;
    bool Angle_Update_FLAG_ = false;

    UART_HandleTypeDef *huart_;
    
public:
    Imu() {};
    ~Imu() {};
    void Init(UART_HandleTypeDef *huart);
    void Decode();
};

#endif