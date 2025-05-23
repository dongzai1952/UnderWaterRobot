#ifndef __DISTANCE_SENSOR_HPP__
#define __DISTANCE_SENSOR_HPP__

#include "main.h"

class DistanceSensor
{
private:
    uint16_t last_last_distance_ = 0;
    uint16_t last_distance_ = 0;
    uint16_t distance_ = 0;  //mm
    float distance_f_ = 0.0f;  //cm
    uint8_t tx_buff_[1] = {0xff};
    uint8_t rx_data_;
    uint8_t rx_buf_[4];
    uint8_t rx_index_ = 0;  //当前接收位置
    UART_HandleTypeDef *huart_ = nullptr;  //传感器串口
    uint16_t receive_fail_tick_ = 0;

public:
    bool last_receive_success_ = true;
    bool receive_success_ = true;
    DistanceSensor() {};
    ~DistanceSensor() {};
    void Init(UART_HandleTypeDef *huart);
    void UpdateData();
    void Decode();
    float GetDistance()
    {
        return distance_f_;
    }
};


#endif