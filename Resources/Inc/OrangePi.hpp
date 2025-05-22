#ifndef __ORANGE_PI_HPP__
#define __ORANGE_PI_HPP__

#include "main.h"

class OrangePi
{
struct Pos
{
    float x = 0;
    float y = 0;
};


private:
    uint8_t rx_data_;
    uint8_t rx_buf_[6];
    uint8_t rx_index_ = 0;  //当前接收位置
    UART_HandleTypeDef *huart_ = nullptr;  //传感器串口

public:
    Pos pos_;
    bool is_find_ = false;  //是否发现目标
    OrangePi() {};
    ~OrangePi() {};
    void Init(UART_HandleTypeDef *huart);
    void Decode();
};


#endif