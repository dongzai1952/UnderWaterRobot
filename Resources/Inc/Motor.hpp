#ifndef __MOTOR_HPP__
#define __MOTOR_HPP__

#include "main.h"

class Motor
{
private:
    TIM_HandleTypeDef *htim_ptr_ = nullptr;
    int channel_ = 0;
    int speed_ = 0;  //传入速度
    int speed_set_ = 0;  //设置的速度
    int max_speed_ = 0;
    int dead_zone_f_ = 0;  //前进死区
    int dead_zone_b_ = 0;  //后退死区
public:
    Motor() {};
    ~Motor() {};
    void Init(TIM_HandleTypeDef *htim_ptr, int channel, int dead_zone_f, int dead_zone_b, int max_speed);
    void SetInput(int speed);
    void Stop();
};
#endif