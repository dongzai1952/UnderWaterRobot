#ifndef __MOTOR_HPP__
#define __MOTOR_HPP__

#include "main.h"

class Motor
{
private:
    TIM_HandleTypeDef *htim_ptr_ = nullptr;
    int channel_ = 0;
    float speed_ = 0.0f;  //传入速度
    int speed_set_ = 0;  //设置的速度
    float max_speed_ = 0.0f;  //最大速度
    int dead_zone_f_ = 0;  //前进死区
    int dead_zone_b_ = 0;  //后退死区
    bool dest_ = 1;  //电机方向：1正；0反
public:
    Motor() {};
    ~Motor() {};
    void Init(TIM_HandleTypeDef *htim_ptr, int channel, int dead_zone_f, int dead_zone_b, float max_speed, bool dest);
    void SetInput(float speed);
    void Stop();
    int map_float_to_int(float x, float min_in, float max_in);
};
#endif