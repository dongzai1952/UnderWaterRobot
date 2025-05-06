#include "DeepSensor.hpp"
#include "MS5837.h"

void DeepSensor::Init()
{
    IsReady_ = MS5837Init();
}

void DeepSensor::UpdateData()
{
    last2_press_ = last_press_;
	last_press_ = press_;
    press_ = MS5837DataGet();
    depth_ = 1000*((last2_press_+last_press_+press_)/3-100250)/9810;  //压强计算深度
}

float DeepSensor::GetPress()
{
    return press_;
}

float DeepSensor::GetDepth()
{
    return depth_;
}