#ifndef __DEEP_SENSOR_HPP__
#define __DEEP_SENSOR_HPP__

class DeepSensor
{
private:
    float press_ = 0.0f;  //压强
    float atp_press_ = 100000.0f;  //大气压强
    float last_press_ = 0.0f;  //前一次压强
    float last2_press_ = 0.0f;  //前两次压强
    float depth_ = 0.0f;  //深度(mm)
    bool IsReady_ = false;  //初始化是否成功
public:
    DeepSensor() {};
    ~DeepSensor() {};
    void Init();
    void UpdateData();
    float GetPress();
    float GetDepth();
};

#endif