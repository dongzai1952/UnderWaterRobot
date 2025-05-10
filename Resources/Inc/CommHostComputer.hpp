#ifndef __COMM_HOST_COMPUTER_HPP__
#define __COMM_HOST_COMPUTER_HPP__

#include "main.h"
#include "DeepSensor.hpp"
#include "DistanceSensor.hpp"

#define RX_BUFF_LEN_MAX 10

class CommHostComputer
{
private:
    //依赖类
    DeepSensor *deep_sensor_ptr_ = nullptr;
    DistanceSensor *distance_sensor_ptr_x_ = nullptr;
    DistanceSensor *distance_sensor_ptr_y_ = nullptr;

    //从上位机接收数据
    uint8_t rx_data_;  //串口接收单字节数据
    uint8_t rx_buf_[RX_BUFF_LEN_MAX];
    uint8_t rx_index_ = 0;  //当前接收位置
    uint8_t frame_len_ = 0;  //帧长度

    //发送给上位机数据
    struct SendDataFrame{
        uint8_t header = 0xEF;     // 0xEF
        float depth = 0.0f;        // 深度
        float x = 0.0f;            // x坐标
        float y = 0.0f;            // y坐标
        uint8_t isOn = 0;  // 是否开启 (0/1)
        uint8_t isGrab = 0; // 是否抓取 (0/1)
        uint8_t controlMode = 0;// 自控(0)/手控(1)
    } send_data_;
public:
    struct Cmd
    {
        float speed_x = 0;  //x方向水平速度
        float speed_y = 0;  //y方向水平速度
        float w = 0;        //转速
        float depth = 0;    //深度
        bool is_on = false; //开关
        bool is_grab = false;  //抓取、放下
        uint8_t control_mode = 1;  //控制模式,默认手控
    } cmd_;  //遥控器控制量
    
    UART_HandleTypeDef *huart_ = nullptr;  //lora串口
    CommHostComputer() {};
    ~CommHostComputer() {};
    void Init(UART_HandleTypeDef *huart);
    void Decode();
    void EncodeAndSendData();
    void Print(uint8_t *data);
    void RegisterDeepSensor(DeepSensor *deep_sensor_ptr)
    {
        deep_sensor_ptr_ = deep_sensor_ptr;
    }
    void RegisteDistanceSensor_x(DistanceSensor *distance_sensor_ptr_x)
    {
        distance_sensor_ptr_x_ = distance_sensor_ptr_x;
    }
    void RegisteDistanceSensor_y(DistanceSensor *distance_sensor_ptr_y)
    {
        distance_sensor_ptr_y_ = distance_sensor_ptr_y;
    }
};

#endif