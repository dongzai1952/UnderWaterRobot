#ifndef __COMM_HOST_COMPUTER_HPP__
#define __COMM_HOST_COMPUTER_HPP__

#include "main.h"
#define RX_BUFF_LEN_MAX 10

class CommHostComputer
{
private:
    uint8_t rx_data_;  //串口接收单字节数据
    uint8_t rx_buf_[RX_BUFF_LEN_MAX];
    uint8_t rx_index_ = 0;  //当前接收位置
    uint8_t frame_len_ = 0;  //帧长度
public:
    struct Cmd
    {
        float speed_x = 0;  //x方向水平速度
        float speed_y = 0;  //y方向水平速度
        float w = 0;        //转速
        float depth = 0;    //深度
        bool is_on = false; //开关
        bool is_grab = false;  //抓取、放下
    } cmd_;  //遥控器控制量
    
    UART_HandleTypeDef *huart_ = nullptr;  //lora串口
    CommHostComputer() {};
    ~CommHostComputer() {};
    void Init(UART_HandleTypeDef *huart);
    void Decode();
    void SendData();
    void Print();
};

#endif