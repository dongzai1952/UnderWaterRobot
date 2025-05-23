#include "DistanceSensor.hpp"

void DistanceSensor::Init(UART_HandleTypeDef *huart)
{
    huart_ = huart;

    //开启串口中断
    HAL_UART_Receive_IT(huart_, &rx_data_, 1);
}

void DistanceSensor::UpdateData()
{
    HAL_UART_Transmit(huart_, tx_buff_, 1, 0xffff);  //发送一帧数据
}

void DistanceSensor::Decode()
{
    if(rx_data_==0xff) rx_index_=0;  //帧头
    rx_buf_[rx_index_] = rx_data_;  // 存储接收到的字符
    rx_index_++;

    last_last_distance_ = last_distance_;
    last_distance_ = distance_;
    last_receive_success_ = receive_success_;
    if(rx_index_ == 4)  //接收完一帧数据
    {
        if (rx_buf_[0] == 0xFF && (((rx_buf_[0] + rx_buf_[1] + rx_buf_[2])&0x00ff) == rx_buf_[3]))  //成功接收数据
        {
            distance_ = (rx_buf_[1] << 8) | rx_buf_[2];  //合并高低字节
            distance_f_ = distance_ / 10.0;
        }
        else
        {
            rx_index_ = 0;  //数据错误重新接收
            receive_success_ = false;
        }
    }

    if(last_distance_ == distance_)
    {
        receive_fail_tick_ ++;
        if(receive_fail_tick_ > 50)
        {
            receive_success_ = false;
            receive_fail_tick_ = 0;
        }
    }
    else if(last_receive_success_ && (distance_ - last_distance_ > 50))
    {
        receive_success_ = false;
    }
    else if(distance_ < 10)
    {
        receive_success_ = false;
    }
    else receive_success_ = true;
    //开启串口中断
    HAL_UART_Receive_IT(huart_, &rx_data_, 1);
}