#include "DistanceSensor.hpp"

void DistanceSensor::Init(UART_HandleTypeDef *huart)
{
    huart_ = huart;
}

void DistanceSensor::UpdateData()
{
    HAL_UART_Transmit(huart_, tx_buff_, 1, 0xffff);  //发送一帧数据
}

/**
 * @brief       解码函数，自动将接收到的一字节数据合并并解码
 * @param       rx_data: 接收到的一字节数据
 * @arg         None
 * @retval      None
 * @note        None
 */
void DistanceSensor::Decode(uint8_t rx_data)
{
    if(rx_data==0xff) rx_index_=0;  //帧头
    rx_buf_[rx_index_] = rx_data;  // 存储接收到的字符
    rx_index_++;
    if(rx_index_ == 4)  //接收完一帧数据
    {
        if (rx_buf_[0] == 0xFF && (((rx_buf_[0] + rx_buf_[1] + rx_buf_[2])&0x00ff) == rx_buf_[3])) {
            distance_ = (rx_buf_[1] << 8) | rx_buf_[2];  //合并高低字节
        }
        else rx_index_ = 0;  //数据错误重新接收
    }
}