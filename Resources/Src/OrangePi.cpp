#include "OrangePi.hpp"

void OrangePi::Init(UART_HandleTypeDef *huart)
{
    huart_ = huart;

    //开启串口中断
    HAL_UART_Receive_IT(huart_, &rx_data_, 1);
}

void OrangePi::Decode()
{   
    if(rx_data_ == 0xFF) 
    {
        rx_index_ = 0;  // 检测到帧头，重置索引
    }

    if(rx_data_ == 0xFE)
    {
        is_find_ = false;
    }
    
    rx_buf_[rx_index_] = rx_data_;  // 存储接收到的字节
    rx_index_++;
    
    if(rx_index_ == 6)  // 接收完完整帧(6字节)
    {
        // 校验计算：帧头 + x高 + x低 + y高 + y低
        uint8_t checksum = (rx_buf_[0] + rx_buf_[1] + rx_buf_[2] + rx_buf_[3] + rx_buf_[4]) & 0xFF;
        
        if(rx_buf_[0] == 0xFF && checksum == rx_buf_[5])  // 校验帧头和校验和
        {
            // 合并x坐标高低字节并转换为float
            int16_t x_raw = (rx_buf_[1] << 8) | rx_buf_[2];
            float x = (float)x_raw / 32700.0f;
            
            // 合并y坐标高低字节并转换为float
            int16_t y_raw = (rx_buf_[3] << 8) | rx_buf_[4];
            float y = (float)y_raw / 32700.0f;
            
            // 检查范围是否在-1到1之间
            if(x >= -1.0f && x <= 1.0f && y >= -1.0f && y <= 1.0f)
            {
                pos_.x = x;
                pos_.y = y;
                is_find_ = true;
            }
        }
        rx_index_ = 0;  // 准备接收下一帧
    }
    
    // 重新开启串口中断
    HAL_UART_Receive_IT(huart_, &rx_data_, 1);
}