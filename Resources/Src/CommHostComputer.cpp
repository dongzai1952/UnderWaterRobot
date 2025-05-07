#include "CommHostComputer.hpp"

void CommHostComputer::Init(UART_HandleTypeDef *huart)
{
    huart_ = huart;

    //M1和M0置低电平
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);

    //开启串口中断
    HAL_UART_Receive_IT(huart_, &rx_data_, 1);
}

void CommHostComputer::Decode()
{
    if(rx_data_==0xff || rx_data_==0xfe)
    {
        rx_index_ = 0;  //帧头 
        frame_len_ = 6;
    }
    else if(rx_data_==0xfd)
    {
        rx_index_ = 0;  //帧头 
        frame_len_ = 4;
    }
    else if(rx_data_==0xfc || rx_data_==0xfb)
    {
        rx_index_ = 0;  //帧头 
        frame_len_ = 3;
    }
    
    rx_buf_[rx_index_] = rx_data_;  // 存储接收到的字符
    rx_index_++;
    if(rx_index_ > RX_BUFF_LEN_MAX-1) rx_index_ = 0;  //防止越界

    if(rx_index_ == frame_len_)  //接收完一帧数据
    {
        //检查帧头
        if (rx_buf_[0]!=0xFF && rx_buf_[0]!=0xFE && rx_buf_[0]!=0xFD && rx_buf_[0]!=0xFC && rx_buf_[0]!=0xFB)
        {
            rx_index_ = 0;
            HAL_UART_Receive_IT(huart_, &rx_data_, 1);
            return;
        } 

        //检查校验和
        uint8_t checksum = 0;
        for(int i=1; i<frame_len_-1; i++)
        {
            checksum += rx_buf_[i];
        }
        checksum &= 0xFF;
        if(checksum != rx_buf_[frame_len_-1] && checksum != rx_buf_[frame_len_-1]-0x0F)
        {
            rx_index_ = 0;
            HAL_UART_Receive_IT(huart_, &rx_data_, 1);
            return;
        }

        //解包
        switch (rx_buf_[0])
        {
        case 0xFF:
            {
            // 重组 Int16
            int16_t x = (rx_buf_[1] << 8) | rx_buf_[2];
            int16_t y = (rx_buf_[3] << 8) | rx_buf_[4];
            // 映射回 float（-1.0 ~ 1.0）
            cmd_.speed_x = (float)x / 32757.0f;
            cmd_.speed_y = (float)y / 32757.0f;
            }
            break;

        case 0xFE:
            {
            // 重组 Int16
            int16_t x = (rx_buf_[1] << 8) | rx_buf_[2];
            //int16_t y = (rx_buf_[3] << 8) | rx_buf_[4];
            // 映射回 float（-1.0 ~ 1.0）
            cmd_.w = (float)x / 32757.0f;
            //cmd_.speed_y = (float)y / 32757.0f;  //y暂时用不到
            }
            break;

        case 0xFD:
            {
            // 重组 Int16
            int16_t x = (rx_buf_[1] << 8) | rx_buf_[2];
            cmd_.depth = (float)x / 32757.0f;
            }
            break;

        case 0xFC:
            {
            if(rx_buf_[1] == 0x01)
            {
                cmd_.is_on = !cmd_.is_on;
            }
            }
            break;

        case 0xFB:
            {
            if(rx_buf_[1] == 0x01)
            {
                cmd_.is_grab = !cmd_.is_grab;
            }
            }
            break;

        default:
            break;
        }
    }

    //开启串口中断
    HAL_UART_Receive_IT(huart_, &rx_data_, 1);
}