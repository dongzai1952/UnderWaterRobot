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
    if(rx_data_==0xFF || rx_data_==0xFE)
    {
        rx_index_ = 0;  //帧头 
        frame_len_ = 6;
    }
    else if(rx_data_==0xFD)
    {
        rx_index_ = 0;  //帧头 
        frame_len_ = 4;
    }
    else if(rx_data_==0xFC || rx_data_==0xFB || rx_data_==0xFA)
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
        if (rx_buf_[0]!=0xFF && rx_buf_[0]!=0xFE && rx_buf_[0]!=0xFD && rx_buf_[0]!=0xFC && rx_buf_[0]!=0xFB && rx_buf_[0]!=0xFA)
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

        case 0xFA:
            {
            if(rx_buf_[1] == 0x01)
            {
                cmd_.control_mode = !cmd_.control_mode;
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

void CommHostComputer::EncodeAndSendData()
{
    send_data_.header = 0xEF;
    if(deep_sensor_ptr_ != nullptr) send_data_.depth = deep_sensor_ptr_->GetDepth();
    if(distance_sensor_ptr_x_ != nullptr) send_data_.x = distance_sensor_ptr_x_->GetDistance();
    if(distance_sensor_ptr_y_ != nullptr) send_data_.y = distance_sensor_ptr_y_->GetDistance();
    send_data_.isOn = cmd_.is_on;
    send_data_.isGrab = cmd_.is_grab;
    send_data_.controlMode = cmd_.control_mode;

    uint8_t txBuffer[11]; // 11字节的发送缓冲区
    uint8_t checksum = 0;
    
    // 1. 设置帧头
    txBuffer[0] = 0xEF;
    checksum += txBuffer[0];
    
    // 2. 编码深度 (假设乘以100转换为整数)
    int16_t depthInt = (int16_t)(send_data_.depth * 100);
    txBuffer[1] = (uint8_t)((depthInt >> 8) & 0xFF); // 高字节
    txBuffer[2] = (uint8_t)(depthInt & 0xFF);        // 低字节
    checksum += txBuffer[1] + txBuffer[2];
    
    // 3. 编码x坐标 (假设乘以100转换为整数)
    int16_t xInt = (int16_t)(send_data_.x * 100);
    txBuffer[3] = (uint8_t)((xInt >> 8) & 0xFF); // 高字节
    txBuffer[4] = (uint8_t)(xInt & 0xFF);        // 低字节
    checksum += txBuffer[3] + txBuffer[4];
    
    // 4. 编码y坐标 (假设乘以100转换为整数)
    int16_t yInt = (int16_t)(send_data_.y * 100);
    txBuffer[5] = (uint8_t)((yInt >> 8) & 0xFF); // 高字节
    txBuffer[6] = (uint8_t)(yInt & 0xFF);        // 低字节
    checksum += txBuffer[5] + txBuffer[6];
    
    // 5. 编码状态标志
    txBuffer[7] = send_data_.isOn ? 0x01 : 0x00;
    txBuffer[8] = send_data_.isGrab ? 0x01 : 0x00;
    checksum += txBuffer[7] + txBuffer[8];
    
    // 6. 编码控制模式
    txBuffer[9] = send_data_.controlMode;
    checksum += txBuffer[9];
    
    // 7. 计算并设置校验和
    txBuffer[10] = checksum;
    
    // 8. 通过串口发送数据
    HAL_UART_Transmit(huart_, txBuffer, sizeof(txBuffer), HAL_MAX_DELAY);
}

void CommHostComputer::Print(uint8_t *data)
{
    int data_len = 0;
    for(int i=0; data[i]!='\0'&&i<50; i++)
    {
        data_len++;
    }
    HAL_UART_Transmit(huart_, data, data_len, HAL_MAX_DELAY);
}