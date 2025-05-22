#include "NewImu.hpp"

void NewImu::Init(UART_HandleTypeDef *huart, uint8_t *imu_rx_buf) {
    huart_ = huart;
    imu_rx_buf_ = imu_rx_buf;
    HAL_UART_Receive_DMA(huart_, imu_rx_buf_, IMU_FRAME_SIZE);
    __HAL_UART_ENABLE_IT(huart_, UART_IT_IDLE);  // 手动使能IDLE中断
}

uint16_t NewImu::CalculateCRC16(const uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for(uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for(uint8_t j = 0; j < 8; j++) {
            if(crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void NewImu::Decode() {
    // // 帧头检测
    // if(rx_index_ == 0 && rx_data_ != 0x80) return;
    // if(rx_index_ == 1 && rx_data_ != 0x01) {
    //     rx_index_ = 0;
    //     return;
    // }
    
    // rx_buf_[rx_index_++] = rx_data_;
    
    // // 完整帧接收
    // if(rx_index_ >= 30) {
    //     // 检查帧尾
    //     if(rx_buf_[28] != 0x0D || rx_buf_[29] != 0x0A) {
    //         rx_index_ = 0;
    //         return;
    //     }
        
    //     // // 校验和检查
    //     // uint16_t calc_crc = CalculateCRC16(&rx_buf_[2], 24);
    //     // uint16_t recv_crc = (rx_buf_[27] << 8) | rx_buf_[26];
        
    if (imu_rx_buf_[0] == 0x80 && imu_rx_buf_[1] == 0x01 &&  // 帧头
        imu_rx_buf_[28] == 0x0D && imu_rx_buf_[29] == 0x0A)  // 帧尾
    {
        // 解析数据
        gyro_x_ = (int16_t)((imu_rx_buf_[7] << 8) | imu_rx_buf_[6]) / 64.0f;
        gyro_y_ = (int16_t)((imu_rx_buf_[9] << 8) | imu_rx_buf_[8]) / 64.0f;
        gyro_z_ = (int16_t)((imu_rx_buf_[11] << 8) | imu_rx_buf_[10]) / 64.0f;
        
        acc_x_ = (int16_t)((imu_rx_buf_[13] << 8) | imu_rx_buf_[12]);
        acc_y_ = (int16_t)((imu_rx_buf_[15] << 8) | imu_rx_buf_[14]);
        acc_z_ = (int16_t)((imu_rx_buf_[17] << 8) | imu_rx_buf_[16]);
        
        pitch_ = (int16_t)((imu_rx_buf_[19] << 8) | imu_rx_buf_[18]) / 100.0f;
        roll_ = (int16_t)((imu_rx_buf_[21] << 8) | imu_rx_buf_[20]) / 100.0f;
        yaw_ = (int16_t)((imu_rx_buf_[23] << 8) | imu_rx_buf_[22]) / 100.0f;
        
        temp_ = (int16_t)((imu_rx_buf_[25] << 8) | imu_rx_buf_[24]) / 100.0f;
    }
    else return;
    //     //if(calc_crc == recv_crc) {
    //         // 解析数据
    //         gyro_x_ = (int16_t)((rx_buf_[7] << 8) | rx_buf_[6]) / 64.0f;
    //         gyro_y_ = (int16_t)((rx_buf_[9] << 8) | rx_buf_[8]) / 64.0f;
    //         gyro_z_ = (int16_t)((rx_buf_[11] << 8) | rx_buf_[10]) / 64.0f;
            
    //         acc_x_ = (int16_t)((rx_buf_[13] << 8) | rx_buf_[12]);
    //         acc_y_ = (int16_t)((rx_buf_[15] << 8) | rx_buf_[14]);
    //         acc_z_ = (int16_t)((rx_buf_[17] << 8) | rx_buf_[16]);
            
    //         pitch_ = (int16_t)((rx_buf_[19] << 8) | rx_buf_[18]) / 100.0f;
    //         roll_ = (int16_t)((rx_buf_[21] << 8) | rx_buf_[20]) / 100.0f;
    //         yaw_ = (int16_t)((rx_buf_[23] << 8) | rx_buf_[22]) / 100.0f;
            
    //         temp_ = (int16_t)((rx_buf_[25] << 8) | rx_buf_[24]) / 100.0f;
            
    //         data_ready_ = true;
    //     //}
        
    //     rx_index_ = 0;
    // }
    
    //HAL_UART_Receive_IT(huart_, &rx_data_, 1);
}

void NewImu::ResetYaw() {
    uint8_t cmd[] = {0xFF, 0x1E, 0x02, 0x00, 0x00, 0x00, 0xBD, 0xAE};
    HAL_UART_Transmit(huart_, cmd, sizeof(cmd), HAL_MAX_DELAY);
    yaw_zero_ = 0;
}

void NewImu::SetYaw(float target_yaw) {
    int16_t yaw_value = static_cast<int16_t>(target_yaw * 100);
    uint8_t lsb = yaw_value & 0xFF;
    uint8_t hsb = (yaw_value >> 8) & 0xFF;
    
    uint8_t cmd[8] = {0xFF, 0x1E, lsb, hsb, 0x00, 0x00};
    uint16_t crc = CalculateCRC16(&cmd[2], 4);
    cmd[6] = crc & 0xFF;
    cmd[7] = (crc >> 8) & 0xFF;
    
    HAL_UART_Transmit(huart_, cmd, sizeof(cmd), HAL_MAX_DELAY);
    yaw_zero_ = yaw_ - target_yaw;
}

void NewImu::StartAutoOutput() {
    uint8_t cmd[] = {0xFF, 0x07, 0x00, 0x00, 0x00, 0x00, 0xA1, 0xD4};
    HAL_UART_Transmit(huart_, cmd, sizeof(cmd), HAL_MAX_DELAY);
}

void NewImu::StopAutoOutput() {
    uint8_t cmd[] = {0xFF, 0x07, 0x00, 0x06, 0x00, 0x00, 0x41, 0xD5};
    HAL_UART_Transmit(huart_, cmd, sizeof(cmd), HAL_MAX_DELAY);
}

void NewImu::GyroCalibration() {
    uint8_t cmd[] = {0xFF, 0x1C, 0x00, 0x00, 0x00, 0x00, 0xC5, 0xD6};
    HAL_UART_Transmit(huart_, cmd, sizeof(cmd), HAL_MAX_DELAY);
    HAL_Delay(5000);  // 校准需要时间
}

void NewImu::SaveToFlash() {
    uint8_t cmd[] = {0xFF, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x70, 0x16};
    HAL_UART_Transmit(huart_, cmd, sizeof(cmd), HAL_MAX_DELAY);
}