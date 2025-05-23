#include "NewImu.hpp"

void NewImu::Init(UART_HandleTypeDef *huart, float deta_time) {
    huart_ = huart;
    dt = deta_time;
    INSReset();
    HAL_UART_Receive_IT(huart_, &rx_data_, 1);
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
    // 帧头检测
    if(rx_data_ == 0x80)
    {
        rx_index_ = 0;
    }
    
    rx_buf_[rx_index_] = rx_data_;
    rx_index_++;
    
    // 完整帧接收
    if(rx_index_ == 38) {
        // 检查帧尾
        if(rx_buf_[36] != 0x0D || rx_buf_[37] != 0x0A) {
            rx_index_ = 0;
            return;
        }
        
        // // 校验和检查
        // uint16_t calc_crc = CalculateCRC16(&rx_buf_[2], 24);
        // uint16_t recv_crc = (rx_buf_[27] << 8) | rx_buf_[26];
        
        // if(calc_crc == recv_crc) {
            // 解析数据
            gyro_x_ = (int16_t)((rx_buf_[7] << 8) | rx_buf_[6]) / 64.0f;
            gyro_y_ = (int16_t)((rx_buf_[9] << 8) | rx_buf_[8]) / 64.0f;
            gyro_z_ = (int16_t)((rx_buf_[11] << 8) | rx_buf_[10]) / 64.0f;
            
            acc_x_ = (int16_t)((rx_buf_[13] << 8) | rx_buf_[12]);
            acc_y_ = (int16_t)((rx_buf_[15] << 8) | rx_buf_[14]);
            acc_z_ = (int16_t)((rx_buf_[17] << 8) | rx_buf_[16]);
            
            pitch_ = (int16_t)((rx_buf_[19] << 8) | rx_buf_[18]) / 100.0f;
            roll_ = (int16_t)((rx_buf_[21] << 8) | rx_buf_[20]) / 100.0f;
            yaw_ = (int16_t)((rx_buf_[23] << 8) | rx_buf_[22]) / 100.0f;
            
            temp_ = (int16_t)((rx_buf_[25] << 8) | rx_buf_[24]) / 100.0f;
            
        //     data_ready_ = true;
        // }
        
        rx_index_ = 0;
    }
    
    HAL_UART_Receive_IT(huart_, &rx_data_, 1);
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

// 角度转弧度
inline float deg2rad(float deg) {
    return deg * M_PI / 180.0f;
}

void NewImu::INSUpdate()
{
    //加速度单位转为m/s^2
    float acc_x_mps2 = acc_x_ * g/1000.0f;  // 1 mg = 0.00980665 m/s²
    float acc_y_mps2 = acc_y_ * g/1000.0f;
    float acc_z_mps2 = acc_z_ * g/1000.0f;

    // 1. 将加速度从本体坐标系转换到世界坐标系
     // 1. 角度制 → 弧度制
    float roll = deg2rad(roll_);
    float pitch = deg2rad(pitch_);
    float yaw = deg2rad(yaw_);

    // 使用当前姿态（roll, pitch, yaw）进行旋转矩阵计算
    float cos_roll = cos(roll);
    float sin_roll = sin(roll);
    float cos_pitch = cos(pitch);
    float sin_pitch = sin(pitch);
    float cos_yaw = cos(yaw);
    float sin_yaw = sin(yaw);
    
    // 旋转矩阵（本体到世界）
    float acc_world_x = acc_x_mps2 * (cos_yaw * cos_pitch) + 
                        acc_y_mps2 * (cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll) + 
                        acc_z_mps2 * (cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll);
    
    float acc_world_y = acc_x_mps2 * (sin_yaw * cos_pitch) + 
                        acc_y_mps2 * (sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll) + 
                        acc_z_mps2 * (sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll);
    
    float acc_world_z = acc_x_mps2 * (-sin_pitch) + 
                        acc_y_mps2 * (cos_pitch * sin_roll) + 
                        acc_z_mps2 * (cos_pitch * cos_roll);
    
    // 2. 去除重力分量（假设z轴向上）
    acc_world_z -= g;
    
    // 3. 积分加速度得到速度（使用梯形积分提高精度）
    velocity_x += (last_acc_world_x + acc_world_x) * 0.5f * dt;
    velocity_y += (last_acc_world_y + acc_world_y) * 0.5f * dt;
    
    // 4. 积分速度得到位移（同样使用梯形积分）
    displacement_x += velocity_x * dt;
    displacement_y += velocity_y * dt;
    
    // 保存当前加速度值供下次使用
    last_acc_world_x = acc_world_x;
    last_acc_world_y = acc_world_y;
}

void NewImu::INSReset()
{
    velocity_x = 0;
    velocity_y = 0;
    displacement_x = 0;
    displacement_y = 0;
    last_acc_world_x = 0;
    last_acc_world_y = 0;
}