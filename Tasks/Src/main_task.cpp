#include "main_task.hpp"
#include "main.h"
#include "MS5837.h"
#include "usart.h"

#include "DeepSensor.hpp"
#include "Imu.hpp"

DeepSensor *deep_sensor_ptr = new DeepSensor;
Imu *imu_ptr = new Imu;
uint8_t rx_data;
float ps=0;

void MainInit()
{
    //deep_sensor_ptr->Init();
    imu_ptr->Init(&huart1);
}

void MainTask()
{
    //deep_sensor_ptr->UpdateData();
    //ps=deep_sensor_ptr->GetPress();
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    imu_ptr->Decode();
  }
}