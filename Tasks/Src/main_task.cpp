#include "main_task.hpp"
#include "main.h"
#include "MS5837.h"
#include "usart.h"
#include "tim.h"

#include "DeepSensor.hpp"
#include "DistanceSensor.hpp"
#include "Imu.hpp"
#include "CommHostComputer.hpp"
#include "Motor.hpp"

DeepSensor *deep_sensor_ptr = new DeepSensor;
DistanceSensor *distance_sensor_x_ptr = new DistanceSensor;
DistanceSensor *distance_sensor_y_ptr = new DistanceSensor;
Imu *imu_ptr = new Imu;
uint8_t rx_data;
Motor *motor1_ptr = new Motor;
Motor *motor2_ptr = new Motor; 
Motor *motor3_ptr = new Motor; 
Motor *motor4_ptr = new Motor; 
Motor *motor5_ptr = new Motor; 
Motor *motor6_ptr = new Motor;  
float ps=0;

CommHostComputer *lora_ptr = new CommHostComputer;


void MainInit()
{
    deep_sensor_ptr->Init();

    distance_sensor_x_ptr->Init(&huart5);
    distance_sensor_y_ptr->Init(&huart6);

    imu_ptr->Init(&huart1);
    
    lora_ptr->RegisterDeepSensor(deep_sensor_ptr);
    lora_ptr->RegisteDistanceSensor_x(distance_sensor_x_ptr);
    lora_ptr->RegisteDistanceSensor_x(distance_sensor_y_ptr);
    lora_ptr->Init(&huart2);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);  //开启大功率开关
    HAL_Delay(10);

    motor1_ptr->Init(&htim4, TIM_CHANNEL_1, 5, 5, 500);    //转不了
    //HAL_Delay(10000);
    motor2_ptr->Init(&htim4, TIM_CHANNEL_2, 5, 5, 500);    //能跑
    //HAL_Delay(10000);
    motor3_ptr->Init(&htim4, TIM_CHANNEL_3, 5, 5, 500);    //转不了
    //HAL_Delay(10000);
    motor4_ptr->Init(&htim4, TIM_CHANNEL_4, 5, 5, 500);    //能跑
    //HAL_Delay(10000);
    motor5_ptr->Init(&htim1, TIM_CHANNEL_3, 5, 5, 500);  //转不了
    //HAL_Delay(10000);
    motor6_ptr->Init(&htim1, TIM_CHANNEL_4, 5, 5, 500);
    HAL_Delay(10000);
}

int set_speed=0;
void MainTask()
{
    deep_sensor_ptr->UpdateData();
    distance_sensor_x_ptr->UpdateData();
    distance_sensor_y_ptr->UpdateData();
    if(lora_ptr->cmd_.is_grab == true)
    {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);  //继电器1
    }
    else
    {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);  //继电器1
    }
    
    lora_ptr->EncodeAndSendData();
    HAL_Delay(100);
    //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    // set_speed = (int)(500.0f*lora_ptr->cmd_.speed_x);
    // motor_ptr->SetInput(set_speed);

    motor1_ptr->SetInput(200);
    motor2_ptr->SetInput(200);
    motor3_ptr->SetInput(200);
    motor4_ptr->SetInput(200);
    motor5_ptr->SetInput(200);
    motor6_ptr->SetInput(200);
    //HAL_Delay(20);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    imu_ptr->Decode();
  }
  else if (huart->Instance == USART2)
  {
    lora_ptr->Decode();
  }
  else if (huart->Instance == UART5)
  {
    distance_sensor_x_ptr->Decode();
  }
  else if (huart->Instance == USART6)
  {
    distance_sensor_y_ptr->Decode();
  }
}