#include "main_task.hpp"
#include "main.h"
#include "MS5837.h"
#include "usart.h"
#include "tim.h"

#include "DeepSensor.hpp"
#include "Imu.hpp"
#include "CommHostComputer.hpp"
#include "Motor.hpp"

DeepSensor *deep_sensor_ptr = new DeepSensor;
Imu *imu_ptr = new Imu;
uint8_t rx_data;
Motor *motor_ptr = new Motor; 
float ps=0;

CommHostComputer *lora_ptr = new CommHostComputer;


void MainInit()
{
    //deep_sensor_ptr->Init();
    //imu_ptr->Init(&huart1);
    lora_ptr->Init(&huart2);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);  //开启大功率开关
    HAL_Delay(100);
    //motor_ptr->Init(&htim24, TIM_CHANNEL_4, 5, 5, 500);
    //HAL_TIM_PWM_Start(&htim24,TIM_CHANNEL_4);
    //__HAL_TIM_SetCompare(&htim24,TIM_CHANNEL_4,1500);
    //HAL_Delay(10000);
}

int set_speed=0;
void MainTask()
{
    //deep_sensor_ptr->UpdateData();
    //ps=deep_sensor_ptr->GetPress();
    if(lora_ptr->cmd_.is_grab == true)
    {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);  //继电器1
    }
    else
    {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);  //继电器1
    }
    
    //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    // set_speed = (int)(500.0f*lora_ptr->cmd_.speed_x);
    // motor_ptr->SetInput(set_speed);
    //motor_ptr->SetInput(200);
    //HAL_Delay(20);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    //imu_ptr->Decode();
  }
  else if (huart->Instance == USART2)
  {
    lora_ptr->Decode();
  }
}