#include "main_task.hpp"
#include "main.h"
#include "MS5837.h"
#include "usart.h"
#include "tim.h"
#include <deque>
#include <utility> 

#include "DeepSensor.hpp"
#include "DistanceSensor.hpp"
#include "Imu.hpp"
#include "CommHostComputer.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"

enum class AutoState : uint8_t
{
    Search = 0,
    Grab,
    Return,
};

// 定义点位类型（x, y）
using Point = std::pair<float, float>;

//软件计时
float main_task_time_cost = 0;
uint32_t start_time, end_time;
float execution_time_us;
uint64_t main_tick = 0;

//组件指针
DeepSensor *deep_sensor_ptr = new DeepSensor;
DistanceSensor *distance_sensor_x_ptr = new DistanceSensor;
DistanceSensor *distance_sensor_y_ptr = new DistanceSensor;
Imu *imu_ptr = new Imu;
uint8_t rx_data;
Motor* motor_ptr = new Motor[6];
CommHostComputer *lora_ptr = new CommHostComputer;

PIDController *depth_pid_ptr = new PIDController;
PIDController *yaw_pid_ptr = new PIDController;
PIDController *pich_pid_ptr = new PIDController;
PIDController *x_pid_ptr = new PIDController;
PIDController *y_pid_ptr = new PIDController;

//变量定义
float depth_ref = 15.0f;
float depth_cur = 0.0f;
float depth_ffd = 0.0f;
float acc_x_cur = 0.0f;
float acc_y_cur = 0.0f;
float speed_x_cur = 0.0f;
float speed_x_ref = 0.0f;
float speed_y_cur = 0.0f;
float speed_y_ref = 0.0f;
float pos_x_cur = 0.0f;
float pos_x_ref = 0.0f;
float pos_y_cur = 0.0f;
float pos_y_ref = 0.0f;
float yaw_cur = 0.0f;
float yaw_ref = 0.0f;
float last_yaw = 0.0f;
float pich_cur = 0.0f;
float pich_ref = 0.0f;
uint8_t path_index = 0;

AutoState auto_state = AutoState::Search;
std::deque<Point> path;  //机器人运行路径

void Start_Timer_Measurement(void) {
    HAL_TIM_Base_Start(&htim5);  // 替换htimx为您的定时器句柄
    __HAL_TIM_SET_COUNTER(&htim5, 0);
    start_time = __HAL_TIM_GET_COUNTER(&htim5);
}

float Stop_Timer_Measurement(void) {
    end_time = __HAL_TIM_GET_COUNTER(&htim5);
    HAL_TIM_Base_Stop(&htim5);
    
    execution_time_us = (float)(end_time - start_time);  // 已经是us单位
    return execution_time_us;
}

void MainInit()
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);  //开启大功率开关
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);  //pi
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);  //继电器2
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 50);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);  //LED1
    HAL_Delay(10);

    DefinePath();

    deep_sensor_ptr->Init();

    distance_sensor_x_ptr->Init(&huart4);
    distance_sensor_y_ptr->Init(&huart5);

    imu_ptr->Init(&huart1);
    
    lora_ptr->RegisterDeepSensor(deep_sensor_ptr);
    lora_ptr->RegisteDistanceSensor_x(distance_sensor_x_ptr);
    lora_ptr->RegisteDistanceSensor_y(distance_sensor_y_ptr);
    lora_ptr->Init(&huart2);

    motor_ptr[0].Init(&htim4, TIM_CHANNEL_1, 20, 3, 50, 0);
    motor_ptr[1].Init(&htim4, TIM_CHANNEL_4, 20, 3, 50, 0);
    motor_ptr[2].Init(&htim4, TIM_CHANNEL_2, 20, 3, 50, 0);
    motor_ptr[3].Init(&htim1, TIM_CHANNEL_3, 25, 3, 50, 0);
    motor_ptr[4].Init(&htim4, TIM_CHANNEL_3, 3, 20, 50, 1);
    motor_ptr[5].Init(&htim1, TIM_CHANNEL_4, 5, 25, 50, 1);
    HAL_Delay(3000);

    depth_pid_ptr->init(2.0f, 0.01f, 0.0f, 50, 50, 0);  //1.2 0 0
    yaw_pid_ptr->init(0.6f, 0.01f, 0.0f, 50, 50, 0);  //0.3 0 0
    pich_pid_ptr->init(0.5f, 0.0f, 0.1f, 50, 50, 0);
    x_pid_ptr->init(2.0f, 0.0f, 0.0f, 50, 50, 0);
    y_pid_ptr->init(2.0f, 0.0f, 0.0f, 50, 50, 0);

    pich_ref = imu_ptr->GetPich();  //初始pich值为控制目标
    lora_ptr->cmd_.is_on = true;  //运行
    lora_ptr->cmd_.control_mode = 0;

    HAL_Delay(30000);  //计时等待imu稳定
    imu_ptr->SetYawZero();
    //yaw_ref = imu_ptr->GetYaw();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);  //电磁铁开
}

int set_speed=0;
void MainTask()
{
    Start_Timer_Measurement();

    // deep_sensor_ptr->UpdateData();
    // distance_sensor_x_ptr->UpdateData();
    // distance_sensor_y_ptr->UpdateData();
    // if(lora_ptr->cmd_.is_grab == true)
    // {
    //   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);  //继电器1
    // }
    // else
    // {
    //   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);  //继电器1
    // }
    
    //lora_ptr->EncodeAndSendData();  //目前发送数据会导致上位机卡死

    // set_speed = (int)(500.0f*lora_ptr->cmd_.speed_x);

    UpdateData();
    Run();

    main_task_time_cost = Stop_Timer_Measurement();
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim6)
    {
      Start_Timer_Measurement();
      main_tick++;
      MainTask();
      main_task_time_cost = Stop_Timer_Measurement();
    }
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
  else if (huart->Instance == UART4)
  {
    distance_sensor_x_ptr->Decode();
  }
  else if (huart->Instance == UART5)
  {
    distance_sensor_y_ptr->Decode();
  }
}

void UpdateData()
{
  deep_sensor_ptr->UpdateData();
  if(main_tick%2 == 1) distance_sensor_x_ptr->UpdateData();
  else distance_sensor_y_ptr->UpdateData();

  distance_sensor_y_ptr->UpdateData();
  
  if(lora_ptr->cmd_.is_grab == true)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);  //继电器1
  }
  else
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);  //继电器1
  }

  //update curr
  //过零处理
  last_yaw = yaw_cur;
  yaw_cur = imu_ptr->GetYaw();
  // if(yaw_cur - last_yaw > 1.5*180) yaw_cur += 2*180;  //这个过零有问题
  // else if(yaw_cur - last_yaw < -1.5*180) yaw_cur -= 2*180;
  pich_cur = imu_ptr->GetPich();
  depth_cur = deep_sensor_ptr->GetDepth();

  speed_x_cur += imu_ptr->GetAccX() * kControlPeriod;
  speed_y_cur += imu_ptr->GetAccY() * kControlPeriod;

  pos_x_cur = distance_sensor_x_ptr->GetDistance();
  pos_y_cur = distance_sensor_y_ptr->GetDistance();
}

void Run()
{
  if(lora_ptr->cmd_.is_on)
  {
      if(lora_ptr->cmd_.control_mode == 1)
      {
        RunOnControl();
      }
      else
      {
        RunOnAuto();
      }
  }
  else
  {
      RunOnDead();
  }
  
}


void RunOnControl()
{
  //update ref
  depth_ref -= lora_ptr->cmd_.depth * kDepthSpeed * kControlPeriod;
  yaw_ref -= lora_ptr->cmd_.w * kRoundSpeed * kControlPeriod;
  pich_ref = 0;
  speed_x_ref = lora_ptr->cmd_.speed_x * kSpeedX;
  speed_y_ref = lora_ptr->cmd_.speed_y * kSpeedY;

  //计算PID
  float motor_set[6] = {0};
  //深度
  float depth_cacu = depth_pid_ptr->cacu(depth_ref, depth_cur, kControlPeriod);
  motor_set[1] -= depth_cacu;
  motor_set[4] -= depth_cacu;
  //加上重力前馈
  motor_set[1] += depth_ffd;
  motor_set[4] += depth_ffd;


  //yaw
  float yaw_cacu = yaw_pid_ptr->cacu(yaw_ref, yaw_cur, kControlPeriod);
  motor_set[0] -= yaw_cacu;
  motor_set[2] += yaw_cacu;
  motor_set[3] += yaw_cacu;
  motor_set[5] -= yaw_cacu;

  //pich
  float pich_cacu = pich_pid_ptr->cacu(pich_ref, pich_cur, kControlPeriod);
  motor_set[1] -= pich_cacu;
  motor_set[4] += pich_cacu;

  //speed_x
  motor_set[0] += speed_x_ref;
  motor_set[2] -= speed_x_ref;
  motor_set[3] += speed_x_ref;
  motor_set[5] -= speed_x_ref;

  //speed_y
  motor_set[0] += speed_y_ref;
  motor_set[2] += speed_y_ref;
  motor_set[3] += speed_y_ref;
  motor_set[5] += speed_y_ref;

  //设置电机
  for(int i=0; i<6; i++)
  {
    if(i==1 || i==4)
    {
      motor_set[i] = Bound(motor_set[i], -40.0f, 40.0f);
    }
    else
    {
      motor_set[i] = Bound(motor_set[i], -30.0f, 30.0f);
    }
    motor_ptr[i].SetInput(motor_set[i]);
  }
}

void RunOnAuto()
{
  SetSearchRef();

  //计算PID
  float motor_set[6] = {0};
  //深度
  float depth_cacu = depth_pid_ptr->cacu(depth_ref, depth_cur, kControlPeriod);
  motor_set[1] -= depth_cacu;
  motor_set[4] -= depth_cacu;
  //加上重力前馈
  motor_set[1] += depth_ffd;
  motor_set[4] += depth_ffd;


  //yaw
  float yaw_cacu = yaw_pid_ptr->cacu(yaw_ref, yaw_cur, kControlPeriod);
  motor_set[0] -= yaw_cacu;
  motor_set[2] += yaw_cacu;
  motor_set[3] += yaw_cacu;
  motor_set[5] -= yaw_cacu;

  //pich
  float pich_cacu = pich_pid_ptr->cacu(pich_ref, pich_cur, kControlPeriod);
  motor_set[1] -= pich_cacu;
  motor_set[4] += pich_cacu;

  //speed_x
  float speed_x_cacu = x_pid_ptr->cacu(pos_x_ref, pos_x_cur, kControlPeriod);
  motor_set[0] += speed_x_cacu;
  motor_set[2] -= speed_x_cacu;
  motor_set[3] += speed_x_cacu;
  motor_set[5] -= speed_x_cacu;

  //speed_y
  float speed_y_cacu = y_pid_ptr->cacu(pos_y_ref, pos_y_cur, kControlPeriod);
  motor_set[0] += speed_y_cacu;
  motor_set[2] += speed_y_cacu;
  motor_set[3] += speed_y_cacu;
  motor_set[5] += speed_y_cacu;

  //设置电机
  for(int i=0; i<6; i++)
  {
    if(i==1 || i==4)
    {
      motor_set[i] = Bound(motor_set[i], -40.0f, 40.0f);
    }
    else
    {
      motor_set[i] = Bound(motor_set[i], -30.0f, 30.0f);
    }
    motor_ptr[i].SetInput(motor_set[i]);
  }
}

void SetSearchRef()
{
  //update ref
  depth_ref = 15;
  //yaw_ref = 0;
  pich_ref = 0;

  if(fabs(path[path_index].first - pos_x_cur) < kPointTH && fabs(path[path_index].second - pos_y_cur) < kPointTH)
  {
    path_index++;
    if(path_index > 3) path_index = 0;
  }
  pos_x_ref = path[path_index].first;
  pos_y_ref = path[path_index].second;

  pos_x_ref = LimDiff(pos_x_ref, pos_x_cur, 200);
  pos_y_ref = LimDiff(pos_y_ref, pos_y_cur, 200);

  // pos_x_ref = 120;
  // pos_y_ref = 80;
  // pos_x_ref = LimDiff(pos_x_ref, pos_x_cur, 200);
  // pos_y_ref = LimDiff(pos_y_ref, pos_y_cur, 200);
}

void RunOnDead()
{
  for(int i=0; i<6; i++)
  {
    motor_ptr[i].Stop();
    //imu_ptr->ResetYaw();  //z轴置零  这个东西会花很长时间谨慎调用
    ResetPID();
  }
}

void ResetPID()
{
  depth_pid_ptr->reset();
  yaw_pid_ptr->reset();
  pich_pid_ptr->reset();
  x_pid_ptr->reset();
  y_pid_ptr->reset();
}

void DefinePath()
{
  path.push_back({70.0, 70.0});
  path.push_back({120.0, 120.0});
  path.push_back({120.0, 120.0});
  path.push_back({70.0, 120.0});
}

float Bound(float x, float lim1, float lim2)
{
  float max_lim, min_lim;

  /* 设置上下限 */
  if (lim1 >= lim2) {
    max_lim = lim1;
    min_lim = lim2;
  } else {
    max_lim = lim2;
    min_lim = lim1;
  }

  /* 限制范围 */
  if (x > max_lim) {
    return max_lim;
  } else if (x < min_lim) {
    return min_lim;
  } else {
    return x;
  }
}

/**
  * @brief       限定变化率
  * @param       ref_vel: 期望值
  * @param       curr_vel: 当前值
  * @param       max_diff: 最大的变化速率
  * @retval      符合限定变化率的值
  * @note        None
  */
float LimDiff(float ref_vel, float curr_vel, float max_diff)
{
    float dvel = ref_vel - curr_vel;

    /* 限定变化差值 */
    if (dvel > max_diff * kControlPeriod) {
        dvel = max_diff * kControlPeriod;
    } else if (dvel < -max_diff * kControlPeriod) {
            dvel = -max_diff * kControlPeriod;
    }

    return curr_vel + dvel;
};