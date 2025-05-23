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
#include "NewImu.hpp"
#include "CommHostComputer.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"
#include "OrangePi.hpp"

void SetShit(float speedX, float speedY, float depth, bool isEMOn, uint64_t time);

enum class AutoState : uint8_t
{
    Search = 0,
    Grab,
    Return,
};

enum class GrabState : uint8_t
{
    Down = 0,  //下降
    Grab,
    Up,  //上升
};

enum class ReturnState : uint8_t
{
    Back = 0,  //回程
    Drop,  //释放
};

struct Point {
    float x;
    float y;
    bool need_grasp;  // 是否需要抓取
};

struct Shit
{
    uint64_t keep_time;  //持续时间
    uint64_t head_time;  //开始时间
    float speedX;
    float speedY;
    float depth;
    bool isEMOn;
};

//软件计时
float main_task_time_cost = 0;
uint32_t start_time, end_time;
float execution_time_us;
uint64_t main_tick = 0;

//组件指针
DeepSensor *deep_sensor_ptr = new DeepSensor;
DistanceSensor *distance_sensor_x_ptr = new DistanceSensor;
DistanceSensor *distance_sensor_y_ptr = new DistanceSensor;
//Imu *imu_ptr = new Imu;
NewImu *imu_ptr = new NewImu;
uint8_t rx_data;
Motor* motor_ptr = new Motor[6];
CommHostComputer *lora_ptr = new CommHostComputer;
OrangePi *pi_ptr = new OrangePi;

PIDController *depth_pid_ptr = new PIDController;
PIDController *yaw_pid_ptr = new PIDController;
PIDController *pich_pid_ptr = new PIDController;
PIDController *x_pid_ptr = new PIDController;
PIDController *y_pid_ptr = new PIDController;

//变量定义
float depth_ref = kDepthNormal;
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
float last_yaw_sum = 0.0f;
uint16_t last_yaw_cnt = 0;
float pitch_cur = 0.0f;
float pitch_ref = 0.0f;
uint8_t path_index = 0;  //路径目录
uint8_t is_grab = 0;  //是否抓取到标志
uint8_t is_return = 0;  //是否返回的标志
int16_t grab_tick = 0;
int16_t return_tick = 0;
int16_t drop_tick = 0;
Point basket_point = {0, 0, false};  //篮筐位置
Shit shit[100];
uint32_t shit_index = 0;
uint64_t head_time = 0;  //Shit使用

AutoState auto_state = AutoState::Search;
GrabState grab_state = GrabState::Down;
ReturnState return_state = ReturnState::Back;
std::deque<Point> path;  //机器人运行路径

uint64_t ref_time = 0;  //预期运行时间

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
    // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);  //电磁铁开
    // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);  //继电器2
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 50);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);  //LED1
    HAL_Delay(10);

    DefinePath();

    deep_sensor_ptr->Init();

    distance_sensor_x_ptr->Init(&huart4);
    distance_sensor_y_ptr->Init(&huart5);

    imu_ptr->Init(&huart8, kControlPeriod);
    //imu_ptr->StartAutoOutput();
    
    lora_ptr->RegisterDeepSensor(deep_sensor_ptr);
    lora_ptr->RegisteDistanceSensor_x(distance_sensor_x_ptr);
    lora_ptr->RegisteDistanceSensor_y(distance_sensor_y_ptr);
    lora_ptr->Init(&huart2);

    pi_ptr->Init(&huart3);

    motor_ptr[0].Init(&htim4, TIM_CHANNEL_1, 20, 3, 50, 0);
    motor_ptr[1].Init(&htim4, TIM_CHANNEL_4, 20, 3, 50, 0);
    motor_ptr[2].Init(&htim4, TIM_CHANNEL_2, 20, 3, 50, 0);
    motor_ptr[3].Init(&htim1, TIM_CHANNEL_3, 25, 3, 50, 0);
    motor_ptr[4].Init(&htim4, TIM_CHANNEL_3, 3, 20, 50, 1);
    motor_ptr[5].Init(&htim1, TIM_CHANNEL_4, 5, 25, 50, 1);
    HAL_Delay(3000);

    depth_pid_ptr->init(4.0f, 0.2f, 0.0f, 50, 50, 0);  //1.2 0 0
    yaw_pid_ptr->init(0.4f, 0.01f, 0.0f, 50, 50, 0);  //0.3 0 0
    pich_pid_ptr->init(0.5f, 0.0f, 0.1f, 50, 50, 0);
    x_pid_ptr->init(2.0f, 0.0f, 0.0f, 50, 50, 0);
    y_pid_ptr->init(2.0f, 0.0f, 0.0f, 50, 50, 0);

    pitch_ref = imu_ptr->GetPitch();  //初始pitch值为控制目标
    lora_ptr->cmd_.is_on = true;  //运行
    lora_ptr->cmd_.control_mode = 0;  //自动控制

    imu_ptr->ResetYaw();
    //yaw_ref = imu_ptr->GetYaw();

    ShitInit();
}

int set_speed=0;
void MainTask()
{
    UpdateData();
    //Run();
    RunOnShit();
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
  if (huart->Instance == UART8)
  {
    imu_ptr->Decode();
  }
  else if (huart->Instance == USART2)
  {
    lora_ptr->Decode();
  }
  else if (huart->Instance == USART3)
  {
    pi_ptr->Decode();
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
  if(main_tick%10 == 0) distance_sensor_x_ptr->UpdateData();
  else if(main_tick%10 == 5) distance_sensor_y_ptr->UpdateData();
  
  if(lora_ptr->cmd_.is_grab == true)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);  //继电器1
  }
  else
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);  //继电器1
  }

  //update curr
  last_yaw = yaw_cur;
  yaw_cur = imu_ptr->GetYaw();
  
  last_yaw_sum += yaw_cur;
  last_yaw_cnt ++;
  if(main_tick % (300) == 0)  //每3s更新
  {
    last_yaw = last_yaw_sum/last_yaw_cnt;  //取平均
    last_yaw_sum = 0;
    last_yaw_cnt = 0;
  }
  pitch_cur = imu_ptr->GetPitch();
  depth_cur = deep_sensor_ptr->GetDepth();

  imu_ptr->INSUpdate();
  speed_x_cur = imu_ptr->GetSpeedX();
  speed_y_cur = imu_ptr->GetSpeedY();

  if(distance_sensor_x_ptr->receive_success_)
  {
    pos_x_cur = distance_sensor_x_ptr->GetDistance();
  }
  else
  {
    pos_x_cur += speed_x_cur * kControlPeriod;
  }
  
  if(distance_sensor_y_ptr->receive_success_)
  {
    pos_y_cur = distance_sensor_y_ptr->GetDistance();
  }
  else
  {
    pos_y_cur += speed_y_cur * kControlPeriod;
  }
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
  pitch_ref = 0;
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
  float pich_cacu = pich_pid_ptr->cacu(pitch_ref, pitch_cur, kControlPeriod);
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
  if(auto_state == AutoState::Search)
  {
    SetSearchRef();
  }
  else if(auto_state == AutoState::Grab)
  {
    SetGrabRef();
  }
  else if(auto_state == AutoState::Return)
  {
    SetReturnRef();
  }
  

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
  float pich_cacu = pich_pid_ptr->cacu(pitch_ref, pitch_cur, kControlPeriod);
  motor_set[1] -= pich_cacu;
  motor_set[4] += pich_cacu;

  // if(distance_sensor_x_ptr->receive_success_)  //传感器x成功接收
  // {
    //speed_x
    float speed_x_cacu = x_pid_ptr->cacu(pos_x_ref, pos_x_cur, kControlPeriod);
    motor_set[0] += speed_x_cacu;
    motor_set[2] -= speed_x_cacu;
    motor_set[3] += speed_x_cacu;
    motor_set[5] -= speed_x_cacu;
  // }
  
  // if(distance_sensor_y_ptr->receive_success_)  //传感器y成功接收
  // {
    //speed_y
    float speed_y_cacu = y_pid_ptr->cacu(pos_y_ref, pos_y_cur, kControlPeriod);
    motor_set[0] += speed_y_cacu;
    motor_set[2] += speed_y_cacu;
    motor_set[3] += speed_y_cacu;
    motor_set[5] += speed_y_cacu;
  // }
  

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
  depth_ref = kDepthNormal;
  //yaw_ref = last_yaw;
  yaw_ref = 0;
  //pich_ref = 0;

  if(fabs(path[path_index].x - pos_x_cur) < kPointTH && fabs(path[path_index].y - pos_y_cur) < kPointTH)
  {
    if(path[path_index].need_grasp == false)  //该点不需要抓取
    {
      //到下一个点
      path_index++;
      if(path_index > 3) path_index = 0;
    }
    else
    {
      //进入抓取
      auto_state = AutoState::Grab;
    }
    
  }
  pos_x_ref = path[path_index].x;
  pos_y_ref = path[path_index].y;

  pos_x_ref = LimDiff(pos_x_ref, pos_x_cur, 200);
  pos_y_ref = LimDiff(pos_y_ref, pos_y_cur, 200);

  // pos_x_ref = 120;
  // pos_y_ref = 80;
  // pos_x_ref = LimDiff(pos_x_ref, pos_x_cur, 200);
  // pos_y_ref = LimDiff(pos_y_ref, pos_y_cur, 200);
}

void SetGrabRef()
{
  if(grab_state == GrabState::Down)
  {
      //depth_ref = LimDiff(kDepthGrab, depth_cur, 40);
      depth_ref = kDepthGrab;
      //yaw_ref = last_yaw;
      yaw_ref = 0;
      //保持目标位置不变
      pos_x_ref = path[path_index].x;
      pos_y_ref = path[path_index].y;
      pos_x_ref = LimDiff(pos_x_ref, pos_x_cur, 200);
      pos_y_ref = LimDiff(pos_y_ref, pos_y_cur, 200);
      EMOn();  //开启电磁铁

      if(fabs(depth_cur-kDepthGrab) < 3)
      {
        grab_tick++;
      }
      else
      {
        grab_tick /= 2;
      }

      if(grab_tick > 500)
      {
        grab_tick = 0;
        grab_state = GrabState::Up;
      }
  }

  else if(grab_state == GrabState::Up)
  {
      depth_ref = kDepthNormal;
      //yaw_ref = last_yaw;
      yaw_ref = 0;
      //保持目标位置不变
      pos_x_ref = path[path_index].x;
      pos_y_ref = path[path_index].y;
      pos_x_ref = LimDiff(pos_x_ref, pos_x_cur, 200);
      pos_y_ref = LimDiff(pos_y_ref, pos_y_cur, 200);

      if(fabs(depth_cur-kDepthNormal) < 3)
      {
        grab_state = GrabState::Down;
        auto_state = AutoState::Return;
      }
  }

}

void SetReturnRef()
{
  if(return_state == ReturnState::Back)
  {
      depth_ref = kDepthNormal;
      //yaw_ref = last_yaw;
      yaw_ref = 0;
      pos_x_ref = basket_point.x;
      pos_y_ref = basket_point.y;
      pos_x_ref = LimDiff(pos_x_ref, pos_x_cur, 200);
      pos_y_ref = LimDiff(pos_y_ref, pos_y_cur, 200);

      if(fabs(basket_point.x - pos_x_cur) < kPointTH && fabs(basket_point.y - pos_y_cur) < kPointTH)
      {
        return_tick ++;
      }
      else
      {
        return_tick /= 2;
      }
      if(return_tick > 500)
      {
        return_tick = 0;
        return_state = ReturnState::Drop;
      }
  }

  else if(return_state == ReturnState::Drop)
  {
      depth_ref = kDepthNormal;
      //yaw_ref = last_yaw;
      yaw_ref = 0;
      pos_x_ref = basket_point.x;
      pos_y_ref = basket_point.y;
      pos_x_ref = LimDiff(pos_x_ref, pos_x_cur, 200);
      pos_y_ref = LimDiff(pos_y_ref, pos_y_cur, 200);
      EMOff();  //关闭电磁铁

      drop_tick++;
      if(drop_tick > 300)
      {
        drop_tick = 0;
        return_state = ReturnState::Back;
        auto_state = AutoState::Search;
      }
  }
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
  basket_point.x = 20;
  basket_point.y = 20;
  path.push_back({70.0, 70.0, false});
  path.push_back({70.0, 120.0, false});
  path.push_back({120.0, 120.0, false});
  path.push_back({120.0, 70.0, false});
}

void EMOn()
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);  //继电器2 电磁铁开
}

void EMOff()
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);  //继电器2 电磁铁开
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

//妈的，定时狗
void RunOnShit()
{
  // static uint32_t cmp_index = 0;
  // if(main_tick > shit[cmp_index+1].head_time) cmp_index ++;
  // if(cmp_index > shit_index)
  // {
  //   RunOnDead();
  //   return;
  // }
  

  // speed_x_ref = shit[cmp_index].speedX;
  // speed_y_ref = shit[cmp_index].speedY;
  // depth_ref = shit[cmp_index].depth;
  // if(shit[cmp_index].isEMOn == 1) EMOn();
  // else EMOff();

  depth_ref = kDepthNormal;

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
  float pich_cacu = pich_pid_ptr->cacu(pitch_ref, pitch_cur, kControlPeriod);
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

//日你妈的定时狗
void ShitInit()
{
  SetShit(0, 7, kDepthShit, true, 1000);
  SetShit(0, -7, kDepthNormal, true, 1000);
  SetShit(0, 0, kDepthNormal, false, 500);
}

void SetShit(float speedX, float speedY, float depth, bool isEMOn, uint64_t time)
{
  shit[shit_index].head_time = head_time;
  shit[shit_index].speedX = speedX;
  shit[shit_index].speedY = speedY;
  shit[shit_index].depth = depth;
  shit[shit_index].isEMOn = isEMOn;
  shit_index ++;
  head_time += time;
}