#include "Motor.hpp"

void Motor::Init(TIM_HandleTypeDef *htim_ptr, int channel, int dead_zone_f, int dead_zone_b, int max_speed)
{
    htim_ptr_ = htim_ptr;
    channel_ = channel;
    speed_ = 0;
    max_speed_ = max_speed;
    dead_zone_f_ = dead_zone_f;
    dead_zone_b_ = dead_zone_b;
    HAL_TIM_PWM_Start(htim_ptr_, channel_);
    //配置输出波形为1.5ms高电平
    __HAL_TIM_SetCompare(htim_ptr_, channel_, 1500);
    HAL_Delay(10);
}

void Motor::SetInput(int speed)
{
    int speed_set=0;	//电机转速实际赋值
	
	//计算电机转速实际赋值
	if(speed > 0)							//若理想转速大于零
    {
        speed_set = 1500 + speed + dead_zone_f_;		//正向死区补偿
        if(speed_set > max_speed_)					//判断是否超过限速
            {
                speed_set = max_speed_;
            }
    }
	else										//若理想转速小于等于零
    {	
        speed_set = 1500 + speed - dead_zone_b_;		//反向死区补偿
        if(speed_set < (1500-max_speed_))				//判断是否超过限速
            {
                speed_set = 1500 - max_speed_;				//限定为反向最高转速
            }
    }

    __HAL_TIM_SetCompare(htim_ptr_, channel_, speed_set);
}

void Motor::Stop()
{
    //配置输出波形为1.5ms高电平
    __HAL_TIM_SetCompare(htim_ptr_, channel_, 1500);
}