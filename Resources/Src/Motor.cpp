#include "Motor.hpp"

/**
 * @brief       初始化
 * @param       *htim_ptr: 电机定时器
 * @param       channel: 电机定时器通道
 * @param       dead_zone_f: 前向死区
 * @param       dead_zone_b: 后向死区
 * @param       max_speed: 最高速度
 * @param       dest: 方向 0反向；1正向
 * @arg         None
 * @retval      None
 * @note        None
 */
void Motor::Init(TIM_HandleTypeDef *htim_ptr, int channel, int dead_zone_f, int dead_zone_b, float max_speed, bool dest)
{
    htim_ptr_ = htim_ptr;
    channel_ = channel;
    speed_ = 0.0f;
    max_speed_ = max_speed;
    dead_zone_f_ = dead_zone_f;
    dead_zone_b_ = dead_zone_b;
    dest_ = dest;
    HAL_TIM_PWM_Start(htim_ptr_, channel_);
    //配置输出波形为1.5ms高电平
    __HAL_TIM_SetCompare(htim_ptr_, channel_, 1500);
    HAL_Delay(10);
}

int Motor::map_float_to_int(float x, float min_in, float max_in)
{
    // 1. 检查输入范围是否有效
    if (max_in <= min_in) {
        return 1;  // 或者返回一个错误码
    }

    // 2. 计算比例并映射到 [-500, 500]
    float normalized = (x - min_in) / (max_in - min_in);  // 归一化到 [0, 1]
    float scaled = normalized * 1000.0f - 500.0f;       // 映射到 [-500, 500]

    // 3. 四舍五入并限制范围（防止浮点误差越界）
    int result = (int)roundf(scaled);
    if (result < -500) result = -500;
    if (result > 500) result = 500;

    return result;
}

void Motor::SetInput(float speed)
{	
    speed_ = speed;
	//计算电机转速实际赋值
	if(speed_ > 0)							//若理想转速大于零
    {
        if(speed_ > max_speed_)					//判断是否超过限速
        {
            speed_ = max_speed_;
        }
        speed_set_ = map_float_to_int(speed_, -max_speed_, max_speed_) + dead_zone_f_;		//正向死区补偿
        if(dest_ == 0) speed_set_ = -speed_set_;
        speed_set_ += 1500;
    }
	else										//若理想转速小于等于零
    {	
        if(speed_ < (-max_speed_))				//判断是否超过限速
        {
            speed_ = -max_speed_;				//限定为反向最高转速
        }
        speed_set_ = map_float_to_int(speed_, -max_speed_, max_speed_) - dead_zone_b_;		//反向死区补偿
        if(dest_ == 0) speed_set_ = -speed_set_;
        speed_set_ += 1500;
    }

    __HAL_TIM_SetCompare(htim_ptr_, channel_, speed_set_);
}

void Motor::Stop()
{
    //配置输出波形为1.5ms高电平
    __HAL_TIM_SetCompare(htim_ptr_, channel_, 1500);
}