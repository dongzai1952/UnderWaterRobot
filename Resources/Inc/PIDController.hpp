#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP
#include <stdint.h>
#include <math.h>

class PIDController {
public:
    /**
     * @brief PID控制器构造函数
     */
    PIDController();
    
    /**
     * @brief PID控制器初始化
     * @param kp 比例系数
     * @param ki 积分系数
     * @param kd 微分系数
     * @param max_output 最大输出限制
     * @param max_integral 积分限幅值
     * @param dead_zone 死区范围(绝对值)
     */
    void init(float kp, float ki, float kd, float max_output, float max_integral, float dead_zone = 0);
    
    /**
     * @brief 重置PID控制器(清除积分和上次误差)
     */
    void reset();
    
    /**
     * @brief 计算PID输出
     * @param setpoint 设定值
     * @param feedback 反馈值
     * @param dt 时间间隔(秒)
     * @return PID控制输出
     */
    float cacu(float setpoint, float feedback, float dt);
    
    /**
      * @brief 设置PID参数
      * @param kp 比例系数
      * @param ki 积分系数
      * @param kd 微分系数
      */
    void setPID(float kp, float ki, float kd);
    
    /**
     * @brief 设置输出限幅
     * @param max_output 最大输出限制
     */
    void setOutputLimit(float max_output);
    
    /**
     * @brief 设置积分限幅
     * @param max_integral 积分限幅值
     */
    void setIntegralLimit(float max_integral);
    
    /**
     * @brief 设置死区
     * @param dead_zone 死区范围(绝对值)
     */
    void setDeadZone(float dead_zone);
    
private:
    float _kp;              // 比例系数
    float _ki;              // 积分系数
    float _kd;              // 微分系数
    float _max_output;      // 输出限幅
    float _max_integral;    // 积分限幅
    float _dead_zone;       // 死区范围
    
    float _integral;        // 积分项
    float _prev_error;      // 上一次误差
    float _prev_output;     // 上一次输出
};

#endif