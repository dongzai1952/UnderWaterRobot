#include "PIDController.hpp"

PIDController::PIDController() 
    : _kp(0), _ki(0), _kd(0), 
      _max_output(0), _max_integral(0), _dead_zone(0),
      _integral(0), _prev_error(0), _prev_output(0) {
}

void PIDController::init(float kp, float ki, float kd, 
                        float max_output, float max_integral, 
                        float dead_zone) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _max_output = fabsf(max_output);
    _max_integral = fabsf(max_integral);
    _dead_zone = fabsf(dead_zone);
    
    reset();
}

void PIDController::reset() {
    _integral = 0;
    _prev_error = 0;
    _prev_output = 0;
}

float PIDController::cacu(float setpoint, float feedback, float dt) {
    if (dt <= 0) {
        return _prev_output;
    }
    
    float error = setpoint - feedback;
    
    // 死区处理
    if (fabsf(error) <= _dead_zone) {
        error = 0;
    }
    
    // 比例项
    float proportional = _kp * error;
    
    // 积分项(带限幅)
    _integral += _ki * error * dt;
    
    // 积分限幅
    if (_max_integral > 0) {
        if (_integral > _max_integral) {
            _integral = _max_integral;
        } else if (_integral < -_max_integral) {
            _integral = -_max_integral;
        }
    }
    
    // 微分项
    float derivative = 0;
    if (dt > 0) {
        derivative = _kd * (error - _prev_error) / dt;
    }
    _prev_error = error;
    
    // 计算输出
    float output = proportional + _integral + derivative;
    
    // 输出限幅
    if (output > _max_output) {
        output = _max_output;
    } else if (output < -_max_output) {
        output = -_max_output;
    }
    
    _prev_output = output;
    return output;
}

void PIDController::setPID(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void PIDController::setOutputLimit(float max_output) {
    _max_output = fabsf(max_output);
}

void PIDController::setIntegralLimit(float max_integral) {
    _max_integral = fabsf(max_integral);
}

void PIDController::setDeadZone(float dead_zone) {
    _dead_zone = fabsf(dead_zone);
}