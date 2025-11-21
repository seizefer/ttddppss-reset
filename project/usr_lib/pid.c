/**
 * @file pid.c
 * @brief PID控制器模块实现
 */

#include "pid.h"

// 预定义控制器实例
PID_Controller_t PID_Speed_Left;
PID_Controller_t PID_Speed_Right;
PID_Controller_t PID_Steering;
PID_Controller_t PID_Lane;

/**
 * @brief 初始化PID控制器
 */
void PID_Init(PID_Controller_t *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint = 0;
    pid->integral = 0;
    pid->prev_error = 0;
    pid->output_min = -10000;
    pid->output_max = 10000;
    pid->integral_max = 1000;
    pid->output = 0;
}

/**
 * @brief 设置输出限幅
 */
void PID_SetLimits(PID_Controller_t *pid, float out_min, float out_max, float int_max)
{
    pid->output_min = out_min;
    pid->output_max = out_max;
    pid->integral_max = int_max;
}

/**
 * @brief 设置目标值
 */
void PID_SetTarget(PID_Controller_t *pid, float setpoint)
{
    pid->setpoint = setpoint;
}

/**
 * @brief 位置式PID计算
 * @param pid PID控制器
 * @param measurement 当前测量值
 * @return PID输出
 */
float PID_Calculate(PID_Controller_t *pid, float measurement)
{
    float error = pid->setpoint - measurement;

    // 积分累积
    pid->integral += error;

    // 积分限幅
    if (pid->integral > pid->integral_max) {
        pid->integral = pid->integral_max;
    } else if (pid->integral < -pid->integral_max) {
        pid->integral = -pid->integral_max;
    }

    // 微分
    float derivative = error - pid->prev_error;
    pid->prev_error = error;

    // 计算输出
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

    // 输出限幅
    if (output > pid->output_max) {
        output = pid->output_max;
    } else if (output < pid->output_min) {
        output = pid->output_min;
    }

    pid->output = output;
    return output;
}

/**
 * @brief 增量式PID计算
 */
float PID_Calculate_Incremental(PID_Controller_t *pid, float measurement)
{
    float error = pid->setpoint - measurement;

    // 增量计算
    float delta = pid->Kp * (error - pid->prev_error) +
                  pid->Ki * error +
                  pid->Kd * (error - 2 * pid->prev_error + pid->integral);

    pid->integral = pid->prev_error;  // 存储上上次误差
    pid->prev_error = error;

    pid->output += delta;

    // 输出限幅
    if (pid->output > pid->output_max) {
        pid->output = pid->output_max;
    } else if (pid->output < pid->output_min) {
        pid->output = pid->output_min;
    }

    return pid->output;
}

/**
 * @brief 重置PID控制器
 */
void PID_Reset(PID_Controller_t *pid)
{
    pid->integral = 0;
    pid->prev_error = 0;
    pid->output = 0;
}

/**
 * @brief 初始化所有预定义控制器
 */
void PID_InitAll(void)
{
    // 速度PID - 需要根据实际调参
    PID_Init(&PID_Speed_Left, 1.0f, 0.1f, 0.05f);
    PID_SetLimits(&PID_Speed_Left, -10000, 10000, 5000);

    PID_Init(&PID_Speed_Right, 1.0f, 0.1f, 0.05f);
    PID_SetLimits(&PID_Speed_Right, -10000, 10000, 5000);

    // 转向PID
    PID_Init(&PID_Steering, 2.0f, 0.01f, 0.5f);
    PID_SetLimits(&PID_Steering, -100, 100, 500);

    // 车道跟踪PID
    PID_Init(&PID_Lane, 0.5f, 0.01f, 0.1f);
    PID_SetLimits(&PID_Lane, -100, 100, 200);
}
