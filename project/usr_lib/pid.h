/**
 * @file pid.h
 * @brief PID控制器模块
 */

#ifndef __PID_H
#define __PID_H

#include "main.h"

// PID控制器结构体
typedef struct {
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数

    float setpoint;     // 目标值
    float integral;     // 积分累积
    float prev_error;   // 上次误差

    float output_min;   // 输出下限
    float output_max;   // 输出上限
    float integral_max; // 积分限幅

    float output;       // 当前输出
} PID_Controller_t;

// 预定义的PID控制器
extern PID_Controller_t PID_Speed_Left;
extern PID_Controller_t PID_Speed_Right;
extern PID_Controller_t PID_Steering;
extern PID_Controller_t PID_Lane;

// 初始化
void PID_Init(PID_Controller_t *pid, float Kp, float Ki, float Kd);
void PID_SetLimits(PID_Controller_t *pid, float out_min, float out_max, float int_max);
void PID_SetTarget(PID_Controller_t *pid, float setpoint);

// 计算
float PID_Calculate(PID_Controller_t *pid, float measurement);
float PID_Calculate_Incremental(PID_Controller_t *pid, float measurement);

// 重置
void PID_Reset(PID_Controller_t *pid);

// 初始化所有预定义控制器
void PID_InitAll(void);

#endif /* __PID_H */
