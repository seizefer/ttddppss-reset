/**
 * @file motor.h
 * @brief DRV8837电机驱动模块
 */

#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

// 电机速度范围
#define MOTOR_PWM_MIN    0
#define MOTOR_PWM_MAX    10000
#define MOTOR_PWM_STOP   0

// 电机ID
typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT = 1
} Motor_ID;

// 电机方向
typedef enum {
    MOTOR_DIR_FORWARD = 0,
    MOTOR_DIR_BACKWARD = 1,
    MOTOR_DIR_STOP = 2
} Motor_Dir;

// 电机状态结构体
typedef struct {
    int16_t speed_left;     // 左电机速度 (-10000 ~ 10000)
    int16_t speed_right;    // 右电机速度 (-10000 ~ 10000)
    uint8_t enabled;        // 使能状态
} Motor_State_t;

extern Motor_State_t Motor_State;

// 初始化
void Motor_Init(void);

// 基础控制
void Motor_SetSpeed(Motor_ID motor, int16_t speed);
void Motor_SetBothSpeed(int16_t left_speed, int16_t right_speed);
void Motor_Stop(void);
void Motor_Enable(void);
void Motor_Disable(void);

// 运动控制
void Motor_GoStraight(int16_t speed);
void Motor_Turn(int16_t speed, int16_t turn_rate);  // turn_rate: -100~100
void Motor_TurnLeft(int16_t speed, int16_t rate);
void Motor_TurnRight(int16_t speed, int16_t rate);
void Motor_Spin(int16_t speed);  // 原地旋转

// 延时版本
void Motor_GoStraight_ms(int16_t speed, uint32_t ms);
void Motor_Stop_ms(uint32_t ms);

#endif /* __MOTOR_H */
