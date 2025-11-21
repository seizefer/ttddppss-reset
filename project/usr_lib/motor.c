/**
 * @file motor.c
 * @brief DRV8837电机驱动模块实现
 */

#include "motor.h"
#include "tim.h"

Motor_State_t Motor_State = {0, 0, 0};

/**
 * @brief 初始化电机模块
 */
void Motor_Init(void)
{
    // 启动PWM输出
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    Motor_Stop();
    // 使能电机驱动芯片
    Motor_Enable();
}

/**
 * @brief 设置单个电机速度
 * @param motor 电机ID
 * @param speed 速度值 (-10000 ~ 10000)，正值前进，负值后退
 */
void Motor_SetSpeed(Motor_ID motor, int16_t speed)
{
    if (!Motor_State.enabled) return;

    // 限幅
    if (speed > MOTOR_PWM_MAX) speed = MOTOR_PWM_MAX;
    if (speed < -MOTOR_PWM_MAX) speed = -MOTOR_PWM_MAX;

    uint16_t pwm_forward = 0, pwm_backward = 0;

    if (speed > 0) {
        pwm_forward = (uint16_t)speed;
        pwm_backward = 0;
    } else if (speed < 0) {
        pwm_forward = 0;
        pwm_backward = (uint16_t)(-speed);
    } else {
        pwm_forward = 0;
        pwm_backward = 0;
    }

    if (motor == MOTOR_LEFT) {
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, pwm_forward);
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, pwm_backward);
        Motor_State.speed_left = speed;
    } else {
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, pwm_forward);
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, pwm_backward);
        Motor_State.speed_right = speed;
    }
}

/**
 * @brief 同时设置两个电机速度
 */
void Motor_SetBothSpeed(int16_t left_speed, int16_t right_speed)
{
    Motor_SetSpeed(MOTOR_LEFT, left_speed);
    Motor_SetSpeed(MOTOR_RIGHT, right_speed);
}

/**
 * @brief 停止所有电机
 */
void Motor_Stop(void)
{
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 0);
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, 0);
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, 0);
    Motor_State.speed_left = 0;
    Motor_State.speed_right = 0;
}

/**
 * @brief 使能电机
 */
void Motor_Enable(void)
{
    // 拉高nSLEEP引脚使能电机驱动芯片
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
    Motor_State.enabled = 1;
}

/**
 * @brief 禁用电机
 */
void Motor_Disable(void)
{
    Motor_Stop();
    Motor_State.enabled = 0;
    // 拉低nSLEEP引脚禁用电机驱动芯片（进入睡眠模式）
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
}

/**
 * @brief 直行
 * @param speed 速度值
 */
void Motor_GoStraight(int16_t speed)
{
    Motor_SetBothSpeed(speed, speed);
}

/**
 * @brief 转向行驶
 * @param speed 基础速度
 * @param turn_rate 转向率 (-100~100)，负值左转，正值右转
 */
void Motor_Turn(int16_t speed, int16_t turn_rate)
{
    if (turn_rate > 100) turn_rate = 100;
    if (turn_rate < -100) turn_rate = -100;

    int16_t left_speed = speed;
    int16_t right_speed = speed;

    if (turn_rate > 0) {
        // 右转：右轮减速
        right_speed = speed * (100 - turn_rate) / 100;
    } else if (turn_rate < 0) {
        // 左转：左轮减速
        left_speed = speed * (100 + turn_rate) / 100;
    }

    Motor_SetBothSpeed(left_speed, right_speed);
}

/**
 * @brief 左转
 */
void Motor_TurnLeft(int16_t speed, int16_t rate)
{
    Motor_Turn(speed, -rate);
}

/**
 * @brief 右转
 */
void Motor_TurnRight(int16_t speed, int16_t rate)
{
    Motor_Turn(speed, rate);
}

/**
 * @brief 原地旋转
 * @param speed 正值顺时针，负值逆时针
 */
void Motor_Spin(int16_t speed)
{
    Motor_SetBothSpeed(speed, -speed);
}

/**
 * @brief 直行指定时间
 */
void Motor_GoStraight_ms(int16_t speed, uint32_t ms)
{
    Motor_GoStraight(speed);
    HAL_Delay(ms);
}

/**
 * @brief 停止指定时间
 */
void Motor_Stop_ms(uint32_t ms)
{
    Motor_Stop();
    HAL_Delay(ms);
}
