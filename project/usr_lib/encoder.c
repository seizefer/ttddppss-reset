/**
 * @file encoder.c
 * @brief 编码器模块实现
 */

#include "encoder.h"
#include "tim.h"
#include <math.h>

Encoder_Data_t Encoder_Data;

// 内部变量
static int32_t last_count_left = 0;
static int32_t last_count_right = 0;

// 计算常数
static const float PULSE_TO_MM = (3.14159f * ENCODER_WHEEL_DIAMETER) / (ENCODER_PPR * ENCODER_GEAR_RATIO * 4);

/**
 * @brief 初始化编码器
 */
void Encoder_Init(void)
{
    // 清零数据
    Encoder_Data.count_left = 0;
    Encoder_Data.count_right = 0;
    Encoder_Data.speed_left = 0;
    Encoder_Data.speed_right = 0;
    Encoder_Data.distance_left = 0;
    Encoder_Data.distance_right = 0;
    Encoder_Data.last_update = HAL_GetTick();

    // 启动编码器定时器 (假设使用TIM3和TIM4)
    // HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    // HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    last_count_left = 0;
    last_count_right = 0;
}

/**
 * @brief 更新编码器数据
 * 应在定时中断中周期性调用
 */
void Encoder_Update(void)
{
    uint32_t current_time = HAL_GetTick();
    uint32_t dt = current_time - Encoder_Data.last_update;

    if (dt == 0) return;

    // 读取当前计数 (假设使用TIM3和TIM4)
    // int16_t count_left = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    // int16_t count_right = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);

    // 临时使用0 (需要根据实际硬件配置)
    int16_t count_left = 0;
    int16_t count_right = 0;

    // 计算增量
    int32_t delta_left = count_left - last_count_left;
    int32_t delta_right = count_right - last_count_right;

    // 处理溢出
    if (delta_left > 32767) delta_left -= 65536;
    if (delta_left < -32768) delta_left += 65536;
    if (delta_right > 32767) delta_right -= 65536;
    if (delta_right < -32768) delta_right += 65536;

    // 累积计数
    Encoder_Data.count_left += delta_left;
    Encoder_Data.count_right += delta_right;

    // 计算速度 (脉冲/秒)
    Encoder_Data.speed_left = (float)delta_left * 1000.0f / dt;
    Encoder_Data.speed_right = (float)delta_right * 1000.0f / dt;

    // 计算距离 (mm)
    Encoder_Data.distance_left += delta_left * PULSE_TO_MM;
    Encoder_Data.distance_right += delta_right * PULSE_TO_MM;

    // 更新状态
    last_count_left = count_left;
    last_count_right = count_right;
    Encoder_Data.last_update = current_time;
}

/**
 * @brief 获取编码器计数
 */
int32_t Encoder_GetCount(Encoder_ID encoder)
{
    if (encoder == ENCODER_LEFT) {
        return Encoder_Data.count_left;
    } else {
        return Encoder_Data.count_right;
    }
}

/**
 * @brief 获取编码器速度
 */
float Encoder_GetSpeed(Encoder_ID encoder)
{
    if (encoder == ENCODER_LEFT) {
        return Encoder_Data.speed_left;
    } else {
        return Encoder_Data.speed_right;
    }
}

/**
 * @brief 获取行驶距离
 */
float Encoder_GetDistance(Encoder_ID encoder)
{
    if (encoder == ENCODER_LEFT) {
        return Encoder_Data.distance_left;
    } else {
        return Encoder_Data.distance_right;
    }
}

/**
 * @brief 获取总行驶距离（平均值）
 */
float Encoder_GetTotalDistance(void)
{
    return (Encoder_Data.distance_left + Encoder_Data.distance_right) / 2.0f;
}

/**
 * @brief 重置所有编码器数据
 */
void Encoder_Reset(void)
{
    Encoder_Data.count_left = 0;
    Encoder_Data.count_right = 0;
    Encoder_Data.distance_left = 0;
    Encoder_Data.distance_right = 0;
    last_count_left = 0;
    last_count_right = 0;
}

/**
 * @brief 重置单个编码器计数
 */
void Encoder_ResetCount(Encoder_ID encoder)
{
    if (encoder == ENCODER_LEFT) {
        Encoder_Data.count_left = 0;
        Encoder_Data.distance_left = 0;
    } else {
        Encoder_Data.count_right = 0;
        Encoder_Data.distance_right = 0;
    }
}
