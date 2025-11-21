/**
 * @file lane.c
 * @brief 车道检测与跟踪模块实现
 */

#include "lane.h"
#include "pid.h"
#include "motor.h"
#include "openmv_comm.h"

Lane_Track_t Lane_Track;

// 参数配置
#define LANE_LOST_TIMEOUT   500     // 丢失超时(ms)
#define LANE_OFFSET_THRESHOLD 10    // 偏移死区

/**
 * @brief 初始化车道跟踪
 */
void Lane_Init(void)
{
    Lane_Track.state = LANE_STATE_IDLE;
    Lane_Track.offset = 0;
    Lane_Track.angle = 0;
    Lane_Track.turn_rate = 0;
    Lane_Track.base_speed = 3000;   // 默认速度
    Lane_Track.lost_time = 0;
    Lane_Track.stop_detected = 0;
}

/**
 * @brief 更新车道数据
 */
void Lane_Update(int16_t offset, int16_t angle)
{
    Lane_Track.offset = offset;
    Lane_Track.angle = angle;

    if (Lane_Track.state == LANE_STATE_LOST) {
        Lane_Track.state = LANE_STATE_TRACKING;
    }
    Lane_Track.lost_time = HAL_GetTick();
}

/**
 * @brief 处理车道跟踪逻辑
 */
void Lane_Process(void)
{
    if (Lane_Track.state == LANE_STATE_IDLE) {
        return;
    }

    // 检查是否丢失车道
    if (HAL_GetTick() - Lane_Track.lost_time > LANE_LOST_TIMEOUT) {
        Lane_Track.state = LANE_STATE_LOST;
        Motor_Stop();
        return;
    }

    // 检查停止线
    if (Lane_Track.stop_detected) {
        Lane_Track.state = LANE_STATE_STOP;
        Motor_Stop();
        return;
    }

    // 使用PID计算转向率
    PID_SetTarget(&PID_Lane, 0);  // 目标偏移为0
    float pid_output = PID_Calculate(&PID_Lane, (float)Lane_Track.offset);

    // 加入角度补偿
    float angle_compensation = Lane_Track.angle * 0.3f;

    Lane_Track.turn_rate = (int16_t)(pid_output + angle_compensation);

    // 限幅
    if (Lane_Track.turn_rate > 100) Lane_Track.turn_rate = 100;
    if (Lane_Track.turn_rate < -100) Lane_Track.turn_rate = -100;

    // 应用到电机
    Motor_Turn(Lane_Track.base_speed, Lane_Track.turn_rate);
}

/**
 * @brief 设置基础速度
 */
void Lane_SetBaseSpeed(int16_t speed)
{
    Lane_Track.base_speed = speed;
}

/**
 * @brief 获取计算的转向率
 */
int16_t Lane_GetTurnRate(void)
{
    return Lane_Track.turn_rate;
}

/**
 * @brief 获取当前状态
 */
Lane_State Lane_GetState(void)
{
    return Lane_Track.state;
}

/**
 * @brief 开始车道跟踪
 */
void Lane_Start(void)
{
    Lane_Track.state = LANE_STATE_TRACKING;
    Lane_Track.lost_time = HAL_GetTick();
    PID_Reset(&PID_Lane);
}

/**
 * @brief 停止车道跟踪
 */
void Lane_Stop(void)
{
    Lane_Track.state = LANE_STATE_IDLE;
    Motor_Stop();
}
