/**
 * @file lane.h
 * @brief 车道检测与跟踪模块
 */

#ifndef __LANE_H
#define __LANE_H

#include "main.h"

// 车道跟踪状态
typedef enum {
    LANE_STATE_IDLE = 0,
    LANE_STATE_TRACKING,
    LANE_STATE_LOST,
    LANE_STATE_CURVE,
    LANE_STATE_STOP
} Lane_State;

// 车道数据结构
typedef struct {
    Lane_State state;           // 当前状态
    int16_t offset;             // 车道偏移
    int16_t angle;              // 车道角度
    int16_t turn_rate;          // 计算的转向率
    int16_t base_speed;         // 基础速度
    uint32_t lost_time;         // 丢失时间
    uint8_t stop_detected;      // 停止线检测
} Lane_Track_t;

extern Lane_Track_t Lane_Track;

// 初始化
void Lane_Init(void);

// 更新与处理
void Lane_Update(int16_t offset, int16_t angle);
void Lane_Process(void);

// 控制参数
void Lane_SetBaseSpeed(int16_t speed);
int16_t Lane_GetTurnRate(void);

// 状态
Lane_State Lane_GetState(void);
void Lane_Start(void);
void Lane_Stop(void);

#endif /* __LANE_H */
