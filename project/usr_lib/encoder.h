/**
 * @file encoder.h
 * @brief 编码器模块
 */

#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

// 编码器ID
typedef enum {
    ENCODER_LEFT = 0,
    ENCODER_RIGHT = 1
} Encoder_ID;

// 编码器数据结构
typedef struct {
    int32_t count_left;         // 左轮脉冲计数
    int32_t count_right;        // 右轮脉冲计数
    float speed_left;           // 左轮速度 (脉冲/秒)
    float speed_right;          // 右轮速度 (脉冲/秒)
    float distance_left;        // 左轮行驶距离 (mm)
    float distance_right;       // 右轮行驶距离 (mm)
    uint32_t last_update;       // 上次更新时间
} Encoder_Data_t;

extern Encoder_Data_t Encoder_Data;

// 编码器参数
#define ENCODER_PPR             11      // 每转脉冲数
#define ENCODER_GEAR_RATIO      30      // 减速比
#define ENCODER_WHEEL_DIAMETER  65.0f   // 轮子直径 (mm)

// 初始化
void Encoder_Init(void);

// 更新
void Encoder_Update(void);

// 数据获取
int32_t Encoder_GetCount(Encoder_ID encoder);
float Encoder_GetSpeed(Encoder_ID encoder);
float Encoder_GetDistance(Encoder_ID encoder);
float Encoder_GetTotalDistance(void);

// 重置
void Encoder_Reset(void);
void Encoder_ResetCount(Encoder_ID encoder);

#endif /* __ENCODER_H */
