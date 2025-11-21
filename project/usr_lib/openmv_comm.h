/**
 * @file openmv_comm.h
 * @brief OpenMV摄像头通信模块
 */

#ifndef __OPENMV_COMM_H
#define __OPENMV_COMM_H

#include "main.h"

// 帧格式定义
#define OPENMV_FRAME_HEADER     0xAA
#define OPENMV_FRAME_TAIL       0x55
#define OPENMV_MAX_DATA_LEN     16

// 命令码定义
typedef enum {
    OPENMV_CMD_LANE = 0x01,      // 车道检测数据
    OPENMV_CMD_COLOR = 0x02,     // 颜色检测
    OPENMV_CMD_ARROW = 0x03,     // 箭头识别
    OPENMV_CMD_LINE = 0x04,      // 停止线检测
    OPENMV_CMD_TARGET = 0x05,    // 目标位置
    OPENMV_CMD_BASKET = 0x06,    // 篮筐检测
} OpenMV_CmdType;

// 箭头方向
typedef enum {
    ARROW_NONE = 0,
    ARROW_LEFT = 1,
    ARROW_STRAIGHT = 2,
    ARROW_RIGHT = 3
} Arrow_Direction;

// 检测颜色
typedef enum {
    COLOR_NONE = 0,
    COLOR_RED = 1,
    COLOR_GREEN = 2,
    COLOR_BLUE = 3,
    COLOR_YELLOW = 4
} Detect_Color;

// OpenMV数据结构
typedef struct {
    // 车道数据
    int16_t lane_offset;        // 车道偏移 (-160 ~ 160)
    int16_t lane_angle;         // 车道角度
    uint8_t lane_valid;         // 车道有效标志

    // 箭头识别
    Arrow_Direction arrow;      // 箭头方向
    uint8_t arrow_valid;

    // 停止线
    uint8_t stop_line_detected; // 停止线检测
    uint16_t stop_line_distance;// 到停止线距离

    // 颜色检测
    Detect_Color color;         // 检测到的颜色
    uint8_t color_valid;

    // 目标检测
    int16_t target_x;           // 目标X坐标
    int16_t target_y;           // 目标Y坐标
    uint8_t target_valid;

    // 通信状态
    uint8_t connected;          // 连接状态
    uint32_t last_update;       // 最后更新时间
    uint32_t error_count;       // 错误计数
} OpenMV_Data_t;

extern OpenMV_Data_t OpenMV_Data;

// 接收缓冲区
#define OPENMV_RX_BUF_SIZE  32
extern uint8_t OpenMV_RxBuffer[OPENMV_RX_BUF_SIZE];
extern volatile uint8_t OpenMV_RxIndex;
extern volatile uint8_t OpenMV_FrameReady;

// 初始化
void OpenMV_Init(void);

// 数据处理
void OpenMV_ProcessByte(uint8_t byte);
void OpenMV_ParseFrame(void);
uint8_t OpenMV_IsConnected(void);

// 发送命令
void OpenMV_SendCommand(uint8_t cmd, uint8_t *data, uint8_t len);

// 模式切换命令
#define OPENMV_MODE_LANE_FOLLOW     0x10
#define OPENMV_MODE_ARROW_DETECT    0x11
#define OPENMV_MODE_COLOR_DETECT    0x12
#define OPENMV_MODE_TARGET_FIND     0x13

void OpenMV_SetMode(uint8_t mode);

// 获取数据
int16_t OpenMV_GetLaneOffset(void);
Arrow_Direction OpenMV_GetArrow(void);
uint8_t OpenMV_IsStopLine(void);
Detect_Color OpenMV_GetColor(void);

#endif /* __OPENMV_COMM_H */
