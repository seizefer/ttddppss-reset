/**
 * @file openmv_comm.c
 * @brief OpenMV摄像头通信模块实现
 */

#include "openmv_comm.h"
#include "usart.h"
#include <string.h>

OpenMV_Data_t OpenMV_Data;

uint8_t OpenMV_RxBuffer[OPENMV_RX_BUF_SIZE];
volatile uint8_t OpenMV_RxIndex = 0;
volatile uint8_t OpenMV_FrameReady = 0;

// 接收状态机
static enum {
    RX_STATE_IDLE,
    RX_STATE_HEADER,
    RX_STATE_CMD,
    RX_STATE_LEN,
    RX_STATE_DATA,
    RX_STATE_CHECKSUM
} rx_state = RX_STATE_IDLE;

static uint8_t rx_cmd = 0;
static uint8_t rx_len = 0;
static uint8_t rx_data_count = 0;
static uint8_t rx_checksum = 0;

/**
 * @brief 初始化OpenMV通信
 */
void OpenMV_Init(void)
{
    memset(&OpenMV_Data, 0, sizeof(OpenMV_Data));
    memset(OpenMV_RxBuffer, 0, sizeof(OpenMV_RxBuffer));
    OpenMV_RxIndex = 0;
    OpenMV_FrameReady = 0;
    rx_state = RX_STATE_IDLE;

    // 启用UART2接收中断
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
}

/**
 * @brief 处理接收到的字节
 * @param byte 接收到的字节
 */
void OpenMV_ProcessByte(uint8_t byte)
{
    switch (rx_state) {
        case RX_STATE_IDLE:
            if (byte == OPENMV_FRAME_HEADER) {
                rx_state = RX_STATE_HEADER;
                rx_checksum = byte;
            }
            break;

        case RX_STATE_HEADER:
            rx_cmd = byte;
            rx_checksum += byte;
            rx_state = RX_STATE_CMD;
            break;

        case RX_STATE_CMD:
            rx_len = byte;
            rx_checksum += byte;
            if (rx_len > OPENMV_MAX_DATA_LEN) {
                rx_state = RX_STATE_IDLE;
                OpenMV_Data.error_count++;
            } else if (rx_len == 0) {
                rx_state = RX_STATE_CHECKSUM;
            } else {
                rx_data_count = 0;
                rx_state = RX_STATE_DATA;
            }
            break;

        case RX_STATE_DATA:
            OpenMV_RxBuffer[rx_data_count++] = byte;
            rx_checksum += byte;
            if (rx_data_count >= rx_len) {
                rx_state = RX_STATE_CHECKSUM;
            }
            break;

        case RX_STATE_CHECKSUM:
            if (byte == (rx_checksum & 0xFF)) {
                OpenMV_RxIndex = rx_len;
                OpenMV_FrameReady = 1;
                // 解析数据
                OpenMV_ParseFrame();
            } else {
                OpenMV_Data.error_count++;
            }
            rx_state = RX_STATE_IDLE;
            break;

        default:
            rx_state = RX_STATE_IDLE;
            break;
    }
}

/**
 * @brief 解析接收到的帧
 */
void OpenMV_ParseFrame(void)
{
    if (!OpenMV_FrameReady) return;

    OpenMV_Data.last_update = HAL_GetTick();
    OpenMV_Data.connected = 1;

    switch (rx_cmd) {
        case OPENMV_CMD_LANE:
            if (OpenMV_RxIndex >= 4) {
                OpenMV_Data.lane_offset = (int16_t)((OpenMV_RxBuffer[1] << 8) | OpenMV_RxBuffer[0]);
                OpenMV_Data.lane_angle = (int16_t)((OpenMV_RxBuffer[3] << 8) | OpenMV_RxBuffer[2]);
                OpenMV_Data.lane_valid = 1;
            }
            break;

        case OPENMV_CMD_COLOR:
            if (OpenMV_RxIndex >= 1) {
                OpenMV_Data.color = (Detect_Color)OpenMV_RxBuffer[0];
                OpenMV_Data.color_valid = 1;
            }
            break;

        case OPENMV_CMD_ARROW:
            if (OpenMV_RxIndex >= 1) {
                OpenMV_Data.arrow = (Arrow_Direction)OpenMV_RxBuffer[0];
                OpenMV_Data.arrow_valid = 1;
            }
            break;

        case OPENMV_CMD_LINE:
            if (OpenMV_RxIndex >= 3) {
                OpenMV_Data.stop_line_detected = OpenMV_RxBuffer[0];
                OpenMV_Data.stop_line_distance = (OpenMV_RxBuffer[2] << 8) | OpenMV_RxBuffer[1];
            }
            break;

        case OPENMV_CMD_TARGET:
            if (OpenMV_RxIndex >= 4) {
                OpenMV_Data.target_x = (int16_t)((OpenMV_RxBuffer[1] << 8) | OpenMV_RxBuffer[0]);
                OpenMV_Data.target_y = (int16_t)((OpenMV_RxBuffer[3] << 8) | OpenMV_RxBuffer[2]);
                OpenMV_Data.target_valid = 1;
            }
            break;

        case OPENMV_CMD_BASKET:
            // 篮筐检测数据处理
            break;

        default:
            break;
    }

    OpenMV_FrameReady = 0;
}

/**
 * @brief 检查OpenMV连接状态
 */
uint8_t OpenMV_IsConnected(void)
{
    // 超过500ms未收到数据则认为断开
    if (HAL_GetTick() - OpenMV_Data.last_update > 500) {
        OpenMV_Data.connected = 0;
    }
    return OpenMV_Data.connected;
}

/**
 * @brief 发送命令到OpenMV
 */
void OpenMV_SendCommand(uint8_t cmd, uint8_t *data, uint8_t len)
{
    uint8_t frame[OPENMV_MAX_DATA_LEN + 4];
    uint8_t checksum = 0;
    uint8_t idx = 0;

    frame[idx++] = OPENMV_FRAME_HEADER;
    checksum += OPENMV_FRAME_HEADER;

    frame[idx++] = cmd;
    checksum += cmd;

    frame[idx++] = len;
    checksum += len;

    for (uint8_t i = 0; i < len; i++) {
        frame[idx++] = data[i];
        checksum += data[i];
    }

    frame[idx++] = checksum;

    HAL_UART_Transmit(&huart2, frame, idx, 100);
}

/**
 * @brief 获取车道偏移
 */
int16_t OpenMV_GetLaneOffset(void)
{
    if (OpenMV_Data.lane_valid) {
        return OpenMV_Data.lane_offset;
    }
    return 0;
}

/**
 * @brief 获取箭头方向
 */
Arrow_Direction OpenMV_GetArrow(void)
{
    if (OpenMV_Data.arrow_valid) {
        return OpenMV_Data.arrow;
    }
    return ARROW_NONE;
}

/**
 * @brief 检查是否检测到停止线
 */
uint8_t OpenMV_IsStopLine(void)
{
    return OpenMV_Data.stop_line_detected;
}
