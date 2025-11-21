/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : 自主移动机器人主程序
 * @author         : Rover Team
 * @version        : 1.0
 ******************************************************************************
 * @attention
 *
 * 功能说明：
 * - Patio 1: 路径跟随、过桥、过门
 * - Patio 2: 形状匹配、投球、无线通信
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "pid.h"
#include "openmv_comm.h"
#include "lane.h"
#include "encoder.h"
#include "SR_04.h"
#include "filter.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// 任务状态机
typedef enum {
    TASK_STATE_IDLE = 0,
    // Patio 1 任务
    TASK_P1_LANE_FOLLOW,    // 任务1: 路径跟随
    TASK_P1_BRIDGE,         // 任务2: 过桥
    TASK_P1_GATE,           // 任务3: 过门
    // Patio 2 任务
    TASK_P2_SHAPE_MATCH,    // 任务4: 形状匹配
    TASK_P2_BALL_RELEASE,   // 任务5: 投球
    TASK_P2_COMMUNICATION,  // 任务6: 无线通信
    TASK_STATE_DONE
} Task_State_t;

// 系统运行模式
typedef enum {
    MODE_PATIO_1 = 0,
    MODE_PATIO_2 = 1
} Run_Mode_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// 系统参数
#define SYSTEM_TICK_MS          10      // 系统周期 (ms)
#define OBSTACLE_DISTANCE       0.15f   // 障碍物距离阈值 (m)

// 舵机角度
#define SERVO_CENTER            90
#define SERVO_LEFT              45
#define SERVO_RIGHT             135

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

// 系统状态
static Task_State_t g_task_state = TASK_STATE_IDLE;
static Run_Mode_t g_run_mode = MODE_PATIO_1;
static uint32_t g_system_tick = 0;

// 舵机角度
static float g_servo_angle = SERVO_CENTER;

// OpenMV接收
static uint8_t g_openmv_rx_byte;

// HC-12通信标志
static uint8_t g_hc12_open_flag = 0x02;
static uint8_t g_hc12_close_flag = 0x01;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

// 系统函数
static void System_Init(void);
static void System_Process(void);

// 任务处理
static void Task_Process(void);
static void Task_Patio1_LaneFollow(void);
static void Task_Patio1_Bridge(void);
static void Task_Patio1_Gate(void);
static void Task_Patio2_ShapeMatch(void);
static void Task_Patio2_BallRelease(void);
static void Task_Patio2_Communication(void);

// 舵机控制
static void Servo_SetAngle(float angle);

// 调试输出
static void Debug_Print(const char *msg);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// printf重定向
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (unsigned char *)&ch, 1, 0xFFFF);
    while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) != SET);
    return ch;
}

/**
 * @brief 定时器中断回调
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == htim12.Instance)
    {
        // 超声波触发
        SR_04_Start(GPIOB, GPIO_PIN_12);
    }
}

/**
 * @brief UART接收中断回调
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        // OpenMV数据处理
        OpenMV_ProcessByte(g_openmv_rx_byte);
        // 重新启动接收
        HAL_UART_Receive_IT(&huart2, &g_openmv_rx_byte, 1);
    }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_TIM2_Init();
    MX_USART2_UART_Init();
    MX_TIM1_Init();
    MX_TIM12_Init();
    MX_USART3_UART_Init();
    MX_TIM5_Init();

    /* USER CODE BEGIN 2 */

    // 系统初始化
    System_Init();

    // 启动超声波定时器
    HAL_TIM_Base_Start_IT(&htim12);

    // 启动OpenMV接收
    HAL_UART_Receive_IT(&huart2, &g_openmv_rx_byte, 1);

    Debug_Print("System Ready\r\n");

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        // 主循环处理
        System_Process();

        // 任务处理
        Task_Process();

        // 系统延时
        HAL_Delay(SYSTEM_TICK_MS);
        g_system_tick++;

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief 系统初始化
 */
static void System_Init(void)
{
    // 初始化各模块
    Motor_Init();
    PID_InitAll();
    OpenMV_Init();
    Lane_Init();
    Encoder_Init();

    // 初始化舵机到中位
    Servo_SetAngle(SERVO_CENTER);

    // 默认停止
    Motor_Stop();
}

/**
 * @brief 系统周期处理
 */
static void System_Process(void)
{
    // 更新编码器
    Encoder_Update();

    // 更新车道跟踪
    if (OpenMV_Data.lane_valid) {
        Lane_Update(OpenMV_Data.lane_offset, OpenMV_Data.lane_angle);
        OpenMV_Data.lane_valid = 0;  // 清除标志
    }

    // 安全检查：障碍物紧急停止
    if (SR_04.distance_filtered < OBSTACLE_DISTANCE &&
        SR_04.distance_filtered > 0.01f) {
        if (g_task_state != TASK_STATE_IDLE && g_task_state != TASK_STATE_DONE) {
            Motor_Stop();
        }
    }
}

/**
 * @brief 任务处理状态机
 */
static void Task_Process(void)
{
    switch (g_task_state)
    {
        case TASK_STATE_IDLE:
            // 等待启动信号
            break;

        // Patio 1 任务
        case TASK_P1_LANE_FOLLOW:
            Task_Patio1_LaneFollow();
            break;

        case TASK_P1_BRIDGE:
            Task_Patio1_Bridge();
            break;

        case TASK_P1_GATE:
            Task_Patio1_Gate();
            break;

        // Patio 2 任务
        case TASK_P2_SHAPE_MATCH:
            Task_Patio2_ShapeMatch();
            break;

        case TASK_P2_BALL_RELEASE:
            Task_Patio2_BallRelease();
            break;

        case TASK_P2_COMMUNICATION:
            Task_Patio2_Communication();
            break;

        case TASK_STATE_DONE:
            Motor_Stop();
            break;

        default:
            g_task_state = TASK_STATE_IDLE;
            break;
    }
}

/**
 * @brief 任务1：路径跟随
 */
static void Task_Patio1_LaneFollow(void)
{
    static uint8_t lane_phase = 0;
    static uint32_t phase_timer = 0;

    switch (lane_phase) {
        case 0: // 初始化
            OpenMV_SetMode(OPENMV_MODE_LANE_FOLLOW);
            Lane_SetBaseSpeed(3000);
            Lane_Start();
            phase_timer = HAL_GetTick();
            lane_phase = 1;
            Debug_Print("Task1: Lane Follow Start\r\n");
            break;

        case 1: // 车道跟踪
            Lane_Process();

            // 检查是否到达桥梁（停止线）
            if (OpenMV_IsStopLine() && OpenMV_Data.stop_line_distance < 50) {
                Lane_Stop();
                lane_phase = 2;
            }

            // 超时保护 (30秒)
            if (HAL_GetTick() - phase_timer > 30000) {
                lane_phase = 2;
            }
            break;

        case 2: // 完成，切换到下一任务
            lane_phase = 0;
            g_task_state = TASK_P1_BRIDGE;
            Debug_Print("Task1: Complete\r\n");
            break;
    }
}

/**
 * @brief 任务2：过桥
 */
static void Task_Patio1_Bridge(void)
{
    static uint8_t bridge_phase = 0;
    static uint32_t phase_timer = 0;

    switch (bridge_phase) {
        case 0: // 初始化
            phase_timer = HAL_GetTick();
            bridge_phase = 1;
            Debug_Print("Task2: Bridge Start\r\n");
            break;

        case 1: // 上坡 (加速爬坡)
            Motor_GoStraight(5000);
            // 检测倾斜角度或编码器判断上坡完成
            if (HAL_GetTick() - phase_timer > 2000) {
                bridge_phase = 2;
                phase_timer = HAL_GetTick();
            }
            break;

        case 2: // 桥面 (金属网，稳定速度)
            Motor_GoStraight(3500);
            // 金属网面需要稳定牵引力
            if (HAL_GetTick() - phase_timer > 3000) {
                bridge_phase = 3;
                phase_timer = HAL_GetTick();
            }
            break;

        case 3: // 下坡 (减速)
            Motor_GoStraight(2500);
            if (HAL_GetTick() - phase_timer > 2000) {
                bridge_phase = 4;
            }
            break;

        case 4: // 完成
            Motor_Stop_ms(500);
            bridge_phase = 0;
            g_task_state = TASK_P1_GATE;
            Debug_Print("Task2: Complete\r\n");
            break;
    }
}

/**
 * @brief 任务3：过门
 */
static void Task_Patio1_Gate(void)
{
    static uint8_t gate_phase = 0;
    static uint32_t phase_timer = 0;
    static uint32_t align_timer = 0;

    switch (gate_phase) {
        case 0: // 初始化，切换到目标检测模式
            OpenMV_SetMode(OPENMV_MODE_TARGET_FIND);
            phase_timer = HAL_GetTick();
            gate_phase = 1;
            Debug_Print("Task3: Gate Start\r\n");
            break;

        case 1: // 寻找门框
            if (OpenMV_Data.target_valid) {
                // 对准门中心
                int16_t offset = OpenMV_Data.target_x;
                if (offset > 20) {
                    Motor_Turn(2000, 30);  // 右转
                } else if (offset < -20) {
                    Motor_Turn(2000, -30); // 左转
                } else {
                    Motor_GoStraight(2500);
                    align_timer++;
                    if (align_timer > 50) {  // 对准后前进
                        gate_phase = 2;
                        phase_timer = HAL_GetTick();
                    }
                }
            } else {
                // 未找到，缓慢前进搜索
                Motor_GoStraight(1500);
                align_timer = 0;
            }

            // 超时
            if (HAL_GetTick() - phase_timer > 15000) {
                gate_phase = 2;
            }
            break;

        case 2: // 穿过门
            OpenMV_SetMode(OPENMV_MODE_COLOR_DETECT);
            Motor_GoStraight(2500);

            // 检测红线停止
            if (OpenMV_GetColor() == COLOR_RED) {
                Motor_Stop();
                gate_phase = 3;
            }

            // 超时保护
            if (HAL_GetTick() - phase_timer > 5000) {
                Motor_Stop();
                gate_phase = 3;
            }
            break;

        case 3: // 完成
            gate_phase = 0;
            align_timer = 0;
            g_task_state = TASK_STATE_DONE;
            Debug_Print("Task3: Complete - Patio1 Done!\r\n");
            break;
    }
}

/**
 * @brief 任务4：形状匹配
 */
static void Task_Patio2_ShapeMatch(void)
{
    static uint8_t shape_phase = 0;
    static uint32_t phase_timer = 0;
    static Arrow_Direction detected_arrow = ARROW_NONE;

    switch (shape_phase) {
        case 0: // 初始化，前进到方块区域
            OpenMV_SetMode(OPENMV_MODE_LANE_FOLLOW);
            Lane_SetBaseSpeed(2500);
            Lane_Start();
            phase_timer = HAL_GetTick();
            shape_phase = 1;
            Debug_Print("Task4: Shape Match Start\r\n");
            break;

        case 1: // 移动到方块区域
            Lane_Process();
            // 检测到停止标记或超时
            if (OpenMV_IsStopLine() || (HAL_GetTick() - phase_timer > 10000)) {
                Lane_Stop();
                Motor_Stop_ms(500);
                shape_phase = 2;
                phase_timer = HAL_GetTick();
            }
            break;

        case 2: // 切换到箭头识别模式
            OpenMV_SetMode(OPENMV_MODE_ARROW_DETECT);
            shape_phase = 3;
            break;

        case 3: // 识别箭头
            detected_arrow = OpenMV_GetArrow();
            if (detected_arrow != ARROW_NONE) {
                shape_phase = 4;
                Debug_Print("Arrow detected\r\n");
            }
            // 超时，使用默认直行
            if (HAL_GetTick() - phase_timer > 5000) {
                detected_arrow = ARROW_STRAIGHT;
                shape_phase = 4;
            }
            break;

        case 4: // 根据箭头方向移动
            Servo_SetAngle(SERVO_CENTER);
            if (detected_arrow == ARROW_LEFT) {
                // 左转到目标
                Motor_Turn(3000, -50);
                HAL_Delay(1500);
                Motor_GoStraight_ms(3500, 2000);
            } else if (detected_arrow == ARROW_RIGHT) {
                // 右转到目标
                Motor_Turn(3000, 50);
                HAL_Delay(1500);
                Motor_GoStraight_ms(3500, 2000);
            } else {
                // 直行到目标
                Motor_GoStraight_ms(3500, 2500);
            }
            shape_phase = 5;
            break;

        case 5: // 撞倒立牌
            Motor_GoStraight_ms(4000, 1000);
            Motor_Stop_ms(500);
            shape_phase = 0;
            detected_arrow = ARROW_NONE;
            g_task_state = TASK_P2_BALL_RELEASE;
            Debug_Print("Task4: Complete\r\n");
            break;
    }
}

/**
 * @brief 任务5：投球
 */
static void Task_Patio2_BallRelease(void)
{
    static uint8_t ball_phase = 0;
    static uint32_t phase_timer = 0;
    static uint32_t align_count = 0;

    switch (ball_phase) {
        case 0: // 初始化
            OpenMV_SetMode(OPENMV_MODE_TARGET_FIND);
            phase_timer = HAL_GetTick();
            ball_phase = 1;
            Debug_Print("Task5: Ball Release Start\r\n");
            break;

        case 1: // 移动到释放点，寻找篮筐
            if (OpenMV_Data.target_valid) {
                int16_t offset = OpenMV_Data.target_x;
                if (offset > 15) {
                    Motor_Turn(2000, 25);
                    align_count = 0;
                } else if (offset < -15) {
                    Motor_Turn(2000, -25);
                    align_count = 0;
                } else {
                    Motor_GoStraight(2000);
                    align_count++;
                }

                // 对准且接近
                if (align_count > 30) {
                    ball_phase = 2;
                    phase_timer = HAL_GetTick();
                }
            } else {
                // 搜索篮筐
                Motor_Spin(1500);
            }

            if (HAL_GetTick() - phase_timer > 20000) {
                ball_phase = 2;
            }
            break;

        case 2: // 接近篮筐
            Motor_GoStraight(2000);
            // 使用超声波检测距离
            if (SR_04.distance_filtered < 0.3f && SR_04.distance_filtered > 0.05f) {
                Motor_Stop();
                ball_phase = 3;
            }
            if (HAL_GetTick() - phase_timer > 5000) {
                Motor_Stop();
                ball_phase = 3;
            }
            break;

        case 3: // 释放球 (控制舵机释放机构)
            // 舵机动作释放球
            Servo_SetAngle(45);   // 打开释放机构
            HAL_Delay(1000);
            Servo_SetAngle(90);   // 复位
            ball_phase = 4;
            break;

        case 4: // 完成
            ball_phase = 0;
            align_count = 0;
            g_task_state = TASK_P2_COMMUNICATION;
            Debug_Print("Task5: Complete\r\n");
            break;
    }
}

/**
 * @brief 任务6：无线通信
 */
static void Task_Patio2_Communication(void)
{
    static uint8_t comm_phase = 0;
    static uint32_t phase_timer = 0;
    static uint8_t retry_count = 0;

    switch (comm_phase) {
        case 0: // 移动到种植区
            OpenMV_SetMode(OPENMV_MODE_LANE_FOLLOW);
            Lane_SetBaseSpeed(2500);
            Lane_Start();
            phase_timer = HAL_GetTick();
            comm_phase = 1;
            Debug_Print("Task6: Communication Start\r\n");
            break;

        case 1: // 前进到种植区
            Lane_Process();
            if (OpenMV_IsStopLine() || (HAL_GetTick() - phase_timer > 10000)) {
                Lane_Stop();
                Motor_Stop_ms(500);
                comm_phase = 2;
            }
            break;

        case 2: // 发送数据
            {
                char msg[64];
                // 获取RTC时间 (如果有)
                sprintf(msg, "Team:Rover,Date:2024-12-01,Time:12:00:00\r\n");
                HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 1000);
                phase_timer = HAL_GetTick();
                comm_phase = 3;
                Debug_Print("HC-12 Data Sent\r\n");
            }
            break;

        case 3: // 等待确认
            // 检查UART3是否收到确认
            // 简化处理：等待固定时间
            if (HAL_GetTick() - phase_timer > 2000) {
                if (retry_count < 3) {
                    // 重试
                    retry_count++;
                    comm_phase = 2;
                } else {
                    comm_phase = 4;
                }
            }
            break;

        case 4: // 移动到终点
            Motor_GoStraight_ms(3000, 3000);
            Motor_Stop();
            comm_phase = 5;
            break;

        case 5: // 完成
            comm_phase = 0;
            retry_count = 0;
            g_task_state = TASK_STATE_DONE;
            Debug_Print("Task6: Complete - Patio2 Done!\r\n");
            break;
    }
}

/**
 * @brief 设置舵机角度
 */
static void Servo_SetAngle(float angle)
{
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    uint16_t pulse = (uint16_t)(500 + (angle / 180.0f) * 2000);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pulse);
    g_servo_angle = angle;
}

/**
 * @brief 调试输出
 */
static void Debug_Print(const char *msg)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
}

/* USER CODE END 4 */

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
