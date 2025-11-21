/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "stdio.h"
#include "u8g2.h"
#include "Servo.h"

#include "SR_04.h"
#include "filter.h"

void SystemClock_Config(void);
uint8_t Open_Door_Flag = 0x02;
uint8_t Close_Door_Flag = 0x01;

float Angle = 0.0; // ??????

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) // tim12?????,???????
{
  if (htim->Instance == htim12.Instance)
  {
    SR_04_Start(GPIOB, GPIO_PIN_12);
  }
}
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */

void GoStraight_ms(uint32_t Velocity, uint32_t time_in_ms)
{
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 100);
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, Velocity);
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, 100);
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, Velocity);

  HAL_Delay(time_in_ms);
}

void GoStraight(uint32_t Velocity)
{
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 100);
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, Velocity);
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, 100);
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, Velocity);
}

void Stop()
{
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 100);
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 100);
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, 100);
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, 100);
}

void Stop_ms(uint32_t time_in_ms)
{
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 100);
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 100);
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, 100);
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, 100);

  HAL_Delay(time_in_ms);
}

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

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
  HAL_TIM_Base_Start_IT(&htim12); // ?????timer12
                                  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t Open_Door_Flag = 0x02;
  uint8_t Close_Door_Flag = 0x01;

  uint8_t receive_buffer[3]; // ????????
  uint8_t send_buffer[4];    // ????????
  uint8_t receive_count = 0; // ?????
  // ??OpenMV????????
  while (1)
  {
    // if (SR_04.distance_filtered < 0.15f)
    // {
    //   HAL_UART_Transmit(&huart3, &Close_Door_Flag, sizeof(Close_Door_Flag), 0xFF);
    //   HAL_UART_Transmit(&huart3, &Close_Door_Flag, sizeof(Close_Door_Flag), 0xFF);
    //   HAL_UART_Transmit(&huart3, &Close_Door_Flag, sizeof(Close_Door_Flag), 0xFF);
    //   HAL_UART_Transmit(&huart3, &Close_Door_Flag, sizeof(Close_Door_Flag), 0xFF);
    //   HAL_UART_Transmit(&huart3, &Close_Door_Flag, sizeof(Close_Door_Flag), 0xFF);
    //   HAL_UART_Transmit(&huart3, &Close_Door_Flag, sizeof(Close_Door_Flag), 0xFF);
    //   HAL_UART_Transmit(&huart3, &Close_Door_Flag, sizeof(Close_Door_Flag), 0xFF);
    // }
    // else if (SR_04.distance_filtered > 0.15f)
    // {
    //   HAL_UART_Transmit(&huart3, &Open_Door_Flag, sizeof(Open_Door_Flag), 0xFF);
    //   HAL_UART_Transmit(&huart3, &Open_Door_Flag, sizeof(Open_Door_Flag), 0xFF);
    //   HAL_UART_Transmit(&huart3, &Open_Door_Flag, sizeof(Open_Door_Flag), 0xFF);
    //   HAL_UART_Transmit(&huart3, &Open_Door_Flag, sizeof(Open_Door_Flag), 0xFF);
    //   HAL_UART_Transmit(&huart3, &Open_Door_Flag, sizeof(Open_Door_Flag), 0xFF);
    //   HAL_UART_Transmit(&huart3, &Open_Door_Flag, sizeof(Open_Door_Flag), 0xFF);
    // }
    // ???????????
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) != RESET)
    {
      // ??????????,?????
      uint8_t com_data = (uint8_t)(huart2.Instance->DR & (uint8_t)0x00FF);

      // ???????
      __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_RXNE);

      // ????????
      receive_buffer[receive_count++] = com_data;

      if (SR_04.distance_filtered < 0.15f && SR_04.distance_filtered > 0.0017f) // ??
      {
        if (receive_count == 1)
        {
          if (receive_buffer[0] == 0 || receive_buffer[0] == 1 || receive_buffer[0] == 2)
          {
            Angle = 135; // ??45?
            Servo_SetAngle(Angle);
            HAL_Delay(500);
            Angle = 90; //??
            Servo_SetAngle(Angle);
            HAL_Delay(500);
            Angle = 45; // ??45?
            Servo_SetAngle(Angle);
            HAL_Delay(500);
            Angle = 90; // ??
            Servo_SetAngle(Angle);
            HAL_Delay(500);
            Angle = 45; // ??45?
            Servo_SetAngle(Angle);
            HAL_Delay(500);
            Angle = 90; //??
            Servo_SetAngle(Angle);
            HAL_Delay(500);
            Angle = 135; // ??45?
            Servo_SetAngle(Angle);
            HAL_Delay(500);
          }
          else if (receive_buffer[0] == 10)
          {
            Stop_ms(10000);
            // ??10s
          }
        }
      }
      else if (SR_04.distance_filtered > 0.15f)
      {
        if (receive_count == 1)
        {
          if (receive_buffer[0] == 0)
          {
            Angle = 90; // ??
            Servo_SetAngle(Angle);
            GoStraight(2200);
            HAL_Delay(50);
          }
          else if (receive_buffer[0] == 1)
          {
            Angle = 45; // ??45?
            Servo_SetAngle(Angle);
            GoStraight(2200);
            HAL_Delay(50);
          }
          else if (receive_buffer[0] == 2)
          {
            Angle = 135; // ??45?
            Servo_SetAngle(Angle);
            GoStraight(2200);
            HAL_Delay(50);
          }

          else if (receive_buffer[0] == 3) // 识别到直行
          {
          }
          else if (receive_buffer[0] == 4) // 识别到左转箭头
          {
            Angle = 45; // 左转45度
            Servo_SetAngle(Angle);
            GoStraight(3000);
            HAL_Delay(1500);
          }
          else if (receive_buffer[0] == 5) // 识别到右转箭头
          {
            Angle = 135; // 左转45度
            Servo_SetAngle(Angle);
            GoStraight(3000);
            HAL_Delay(1500);
          }
          else if (receive_buffer[0] == 6) // 识别到红灯
          {
            Stop_ms(50);
          }
          else if (receive_buffer[0] == 7) // 识别到绿灯
          {
            GoStraight_ms(5000, 2000);
          }
          else if (receive_buffer[0] == 11) // 蓝牙
          {
            Stop_ms(5000);
            HAL_UART_Transmit(&huart3, &Open_Door_Flag, sizeof(Open_Door_Flag), 0xFF);
            GoStraight_ms(3000, 4000);
            Stop();
          }

          // ????????
          if (HAL_UART_Transmit(&huart2, receive_buffer, 3, HAL_MAX_DELAY) != HAL_OK)
          {
            // ??????
            Error_Handler();
          }

          // ???????
          receive_count = 0;
          /* USER CODE END 3 */
        }

        /* USER CODE END 3 */
      }
    }
  }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
