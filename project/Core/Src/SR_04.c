#include "SR_04.h"
#include "tim.h"
#include "main.h"
#include "filter.h"

sr_04 SR_04 = {0,0,0,0};

/**
    *启动测量
    *@param *GPIOx      输出引脚所在的GPIO组
            GPIO_Pin    TRIG输出引脚，配置为GPIO_OUTPUT，默认下拉
    *          
    *@return 无
*/
void SR_04_Start(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) // (GPIOB, GPIO_PIN_12) 
{
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
		HAL_TIM_Base_Start_IT(&htim5);
		while((&htim5)->Instance->CNT < 20){}
			HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
			(&htim5)->Instance->CNT = 0;
			HAL_TIM_Base_Stop_IT(&htim5);
}

/**
    *SR_04回调函数进行状态判断
    *@param *GPIOx      输入引脚所在的GPIO组
            GPIO_Pin    ECHO输入引脚，配置为外部中断，默认下拉
    *          
    *@return 无
*/
void SR_04_Callback(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) 
{
    
    //EXIT设置的是双边沿触发模式，为了区分上升沿下降沿，采用类似状态机的思想加一个标志位
    //flag = 0时说明检测到上升沿
    //flag = 1时说明检测到下降沿
		if(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin))
		{
			SR_04.flag = 1;
		}
		else if(!HAL_GPIO_ReadPin(GPIOx, GPIO_Pin))
		{
			SR_04.flag = 0;
		}
    SR_04_Get_distance();
}

/**
    *测距函数
    *@param     无
    *          
    *@return    无
*/
void SR_04_Get_distance()
{
    switch (SR_04.flag)
    {
    case 0:
        SR_04.time = (&htim5)->Instance->CNT;
        HAL_TIM_Base_Stop_IT(&htim5);   //停止计数
        (&htim5)->Instance->CNT = 0;	//计数清零
        break;
    case 1:
        HAL_TIM_Base_Start_IT(&htim5);  //启动TIM5，并使能它的中断 
        break;
    default:
        break;
    }
    
    SR_04.distance = (SR_04.time / 10e5) * 340 / 2.0f;  //时间的单位是us
    SR_04.distance_filtered = kalmanFilter(&kfp_distance ,SR_04.distance);
}

/**
    *重写中断回调函数
    *@param GPIO_Pin    外部中断引脚
    *          
    *@return 无
*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN)
{
    if(GPIO_PIN == GPIO_PIN_13)  //外部中断（若改变需修改）
    {
        SR_04_Callback(GPIOB, GPIO_PIN_13);
    }
}
