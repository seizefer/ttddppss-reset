#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Servo.h"
#include "Serial.h"

uint8_t KeyNum;			//定义用于接收键码的变量
float Angle;			//定义角度变量
uint8_t count;  //控制左右转向
uint8_t RxData;

int main(void)
{
	/*模块初始化*/
	OLED_Init();		//OLED初始化
	Servo_Init();		//舵机初始化
	Serial_Init();		//串口初始化
	
	
	/*显示静态字符串*/
	OLED_ShowString(1, 1, "Angle:");	//1行1列显示字符串Angle:
	OLED_ShowString(2, 1, "Count:");	//1行1列显示字符串Count:
	OLED_ShowString(3, 1, "RxData:");
	
	count = 0;
	
	while (1)
	{
		while(count<=2)
		{
			count++;
			if (count == 2)			
			{
				Angle = 90;				
			}
			Servo_SetAngle(Angle);			//设置舵机的角度为角度变量
			Delay_ms(500);
		}//跳过第一轮
		
		if (Serial_GetRxFlag() == 1)			//检查串口接收数据的标志位
		{
			RxData = Serial_GetRxData();		//获取串口接收的数据
			Serial_SendByte(RxData);			//串口将收到的数据回传回去，用于测试
			OLED_ShowHexNum(3, 8, RxData, 2);	//显示串口接收的数据
			if (RxData == 0xf)
			{
				Angle = 135; //右转45度
			}
			if (RxData == 0x0)
			{
				Angle = 45; //左转45度
			}
			if (RxData == 0xa)
			{
				Angle = 90; //直行
			}
		}
		
		Servo_SetAngle(Angle);			//设置舵机的角度为角度变量
		OLED_ShowNum(1, 7, Angle, 3);	//OLED显示角度变量
	}
}
