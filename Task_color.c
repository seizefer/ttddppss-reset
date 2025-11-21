#include "Task_color.h"
#include "stdio.h"
#include "stm32f405xx.h"

float number ;
char  Rxflag = 0;//表示为接收到颜色
volatile uint8_t received_data = 0; // 用于存储接收到的数据，作为全局变量


void USART1_Init(void)
{
	//USART1_TX:PA 9   
	//USART1_RX:PA10
	GPIO_InitTypeDef GPIO_InitStructure;     //���ڶ˿����ýṹ�����
	USART_InitTypeDef USART_InitStructure;   //���ڲ������ýṹ�����
	NVIC_InitTypeDef NVIC_InitStructure;     //�����ж����ýṹ�����

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);   //��PA�˿�ʱ��

    //USART1_TX   PA9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;          		 //PA9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  		 //�趨IO�ڵ�����ٶ�Ϊ50MHz
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	   		 //�����������
    GPIO_Init(GPIOA, &GPIO_InitStructure);             	 	 //��ʼ��PA9
    //USART1_RX	  PA10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;             //PA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  //��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);                 //��ʼ��PA10 

    //USART1 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;  //��ռ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		    //�����ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			      //IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	                          //����ָ���Ĳ�����ʼ��VIC�Ĵ���

    //USART ��ʼ������
	USART_InitStructure.USART_BaudRate = 115200;                  //���ڲ�����Ϊ115200
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;   //�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;        //һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;           //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   //��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	                  //�շ�ģʽ
    USART_Init(USART1, &USART_InitStructure);                     //��ʼ������1

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //ʹ���ж�
    USART_Cmd(USART1, ENABLE);                     //ʹ�ܴ���1

  	//�����������1���ֽ��޷���ȷ���ͳ�ȥ������
	USART_ClearFlag(USART1, USART_FLAG_TC);        //�崮��1���ͱ�־
		
}

// //USART1 ȫ���жϷ�����
// void USART1_IRQHandler(void)			 
// {
// 	u8 com_data; 
// 	u8 i;
// 	static u8 RxCounter1=0;
// 	static u16 RxBuffer1[10]={0};
// 	static u8 RxState = 0;	
// 	static u8 RxFlag1 = 0;


// 	if( USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET)  	   //�����ж�  
// 	{
// 		USART_ClearITPendingBit(USART1,USART_IT_RXNE);   //����жϱ�־
// 		com_data = USART_ReceiveData(USART1);
			
// 		if(RxState==0&&com_data==0x2C)  
// 		{
// 			RxState=1;
// 			RxBuffer1[RxCounter1++]=com_data;
// 		}
		
// 		else if(RxState==1&&com_data==0x12)  
// 		{
// 	        RxState=2;
// 			RxBuffer1[RxCounter1++]=com_data;
// 		}
		
// 		else if(RxState==2)
// 		{
// 			RxBuffer1[RxCounter1++]=com_data;

// 			if(RxCounter1>=10||com_data == 0x5B)       //RxBuffer1��������,�������ݽ���
// 			{
// 				RxState=3;
// 				RxFlag1=1;
// 			}
// 		}
		
// 		else if(RxState==3)		//����Ƿ���ܵ�������־
// 		{
// 			if(RxBuffer1[RxCounter1-1] == 0x5B)
// 			{
// 				USART_ITConfig(USART1,USART_IT_RXNE,DISABLE);//�ر�DTSABLE�ж�
// 				RxFlag1 = 0;
// 				RxCounter1 = 0;
// 				RxState = 0;
// 				USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
// 			}
// 		    else   //���մ���
// 			{
// 				RxState = 0;
// 				RxCounter1=0;
// 				for(i=0;i<10;i++)
// 				{
// 					RxBuffer1[i]=0x00;      //�����������������
// 				}
// 			}
// 		} 
	
// 		else   //�����쳣
// 		{
// 			RxState = 0;
// 			RxCounter1=0;
// 			for(i=0;i<10;i++)
// 			{
// 				RxBuffer1[i]=0x00;      //�����������������
// 			}
// 	    }

// 	}
		
// }

void USART1_IRQHandler(void)//串口中断函数
{
	u8 com_data; 
	static u8 cnt=0;
	static u8 recieve[10]={0};
	static u8 RxState = 0;



	if( USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET)
    //接收中断 
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE); //中断清除标志
		com_data = USART_ReceiveData(USART1);
		
		if(RxState==0&&com_data=='@')
        { 
        //这里为了让stm32接收端与openmv发送端数据格式相同，所以首先要判断接收数据是否等于@（@是0x2c帧头，上面openmv端数据开头就是@）
			RxState=1;//从@下一个数据开始接收
			receive[cnt++]=com_data;
		}
		else if(RxState==1)
        {
        //代表已经接收到了@，开始接收颜色标志数字
		    recieve[cnt++]=com_data;
			if(cnt>3||com_data == '@')
            {       															
            //接收到最后的@后表示数据接收完成
			RxState=2;
			number = recieve[2]-48;
			}
		}
		else if(RxState==2)																												
        //检测是否接收到最后的结束标志@
		{
			if(receive[cnt-1] == '@')
			{	
				USART_ITConfig(USART1,USART_IT_RXNE,DISABLE);										
                //关闭接收中断
				Rxflag = 1;//表示接收完数据
                RxState = 0;


                 // 解析接收到的数据，并存储到全局变量 received_data 中
                if (cnt == 2 && recive[1] >= '1' && recive[1] <= '3') {
                    received_data = recive[1] - '0'; // 将字符转换为对应的数字
                } else {
                    // 如果接收到的数据不符合要求，则将 received_data 设为 0
                    received_data = 0;
                }
				cnt = 0;


			}else{
                recive[cnt++] = com_data;
            }
		} 
	}
}

