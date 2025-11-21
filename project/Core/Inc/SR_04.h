#ifndef _SR_04_H
#define _SR_04_H
#include "main.h"


typedef struct
{
    uint8_t flag;
    int time;
    float distance;
    float distance_filtered;
}sr_04;

extern sr_04 SR_04;

void SR_04_Start(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void SR_04_Callback(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void SR_04_Get_distance(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN);

#endif
