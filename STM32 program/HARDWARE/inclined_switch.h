#ifndef _INCLINED_H__
#define _INCLINED_H__
#endif

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "delayy.h"

#define titled GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)


void title_sw_init(void);
int get_titled(void);
void EPP_init(void);	
