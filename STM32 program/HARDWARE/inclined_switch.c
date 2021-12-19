#include "inclined_switch.h"
#include "delay.h"



//倾斜开关初始化
void title_sw_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA , &GPIO_InitStructure); 
	
	
}
void EPP_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA , &GPIO_InitStructure); 
	//GPIOA->BRR  = GPIO_Pin_7;//LOW
	
}

//倾斜开关数据读取
int  get_titled(void)
{ int tl=0;
	if(titled==0)
	{ //GPIOB->ODR ^= GPIO_Pin_1;// test sentence
	  delay_ms(10);
		//GPIOB->ODR ^= GPIO_Pin_1;// test sentence
		if(titled==0){tl=1;
		               // GPIOB->ODR ^= GPIO_Pin_1;// test sentence
		               }
	}
	
return tl;
}

