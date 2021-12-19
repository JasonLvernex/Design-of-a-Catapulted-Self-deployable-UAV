#ifndef __LED_H
#define __LED_H

#include "sys.h"
#include "GPIO.H"

#define     LED1(x)       x? (GPIOB->BSRR=GPIO_Pin_8):(GPIOB->BRR=GPIO_Pin_8)
#define     LED2(x)       x? (GPIOB->BSRR=GPIO_Pin_5):(GPIOB->BRR=GPIO_Pin_5)
#define     LED3(x)       x? (GPIOB->BSRR=GPIO_Pin_10):(GPIOB->BRR=GPIO_Pin_10)
#define     LED4(x)       x? (GPIOB->BSRR=GPIO_Pin_9):(GPIOB->BRR=GPIO_Pin_9)
#define     OP_LED1       (GPIOB->IDR&GPIO_Pin_8)? (GPIOB->BRR=GPIO_Pin_8):(GPIOB->BSRR=GPIO_Pin_8)
#define     OP_LED2       (GPIOB->IDR&GPIO_Pin_5)? (GPIOB->BRR=GPIO_Pin_5):(GPIOB->BSRR=GPIO_Pin_5)
#define     OP_LED3       (GPIOB->IDR&GPIO_Pin_10)? (GPIOB->BRR=GPIO_Pin_10):(GPIOB->BSRR=GPIO_Pin_10)
#define     OP_LED4       (GPIOB->IDR&GPIO_Pin_9)? (GPIOB->BRR=GPIO_Pin_9):(GPIOB->BSRR=GPIO_Pin_9)


typedef struct
{
	uint16_t FlashTime;
	enum
	{
		AlwaysOn, 
		AlwaysOff, 
		AllFlashLight,
		AlternateFlash, 
		WARNING,
		DANGEROURS,
		GET_OFFSET	
	}status; 
}sLED;	

extern sLED LED;
extern void LEDInit(void);
extern void LEDtest(void);
extern void PilotLED(void);

#define LED_TAKE_OFF_ENTER      LED.status = WARNING  
#define LED_TAKE_OFF_EXIT       LED.status = AllFlashLight  
#define LED_HEIGHT_LOCK_ENTER   LED.FlashTime=50;LED.status = AlternateFlash
#define LED_HEIGHT_LOCK_EXIT    LED.FlashTime=100;LED.status = AllFlashLight
#define LED_3D_ROLL_ENTER				LED.status = WARNING 
#define LED_3D_ROLL_EXIT        LED.status = AllFlashLight  
#define LED_SAFTY_TAKE_DOWN_ENTER  LED.status = DANGEROURS 
#define LED_SAFTY_TAKE_DOWN_EXIT   LED.status = AlwaysOn
#define LED_GET_MPU_OFFSET_ENTER   LED.status = GET_OFFSET
#define LED_GO_HOME_ENTER       LED.status = WARNING    
#define LED_GO_HOME_EXIT        LED.status = AllFlashLight  

#endif 


