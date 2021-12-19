#ifndef _UNFOLD_H__
#define _UNFOLD_H__
#endif

#include "stm32f10x.h"
#include "inclined_switch.h"
#include "delayy.h"
//#include "megnetometer.h"
#include "led.h"
#include <stdbool.h>
#include "myMath.h"	
#include "all_data.h"

/*
B3 左前
A3 右后
A1 左后
B2 右后
*/





typedef volatile struct
{
	uint32_t desiredmag;     //< set point
	uint32_t offsetmag;      //
	uint32_t prevErrormag;    //< previous error
	uint32_t integmag;        //< integral
	uint32_t kpmag;           //< proportional gain
	uint32_t kimag;           //< integral gain
	uint32_t kdmag;           //< derivative gain
	uint32_t IntegLimitHighmag;       //< integral limit
	uint32_t IntegLimitLowmag;
	uint32_t measuredmag;
	uint32_t outmag;
	uint32_t OutLimitHighmag;
	uint32_t OutLimitLowmag;
}PidMag;



int unfold_rotor(void);
//int magbalance(void);
//void magpidUpdate(PidMag* pidmag,const float dt);
//void MAGFlightPidControl(float dt,PidMag* pidmagmg);
//void MAGMotorControl(void);
void unfold_init(void);
//void magpidRest(PidMag **pid,const uint8_t len);
//u16 range_out (uint32_t out);
