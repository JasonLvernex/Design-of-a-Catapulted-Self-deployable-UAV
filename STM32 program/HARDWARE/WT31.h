#ifndef __WT31_H
#define	__WT31_H

#include "stm32f10x.h"
#include <stdio.h>

#include "control.h"
#include "mpu6050.h"

struct SAcc
{
	short a[3];
	short T;
};
struct SGyro
{
	short w[3];
	short T;
};
struct SAngle
{
	short Angle[3];
	short T;
};

typedef volatile struct
{
	float desired;     //< set point
	float offset;      //
	float prevError;    //< previous error
	float integ;        //< integral
	float kp;           //< proportional gain
	float ki;           //< integral gain
	float kd;           //< derivative gain
	float IntegLimitHigh;       //< integral limit
	float IntegLimitLow;
	float measured;
	float out;
	float OutLimitHigh;
	float OutLimitLow;
}PidWT;
 


void sendcmd(char cmd[]);
void CopeSerial2Data(unsigned char ucData);
int WTbalance(void);
void WTpidUpdate(PidWT* pid,const float dt);
void WTpidRest(PidWT **pid,const uint8_t len);
void WTFlightPidControl(float dt);
void WTMotorControl(void);
void wtpid_para(void);
int PID_Calc(int NextPoint,int SetPoint);
void new_wt_rotor(int pid1,int pid2);
void deal_data(void);

								

#endif 

