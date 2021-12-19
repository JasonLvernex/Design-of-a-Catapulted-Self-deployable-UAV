#include <string.h>
#include <stdio.h>

#include "ALL_DATA.h" 
#include "ALL_DEFINE.h" 
#include "control.h"
#include "pid.h"
#include "wt31.h"
#include "usartwt31.h"
#include "delay.h"
#include "unfold.h"


struct SAcc 		stcAcc;
struct SGyro 		stcGyro;
struct SAngle 	stcAngle;

static unsigned char ucRxBuffer[250];
static unsigned char ucRxCnt = 0;


char YAWCMD[3] = {0XFF,0XAA,0X52};
char ACCCMD[3] = {0XFF,0XAA,0X67};
char SLEEPCMD[3] = {0XFF,0XAA,0X60};
char UARTMODECMD[3] = {0XFF,0XAA,0X61};
char IICMODECMD[3] = {0XFF,0XAA,0X62};

float   Proportion=0.6;                               //比例常数 Proportional Const
float   Integral=0.1;                                 //积分常数 Integral Const
float   Derivative=0;                                 //微分常数 Derivative Const
extern int motor[4];
#define BS 0
#define MOTOR1 motor[0] 
#define MOTOR2 motor[1] 
#define MOTOR3 motor[2] 
#define MOTOR4 motor[3]
int pid1,pid2,_cnt=0;
int x_angle,y_angle,z_angle;

//PidWT pidwt;

PidWT pidangX;
PidWT pidangY;

PidWT *(pidwt[])={&pidangX,&pidangY};

//用串口2给JY模块发送指令
void sendcmd(char cmd[])
{
	char i;
	for(i=0;i<3;i++)
		UART2_Put_Char(cmd[i]);
}

//CopeSerialData为串口2中断调用函数，串口每收到一个数据，调用一次这个函数。
void CopeSerial2Data(unsigned char ucData)
{
	
			

	ucRxBuffer[ucRxCnt]=ucData;	//将收到的数据存入缓冲区中
	ucRxCnt++;
	if (ucRxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt=0;
		return;
	}
		if (ucRxCnt>1&&ucRxBuffer[1]!=0x53) //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {;}//数据不满11个，则返回
	else
	{
		switch(ucRxBuffer[1])//判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据
		{
			//memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据结构体里面，从而实现数据的解析。
			//case 0x51:	memcpy(&stcAcc,&ucRxBuffer[2],8);break;
			//case 0x52:	memcpy(&stcGyro,&ucRxBuffer[2],8);break;
			case 0x53:	//memcpy(&stcAngle,&ucRxBuffer[2],8);break;
            stcAngle.Angle[0] = (((unsigned short)ucRxBuffer[3]<<8)|ucRxBuffer[2]);
						stcAngle.Angle[1] = (((unsigned short)ucRxBuffer[5]<<8)|ucRxBuffer[4]);
						stcAngle.Angle[2] = (((unsigned short)ucRxBuffer[7]<<8)|ucRxBuffer[6]);
						
		}
		ucRxCnt=0;//清空缓存区
	}
	
}//CopeSerialData为串口2中断调用函数，串口每收到一个数据，调用一次这个函数。

////////////////////////////////////////////////////////////////////

void deal_data(void)
{
		if (ucRxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {;}//数据不满11个，则返回
	else
	{
		switch(ucRxBuffer[1])//判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据
		{
			//memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据结构体里面，从而实现数据的解析。
			//case 0x51:	memcpy(&stcAcc,&ucRxBuffer[2],8);break;
			//case 0x52:	memcpy(&stcGyro,&ucRxBuffer[2],8);break;
			case 0x53:	//memcpy(&stcAngle,&ucRxBuffer[2],8);break;
            stcAngle.Angle[0] = (((unsigned short)ucRxBuffer[3]<<8)|ucRxBuffer[2])/32768*180;
						stcAngle.Angle[1] = (((unsigned short)ucRxBuffer[5]<<8)|ucRxBuffer[4])/32768*180;
						
						
		}
		ucRxCnt=0;//清空缓存区
	}
}



int WTbalance(void)
{
	int upper;
	int unfold_flag=1,ready_manual_flag=0;
	if(1)///unfold_flag==unfold_rotor()
	{	
		//GPIOB->ODR ^= GPIO_Pin_1;// test sentence
		unfold_rotor();
		do
	 {
	 //WTFlightPidControl(0.003f);
	 //WTMotorControl();
		 //deal_data();
		 pid1=pid2=0;
		 upper=0;
		 x_angle=(float)stcAngle.Angle[0]/32768*180;
		 y_angle=(float)stcAngle.Angle[1]/32768*180;
		 z_angle=(float)stcAngle.Angle[2]/32768*180;
		 if(x_angle>30||x_angle<-30)
		 {pid1=PID_Calc(x_angle,BS) ;upper=1;}
		 if(y_angle>30||y_angle<-30)
		 {pid2=PID_Calc(y_angle,BS) ;upper=1;}	
		 new_wt_rotor(pid1,pid2);
	// printf("Angle:%.3f %.3f \r\n",(float)stcAngle.Angle[0]/32768*180,(float)stcAngle.Angle[1]/32768*180);
		delay_ms(10);//等待传输完成
		 
		 
		 _cnt++;
		 if (_cnt>1000) upper=0;
	 } while(upper==1);//重新写判断条件  upper==1||_cnt>300
	 
/*{TIM_ITConfig(  //使能或者失能指定的TIM中断
		TIM3, //TIM2
		TIM_IT_Update ,
		DISABLE  //使能
 );}	*/
	/* do //quick shift to mpu
	 {
		 MpuGetData();
		 FlightPidControl(0.003f);
		 MotorControl();
		 mpu_cnt++;
		 delay_ms(3);
	 }while(mpu_cnt<10); */

		ready_manual_flag=1;
	}
	else  ready_manual_flag=0;//之后改为1
	return ready_manual_flag;
}

//mag pid control blocks
void WTpidUpdate(PidWT* pidwt,const float dt)//dt 在这个例程里用的是0.003f
{
	 float error;
	 float deriv;
	
    error = pidwt->desired - pidwt->measured; //当前角度与实际角度的误差

    pidwt->integ += error * dt;	 //误差积分累加值
	
	//  pid->integ = LIMIT(pid->integ,pid->IntegLimitLow,pid->IntegLimitHigh); //进行积分限幅

    deriv = (error - pidwt->prevError)/dt;  //前后两次误差做微分
	
    pidwt->out = pidwt->kp * error + pidwt->ki * pidwt->integ + pidwt->kd * deriv;//PID输出
	
		//pid->out = LIMIT(pid->out,pid->OutLimitLow,pid->OutLimitHigh); //输出限幅
		
    pidwt->prevError = error;  //更新上次的误差
		
}

//PidMag *(magPidObject[])={&pidmagm};
void WTpidRest(PidWT **pid,const uint8_t len)
{
	uint8_t i;
	for(i=0;i<len;i++)
	{
	  	pid[i]->integ = 0;
	    pid[i]->prevError = 0;
	    pid[i]->out = 0;
		pid[i]->offset = 0;
	}
}


//pid control actuator
void WTFlightPidControl(float dt)
{
	volatile static uint8_t status=1;
	//double Angle[2];
	int mag_finish_flag=1;
	float temp=0;
while(mag_finish_flag)
{
	switch(status)
	{		
		case 1: 
			if(unfold_rotor())//rotor unfold suceeded
			{
				status = 2;	
			}			
			else break;
		case 2:  //准备进入控制

      WTpidRest(pidwt,2);
		  pidangX.desired=temp;
		   pidangY.desired=temp;
			status = 3;		
		case 3: //正式进入控制
			
		//  Angle[0] = (short(chrTemp[3]<<8|chrTemp[2]))/32768.0*180;
     // Angle[1] = (short(chrTemp[5]<<8|chrTemp[4]))/32768.0*180;
			
     pidangX.measured = (float)stcAngle.Angle[0]/32768*180; //内环测量值 角度/秒
		 pidangY.measured=  (float)stcAngle.Angle[1]/32768*180;

		
		 	WTpidUpdate(&pidangX,dt);    //调用PID处理函数来处理外环	横滚角PID	
		  WTpidUpdate(&pidangY,dt);
		
			pidangX.desired = pidangX.out;
		  pidangY.desired = pidangY.out;
		
		
      mag_finish_flag=0;
		
		  status = 3;
			break;
		case 4:  //退出控制
			WTpidRest(pidwt,2);
		  mag_finish_flag=0;
			status = 1;
		  break;
		default:
			status = 4;
			break;
	}
	delay_ms(3);
}
}



//电机控制

//#define MOTOR1 motor[0] 
//#define MOTOR2 motor[1] 
//#define MOTOR3 motor[2] 
//#define MOTOR4 motor[3]

void WTMotorControl(void)
{	
	int16_t motor[4];
	volatile static uint8_t status=1;

	
	
	/*if(ALL_flag.unlock == 1) //意外情况，请使用遥控紧急上锁，飞控就可以在任何情况下紧急中止飞行，锁定飞行器，退出PID控制
		status =4;	*/
	switch(status)
	{		
		case 1: //等待解锁	
			motor[0] = motor[1] = motor[2] = motor[3] = 0;  //如果锁定，则电机输出都为0
			if(unfold_rotor())
			{
				status = 2;
			}
		case 2: //解锁完成后判断使用者是否开始拨动遥杆进行飞行控制
			if(1)//可以加锁死判断
			{
				motor[0] = motor[1] = motor[2] = motor[3] = 200;
				status = 3;
			}
			break;
		case 3:
			{
				//int16_t temp;
				
		//油门相关
				//MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = LIMIT(temp,0,900); //留100给姿态控制
				MOTOR1 +=    - pidangX.out + pidangY.out ;//; 姿态输出分配给各个电机的控制量
				MOTOR2 +=    - pidangX.out - pidangY.out  ;//;
				MOTOR3 +=    + pidangX.out - pidangY.out ;
				MOTOR4 +=    + pidangX.out + pidangY.out ;//;
			}	
			break;
		default:
			break;
	}
	
	
	TIM2->CCR1 = LIMIT(motor[0],0,1000);  //更新PWM
	TIM2->CCR2 = LIMIT(motor[1],0,1000);
	TIM2->CCR3 = LIMIT(motor[2],0,1000);
	TIM2->CCR4 = LIMIT(motor[3],0,1000);
} 

void wtpid_para(void)
{
	pidangX.kd = 10.0f;	
	pidangX.ki = 0.0f;
	pidangX.kp = 0.5f;	
	pidangY.kd = 10.0f;	
	pidangY.ki = 0.0f;
	pidangY.kp = 0.5f;	
	
}


//////新加的



int PID_Calc(int NextPoint,int SetPoint) 
{                      
	static int      LastError;                                //Error[-1]
	static int      PrevError;                                //Error[-2]
  int iError,Outpid;                                        //当前误差
	
  iError=SetPoint-NextPoint;                                //增量计算
  Outpid=(Proportion * iError)                              //E[k]项
              -(Integral * LastError)                       //E[k-1]项
              +(Derivative * PrevError);                    //E[k-2]项
              
  PrevError=LastError;                                      //存储误差，用于下次计算
  LastError=iError;
  return(Outpid);                                           //返回增量值
}



void new_wt_rotor(int pid1,int pid2)
{
	      MOTOR1 +=    - pid1 - pid2 ;//; 姿态输出分配给各个电机的控制量
				MOTOR2 +=    - pid1 + pid2  ;//;
				MOTOR3 +=    + pid1 - pid2 ;
				MOTOR4 +=    + pid1 + pid2 ;//;
	TIM2->CCR1 = LIMIT(motor[0],0,1000);  //更新PWM
	TIM2->CCR2 = LIMIT(motor[1],0,1000);
	TIM2->CCR3 = LIMIT(motor[2],0,1000);
	TIM2->CCR4 = LIMIT(motor[3],0,1000);
}


