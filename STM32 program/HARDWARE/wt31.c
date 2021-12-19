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

float   Proportion=0.6;                               //�������� Proportional Const
float   Integral=0.1;                                 //���ֳ��� Integral Const
float   Derivative=0;                                 //΢�ֳ��� Derivative Const
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

//�ô���2��JYģ�鷢��ָ��
void sendcmd(char cmd[])
{
	char i;
	for(i=0;i<3;i++)
		UART2_Put_Char(cmd[i]);
}

//CopeSerialDataΪ����2�жϵ��ú���������ÿ�յ�һ�����ݣ�����һ�����������
void CopeSerial2Data(unsigned char ucData)
{
	
			

	ucRxBuffer[ucRxCnt]=ucData;	//���յ������ݴ��뻺������
	ucRxCnt++;
	if (ucRxBuffer[0]!=0x55) //����ͷ���ԣ������¿�ʼѰ��0x55����ͷ
	{
		ucRxCnt=0;
		return;
	}
		if (ucRxCnt>1&&ucRxBuffer[1]!=0x53) //����ͷ���ԣ������¿�ʼѰ��0x55����ͷ
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {;}//���ݲ���11�����򷵻�
	else
	{
		switch(ucRxBuffer[1])//�ж��������������ݣ�Ȼ���俽������Ӧ�Ľṹ���У���Щ���ݰ���Ҫͨ����λ���򿪶�Ӧ������󣬲��ܽ��յ�������ݰ�������
		{
			//memcpyΪ�������Դ����ڴ濽��������������"string.h"�������ջ��������ַ����������ݽṹ�����棬�Ӷ�ʵ�����ݵĽ�����
			//case 0x51:	memcpy(&stcAcc,&ucRxBuffer[2],8);break;
			//case 0x52:	memcpy(&stcGyro,&ucRxBuffer[2],8);break;
			case 0x53:	//memcpy(&stcAngle,&ucRxBuffer[2],8);break;
            stcAngle.Angle[0] = (((unsigned short)ucRxBuffer[3]<<8)|ucRxBuffer[2]);
						stcAngle.Angle[1] = (((unsigned short)ucRxBuffer[5]<<8)|ucRxBuffer[4]);
						stcAngle.Angle[2] = (((unsigned short)ucRxBuffer[7]<<8)|ucRxBuffer[6]);
						
		}
		ucRxCnt=0;//��ջ�����
	}
	
}//CopeSerialDataΪ����2�жϵ��ú���������ÿ�յ�һ�����ݣ�����һ�����������

////////////////////////////////////////////////////////////////////

void deal_data(void)
{
		if (ucRxBuffer[0]!=0x55) //����ͷ���ԣ������¿�ʼѰ��0x55����ͷ
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {;}//���ݲ���11�����򷵻�
	else
	{
		switch(ucRxBuffer[1])//�ж��������������ݣ�Ȼ���俽������Ӧ�Ľṹ���У���Щ���ݰ���Ҫͨ����λ���򿪶�Ӧ������󣬲��ܽ��յ�������ݰ�������
		{
			//memcpyΪ�������Դ����ڴ濽��������������"string.h"�������ջ��������ַ����������ݽṹ�����棬�Ӷ�ʵ�����ݵĽ�����
			//case 0x51:	memcpy(&stcAcc,&ucRxBuffer[2],8);break;
			//case 0x52:	memcpy(&stcGyro,&ucRxBuffer[2],8);break;
			case 0x53:	//memcpy(&stcAngle,&ucRxBuffer[2],8);break;
            stcAngle.Angle[0] = (((unsigned short)ucRxBuffer[3]<<8)|ucRxBuffer[2])/32768*180;
						stcAngle.Angle[1] = (((unsigned short)ucRxBuffer[5]<<8)|ucRxBuffer[4])/32768*180;
						
						
		}
		ucRxCnt=0;//��ջ�����
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
		delay_ms(10);//�ȴ��������
		 
		 
		 _cnt++;
		 if (_cnt>1000) upper=0;
	 } while(upper==1);//����д�ж�����  upper==1||_cnt>300
	 
/*{TIM_ITConfig(  //ʹ�ܻ���ʧ��ָ����TIM�ж�
		TIM3, //TIM2
		TIM_IT_Update ,
		DISABLE  //ʹ��
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
	else  ready_manual_flag=0;//֮���Ϊ1
	return ready_manual_flag;
}

//mag pid control blocks
void WTpidUpdate(PidWT* pidwt,const float dt)//dt ������������õ���0.003f
{
	 float error;
	 float deriv;
	
    error = pidwt->desired - pidwt->measured; //��ǰ�Ƕ���ʵ�ʽǶȵ����

    pidwt->integ += error * dt;	 //�������ۼ�ֵ
	
	//  pid->integ = LIMIT(pid->integ,pid->IntegLimitLow,pid->IntegLimitHigh); //���л����޷�

    deriv = (error - pidwt->prevError)/dt;  //ǰ�����������΢��
	
    pidwt->out = pidwt->kp * error + pidwt->ki * pidwt->integ + pidwt->kd * deriv;//PID���
	
		//pid->out = LIMIT(pid->out,pid->OutLimitLow,pid->OutLimitHigh); //����޷�
		
    pidwt->prevError = error;  //�����ϴε����
		
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
		case 2:  //׼���������

      WTpidRest(pidwt,2);
		  pidangX.desired=temp;
		   pidangY.desired=temp;
			status = 3;		
		case 3: //��ʽ�������
			
		//  Angle[0] = (short(chrTemp[3]<<8|chrTemp[2]))/32768.0*180;
     // Angle[1] = (short(chrTemp[5]<<8|chrTemp[4]))/32768.0*180;
			
     pidangX.measured = (float)stcAngle.Angle[0]/32768*180; //�ڻ�����ֵ �Ƕ�/��
		 pidangY.measured=  (float)stcAngle.Angle[1]/32768*180;

		
		 	WTpidUpdate(&pidangX,dt);    //����PID�������������⻷	�����PID	
		  WTpidUpdate(&pidangY,dt);
		
			pidangX.desired = pidangX.out;
		  pidangY.desired = pidangY.out;
		
		
      mag_finish_flag=0;
		
		  status = 3;
			break;
		case 4:  //�˳�����
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



//�������

//#define MOTOR1 motor[0] 
//#define MOTOR2 motor[1] 
//#define MOTOR3 motor[2] 
//#define MOTOR4 motor[3]

void WTMotorControl(void)
{	
	int16_t motor[4];
	volatile static uint8_t status=1;

	
	
	/*if(ALL_flag.unlock == 1) //�����������ʹ��ң�ؽ����������ɿؾͿ������κ�����½�����ֹ���У��������������˳�PID����
		status =4;	*/
	switch(status)
	{		
		case 1: //�ȴ�����	
			motor[0] = motor[1] = motor[2] = motor[3] = 0;  //������������������Ϊ0
			if(unfold_rotor())
			{
				status = 2;
			}
		case 2: //������ɺ��ж�ʹ�����Ƿ�ʼ����ң�˽��з��п���
			if(1)//���Լ������ж�
			{
				motor[0] = motor[1] = motor[2] = motor[3] = 200;
				status = 3;
			}
			break;
		case 3:
			{
				//int16_t temp;
				
		//�������
				//MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = LIMIT(temp,0,900); //��100����̬����
				MOTOR1 +=    - pidangX.out + pidangY.out ;//; ��̬����������������Ŀ�����
				MOTOR2 +=    - pidangX.out - pidangY.out  ;//;
				MOTOR3 +=    + pidangX.out - pidangY.out ;
				MOTOR4 +=    + pidangX.out + pidangY.out ;//;
			}	
			break;
		default:
			break;
	}
	
	
	TIM2->CCR1 = LIMIT(motor[0],0,1000);  //����PWM
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


//////�¼ӵ�



int PID_Calc(int NextPoint,int SetPoint) 
{                      
	static int      LastError;                                //Error[-1]
	static int      PrevError;                                //Error[-2]
  int iError,Outpid;                                        //��ǰ���
	
  iError=SetPoint-NextPoint;                                //��������
  Outpid=(Proportion * iError)                              //E[k]��
              -(Integral * LastError)                       //E[k-1]��
              +(Derivative * PrevError);                    //E[k-2]��
              
  PrevError=LastError;                                      //�洢�������´μ���
  LastError=iError;
  return(Outpid);                                           //��������ֵ
}



void new_wt_rotor(int pid1,int pid2)
{
	      MOTOR1 +=    - pid1 - pid2 ;//; ��̬����������������Ŀ�����
				MOTOR2 +=    - pid1 + pid2  ;//;
				MOTOR3 +=    + pid1 - pid2 ;
				MOTOR4 +=    + pid1 + pid2 ;//;
	TIM2->CCR1 = LIMIT(motor[0],0,1000);  //����PWM
	TIM2->CCR2 = LIMIT(motor[1],0,1000);
	TIM2->CCR3 = LIMIT(motor[2],0,1000);
	TIM2->CCR4 = LIMIT(motor[3],0,1000);
}


