#include "unfold.h"
#include "control.h"
#include "mpu6050.h"



#define fLED_H()  GPIOB->BSRR = GPIO_Pin_1
#define fLED_L()  GPIOB->BRR  = GPIO_Pin_1
#define fLED_Toggle()  GPIOB->ODR ^= GPIO_Pin_1

#define hLED_H()  GPIOB->BSRR = GPIO_Pin_2
#define hLED_L()  GPIOB->BRR  = GPIO_Pin_2
#define hLED_Toggle()  GPIOB->ODR ^= GPIO_Pin_2

//-------------------------------------------------
//����ǰ��			 
#define aLED_H()  GPIOB->BSRR = GPIO_Pin_8
#define aLED_L()  GPIOB->BRR  = GPIO_Pin_8
#define aLED_Toggle()  GPIOB->ODR ^= GPIO_Pin_8

#define bLED_H()  GPIOB->BSRR = GPIO_Pin_9
#define bLED_L()  GPIOB->BRR  = GPIO_Pin_9
#define bLED_Toggle()  GPIOB->ODR ^= GPIO_Pin_9

PidMag pidmagm;

uint16_t VirtAddVarTab[] = {0xAA00, 0xAA01, 0xAA02, 0xAA03, 0xAA04, 0xAA05, 0xAA06, 0xAA07, 0xAA08, 0xAA09, 
																		 0xAA0A,0xAA0B, 0xAA0C, 0xAA0D, 0xAA0E, 0xAA0F, 0xAA10, 0xAA11, 0xAA12, 0xAA13, 0xAA14, 
																			0xAA15, 0xAA16, 0xAA17, 0xAA18};



/*����չ������*/
int unfold_rotor(void)
{  int unfold_ready_flag=0,unfold_flag=0;
	unfold_ready_flag=get_titled();
	/*ִ��չ���Լ���Ӧ���ڳ�ʼ��*/
	if(unfold_ready_flag==1)//����û�����ɹ�
	{
  GPIOB->ODR ^= GPIO_Pin_1;// test sentence
	GPIOA->BSRR  = GPIO_Pin_7;//high
  delay_ms(200);		
	GPIOA->BRR  = GPIO_Pin_7;//low
	unfold_flag=1;
 }
		
	else {unfold_flag=0;}	

 return unfold_flag;

}

/*�شŴ�����΢��ƽ�⺯��*/
/*int magbalance(void)
{
	int _cnt=0;
	int unfold_flag=1,ready_manual_flag=0;
	if(unfold_flag==unfold_rotor())
	{	
		//GPIOB->ODR ^= GPIO_Pin_1;// test sentence
		
		do
	 {
	 MAGFlightPidControl(0.003f,&pidmagm);
	 MAGMotorControl();
		 _cnt++;
	 } while( pidmagm.desiredmag - pidmagm.measuredmag>10&&(_cnt<30));
	 
//{TIM_ITConfig(  //ʹ�ܻ���ʧ��ָ����TIM�ж�
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
	 }while(mpu_cnt<10); //

		ready_manual_flag=1;
	}
	else  ready_manual_flag=0;//֮���Ϊ1
	return ready_manual_flag;
}
*/
/*
//mag pid control blocks
void magpidUpdate(PidMag* pidmagg,const float dt)//dt ������������õ���0.003f
{
	 float error;
	 float deriv;
	
    error = pidmagg->desiredmag - pidmagg->measuredmag; //��ǰ�Ƕ���ʵ�ʽǶȵ����

    pidmagg->integmag += error * dt;	 //�������ۼ�ֵ
	
	//  pid->integ = LIMIT(pid->integ,pid->IntegLimitLow,pid->IntegLimitHigh); //���л����޷�

    deriv = (error - pidmagg->prevErrormag)/dt;  //ǰ�����������΢��
	
    pidmagg->outmag = pidmagg->kpmag * error + pidmagg->kimag * pidmagg->integmag + pidmagg->kdmag * deriv;//PID���
	
		//pid->out = LIMIT(pid->out,pid->OutLimitLow,pid->OutLimitHigh); //����޷�
		
    pidmagg->prevErrormag = error;  //�����ϴε����
		
}*/
/*
//PidMag *(magPidObject[])={&pidmagm};
void magpidRest(PidMag **pid,const uint8_t len)
{
	uint8_t i;
	for(i=0;i<len;i++)
	{
	  	pid[i]->integmag = 0;
	    pid[i]->prevErrormag = 0;
	    pid[i]->outmag = 0;
		pid[i]->offsetmag = 0;
	}
}
*/
/*
//pid control actuator
void MAGFlightPidControl(float dt,PidMag* pidmagmg)
{
	volatile static uint8_t status=1;
	int mag_finish_flag=1;
	u16 temp;
while(mag_finish_flag)
{
	switch(status)
	{		
		case 1: 
			if(unfold_rotor())//rotor unfold suceeded
			{
				status = 2;	
			}			
			break;
		case 2:  //׼���������

      magpidRest(&pidmagmg,1);
			STMFLASH_Read(VirtAddVarTab[0],&temp,4);  //���� wanted z
		  pidmagmg->desiredmag=temp;
			status = 3;
		
			break;			
		case 3: //��ʽ�������
			
      pidmagmg->measuredmag = (u16)get_z_mag; //�ڻ�����ֵ �Ƕ�/��

		
		 	magpidUpdate(pidmagmg,dt);    //����PID�������������⻷	�����PID		
			pidmagmg->desiredmag = pidmagmg->outmag; //���⻷��PID�����Ϊ�ڻ�PID������ֵ��Ϊ����PID
      mag_finish_flag=0;
		  status = 1;
			break;
		case 4:  //�˳�����
			magpidRest(&pidmagmg,1);
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

//re range the data
u16 range_out (uint32_t out)
{ 
	u16 range,std_z=0;
	if(set_std_zmag())
	{
		STMFLASH_Read(VirtAddVarTab[1],&std_z,4);
		range=std_z/1000;
		out=out/range;
	}
	return out; 
}

//�������

#define MOTOR1 motor[0] 
#define MOTOR2 motor[1] 
#define MOTOR3 motor[2] 
#define MOTOR4 motor[3]*/ 

/*void MAGMotorControl(void)
{	
	int16_t motor[4];
	volatile static uint8_t status=1;
	static int  run_first_flag1=1,run_first_flag2=1,cases1_flag=0,cases2_flag=0,casess1_flag=0,casess2_flag=0;
	
	
	//if(ALL_flag.unlock == 1) //�����������ʹ��ң�ؽ����������ɿؾͿ������κ�����½�����ֹ���У��������������˳�PID����
		status =4;	*/
/*	switch(status)
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
		//�������
				int cases=0,casess=0;
				if(run_first_flag1)
				{
					motor[0] += +(0.5*range_out (pidmagm.outmag));
					motor[2] += -(0.5*range_out (pidmagm.outmag));
					run_first_flag1=0;
				}
				if(pidmagm.prevErrormag<pidmagm.desiredmag-pidmagm.measuredmag)//need to set an range
				{
					cases=1;
				}
				else if(pidmagm.prevErrormag>pidmagm.desiredmag-pidmagm.measuredmag){cases=2;}
				
			if(!(cases1_flag&cases2_flag))	
			{switch(cases)
				{
					case 1: 
					{	motor[0] += +(0.5*range_out (pidmagm.outmag));
					  motor[2] += -(0.5*range_out (pidmagm.outmag));
					 cases1_flag=1;
					break;}
					case 2:
					{	motor[0] += -(0.5*range_out (pidmagm.outmag));
					  motor[2] += +(0.5*range_out (pidmagm.outmag));
					cases2_flag=1;
					break;}
					default: break;	
				};}
			
				if(run_first_flag2)
				{
					motor[1] += +(0.5*range_out (pidmagm.outmag));
					motor[3] += -(0.5*range_out (pidmagm.outmag));
					run_first_flag2=0;
				}
				if(pidmagm.prevErrormag<pidmagm.desiredmag-pidmagm.measuredmag)
				{
					casess=1;
				}
				else if(pidmagm.prevErrormag>pidmagm.desiredmag-pidmagm.measuredmag){cases=2;}
				
			if(!(casess1_flag&casess2_flag))	
			{switch(casess)
				{
					case 1: 
					{	motor[0] += +(0.5*range_out (pidmagm.outmag));
					  motor[2] += -(0.5*range_out (pidmagm.outmag));
					 casess1_flag=1;
					break;}
					case 2:
					{	motor[0] += -(0.5*range_out (pidmagm.outmag));
					  motor[2] += +(0.5*range_out (pidmagm.outmag));
					casess2_flag=1;
					break;}
					default: break;	
				};}
			

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
*/
void unfold_init(void)
{   

	EPP_init();
	title_sw_init();
	//MAG3110_Init();
	//EE_Init();
	
	/* {	
	pidmagm.kdmag = 2.0f;	
	pidmagm.kimag = 0.0f;
	pidmagm.kpmag = 0.5f;	
	 } */

  //TIM_Cmd(TIM3, DISABLE);


	
//GPIOB->ODR ^= GPIO_Pin_1;// test sentence

 }

