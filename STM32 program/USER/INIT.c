//========================================================================
//	�����ߵ��ӹ�����-�Ա� https://devotee.taobao.com/
//	STM32���ᰮ����QQȺ: 810149456
//	���ߣ�С��
//	�绰:13728698082
//	����:1042763631@qq.com
//	���ڣ�2018.05.17
//	�汾��V1.0
//========================================================================
//�׼������ַ��https://devotee.taobao.com/
//                 �����ߵ��ӹ�����
//�ش�������
//
//         �˳���ֻ�� ����ѧϰ��������ҵ��;����׷�����Σ�
//          
//
//
#include "ALL_DEFINE.h"
#include "unfold.h"
#include "wt31.h"
#include "usartwt31.h"




volatile uint32_t SysTick_count; //ϵͳʱ�����
_st_Mpu MPU6050;   //MPU6050ԭʼ����
_st_Mag AK8975;
_st_AngE Angle;    //��ǰ�Ƕ���ֵ̬
_st_Remote Remote; //ң��ͨ��ֵ


_st_ALL_flag ALL_flag; //ϵͳ��־λ������������־λ��


PidObject pidRateX; //�ڻ�PID����
PidObject pidRateY;
PidObject pidRateZ;

PidObject pidPitch; //�⻷PID����
PidObject pidRoll;
PidObject pidYaw;

PidObject pidHeightRate;
PidObject pidHeightHigh;

/*extern struct SAcc 		stcAcc;
extern struct SGyro 		stcGyro;
extern struct SAngle 	stcAngle;*/
extern struct SAngle 	stcAngle;


uint8_t RC_manualData[32];
static uint8_t execute_flag=0;


void pid_param_Init(void); //PID���Ʋ�����ʼ������дPID�����ᱣ�����ݣ��������ɺ�ֱ���ڳ�������� ����¼���ɿ�


void ALL_Init(void)
 {
	int mpu_cnt=0;
	 //uint32_t tt;
ALL_flag.unlock = 0;
	 
	IIC_Init();             //I2C��ʼ��
	
	//IIC_Init_MAG();
	
	pid_param_Init();       //PID������ʼ��
	 
	LEDInit();              //LED���Ƴ�ʼ��


	//Initial_UART3(9600);
	USART3_Config();        //��λ�����ڳ�ʼ��
	 
	 //Initial_UART1(115200);
	 //	USART1_Config();        //WT31���ڳ�ʼ��

    wtpid_para();          //WTPID_INITIALIZE
	 
//		TIM3_Config();					//ϵͳ�������ڳ�ʼ��
//		TIM_Cmd(TIM3, DISABLE);
		
		TIM2_PWM_Config();			//4·PWM��ʼ��
	
   NRF24L01_init();				//2.4Gң��ͨ�ų�ʼ��
	
  unfold_init();
//GPIOB->ODR ^= GPIO_Pin_1;// test sentence
		
		//tt=get_z_mag();		
/*while(1)
{
    printf("Angle:%.3f %.3f \r\n",(float)stcAngle.Angle[0]/32768*180,(float)stcAngle.Angle[1]/32768*180);
		delay_ms(10);//�ȴ��������
}*/	
					
do
{
	 
if(NRF24L01_RxPacket(RC_manualData)==SUCCESS)
{ 
	  uint8_t i;
		uint8_t CheckSum=0;

		for(i=0;i<31;i++)
		{
			CheckSum +=  RC_manualData[i];
		}
	
		if(RC_manualData[31]==CheckSum &&RC_manualData[0]==0xAA && RC_manualData[1]==0xAF)  //������յ���ң��������ȷ
		{
			
			switch(RC_manualData[30])
			{
				case 0:
					break;
				case 1:
				{
					break;
				}
				case 9:
				{
					MpuInit();
					execute_flag=1;
          ALL_flag.flight_mode=0;//manual mode
					break;
				}
			  default:
				{
		while(!get_titled()){};/**///��Ҫ����д��б�ж�����

	  while( !WTbalance()){};
     ALL_flag.unfold_rd=1;  //ң���йܱ�־λ
		 ALL_flag.flight_mode=1; //�ɻ�Ϊ�׷�ģʽ
		 MpuInit();              //MPU6050��ʼ��
	 
//		 do //quick shift to mpu
//	 {
//		 MpuGetData();
//		 FlightPidControl(0.003f);
//		 MotorControl();
//		 mpu_cnt++;
//		 delay_ms(3);
//	 }while(mpu_cnt<10);
    execute_flag=1;	
	  break;
				}
				
			}
					
		}
		
}
}while(NRF24L01_RxPacket(RC_manualData)!=SUCCESS||execute_flag==0);
//  TIM_Cmd(TIM3, ENABLE);
	TIM3_Config();					//ϵͳ�������ڳ�ʼ��
  USART_Cmd(USART3, DISABLE);
  ALL_flag.procesure_rd=1;
}



void pid_param_Init(void)
{
	
///////////////////////////////////////////////	
	pidRateX.kp = 2.0f;
	pidRateY.kp = 2.0f;
	pidRateZ.kp = 4.0f;
	
	pidRateX.ki = 0.0f;
	pidRateY.ki = 0.0f;
	pidRateZ.ki = 0.0f;	
	
	pidRateX.kd = 0.08f;
	pidRateY.kd = 0.08f;
	pidRateZ.kd = 0.5f;	
	
	pidPitch.kp = 7.0f;
	pidRoll.kp = 7.0f;
	pidYaw.kp = 7.0f;	
////////////////////////////////////////////////////
	
//	pidRateX.kp = 2.0f;
//	pidRateY.kp = 2.0f;
//	pidRateZ.kp = 4.0f;
//	
//	pidRateX.ki = 0.0f;
//	pidRateY.ki = 0.0f;
//	pidRateZ.ki = 0.0f;	
//	
//	pidRateX.kd = 0.28f;
//	pidRateY.kd = 0.28f;
//	pidRateZ.kd = 0.4f;	
//	
//	pidPitch.kp = 7.0f;
//	pidRoll.kp = 7.0f;
//	pidYaw.kp = 7.0f;	

}










