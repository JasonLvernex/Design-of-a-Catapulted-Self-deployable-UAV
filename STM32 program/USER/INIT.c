//========================================================================
//	爱好者电子工作室-淘宝 https://devotee.taobao.com/
//	STM32四轴爱好者QQ群: 810149456
//	作者：小刘
//	电话:13728698082
//	邮箱:1042763631@qq.com
//	日期：2018.05.17
//	版本：V1.0
//========================================================================
//套件购买地址：https://devotee.taobao.com/
//                 爱好者电子工作室
//特此声明：
//
//         此程序只能 用作学习，如用商业用途。必追究责任！
//          
//
//
#include "ALL_DEFINE.h"
#include "unfold.h"
#include "wt31.h"
#include "usartwt31.h"




volatile uint32_t SysTick_count; //系统时间计数
_st_Mpu MPU6050;   //MPU6050原始数据
_st_Mag AK8975;
_st_AngE Angle;    //当前角度姿态值
_st_Remote Remote; //遥控通道值


_st_ALL_flag ALL_flag; //系统标志位，包含解锁标志位等


PidObject pidRateX; //内环PID数据
PidObject pidRateY;
PidObject pidRateZ;

PidObject pidPitch; //外环PID数据
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


void pid_param_Init(void); //PID控制参数初始化，改写PID并不会保存数据，请调试完成后直接在程序里更改 再烧录到飞控


void ALL_Init(void)
 {
	int mpu_cnt=0;
	 //uint32_t tt;
ALL_flag.unlock = 0;
	 
	IIC_Init();             //I2C初始化
	
	//IIC_Init_MAG();
	
	pid_param_Init();       //PID参数初始化
	 
	LEDInit();              //LED闪灯初始化


	//Initial_UART3(9600);
	USART3_Config();        //上位机串口初始化
	 
	 //Initial_UART1(115200);
	 //	USART1_Config();        //WT31串口初始化

    wtpid_para();          //WTPID_INITIALIZE
	 
//		TIM3_Config();					//系统工作周期初始化
//		TIM_Cmd(TIM3, DISABLE);
		
		TIM2_PWM_Config();			//4路PWM初始化
	
   NRF24L01_init();				//2.4G遥控通信初始化
	
  unfold_init();
//GPIOB->ODR ^= GPIO_Pin_1;// test sentence
		
		//tt=get_z_mag();		
/*while(1)
{
    printf("Angle:%.3f %.3f \r\n",(float)stcAngle.Angle[0]/32768*180,(float)stcAngle.Angle[1]/32768*180);
		delay_ms(10);//等待传输完成
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
	
		if(RC_manualData[31]==CheckSum &&RC_manualData[0]==0xAA && RC_manualData[1]==0xAF)  //如果接收到的遥控数据正确
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
		while(!get_titled()){};/**///需要重新写倾斜判断条件

	  while( !WTbalance()){};
     ALL_flag.unfold_rd=1;  //遥控托管标志位
		 ALL_flag.flight_mode=1; //飞机为抛飞模式
		 MpuInit();              //MPU6050初始化
	 
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
	TIM3_Config();					//系统工作周期初始化
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










