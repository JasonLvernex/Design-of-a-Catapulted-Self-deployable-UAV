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
//         �˳���ֻ������ѧϰ��������ҵ��;����׷�����Σ�
//          
//
//
#include "ALL_DATA.h"
#include "nrf24l01.h"
#include "control.h"
#include <math.h>
#include "myMath.h"
#include "LED.h"
#include "Remote.h"


#define SUCCESS 0
#undef FAILED
#define FAILED  1
/*****************************************************************************************
 *  ͨ�����ݴ���
 * @param[in] 
 * @param[out] 
 * @return     
 ******************************************************************************************/	
	uint8_t RC_rxData[32];
void remote_unlock(void);	
void RC_Analy(void)  
{
		static uint16_t cnt;
/*             Receive  and check RC data                               */	
	if(NRF24L01_RxPacket(RC_rxData)==SUCCESS)
	{ 	

		uint8_t i;
		uint8_t CheckSum=0;
		cnt = 0;
		for(i=0;i<31;i++)
		{
			CheckSum +=  RC_rxData[i];
		}
		if(RC_rxData[31]==CheckSum && RC_rxData[0]==0xAA && RC_rxData[1]==0xAF)  //������յ���ң��������ȷ
		{
			  Remote.roll = ((uint16_t)RC_rxData[4]<<8) | RC_rxData[5];  //ͨ��1
				LIMIT(Remote.roll,1000,2000);
				Remote.pitch = ((uint16_t)RC_rxData[6]<<8) | RC_rxData[7];  //ͨ��2
				LIMIT(Remote.pitch,1000,2000);
				Remote.thr = 	((uint16_t)RC_rxData[8]<<8) | RC_rxData[9];   //ͨ��3
				LIMIT(Remote.thr,1000,2000);
				Remote.yaw =  ((uint16_t)RC_rxData[10]<<8) | RC_rxData[11];   //ͨ��4
				LIMIT(Remote.yaw,1000,2000);
				Remote.AUX1 =  ((uint16_t)RC_rxData[12]<<8) | RC_rxData[13];   //ͨ��5  ���Ͻǰ���������ͨ��5  
				LIMIT(Remote.AUX1,1000,2000);
				Remote.AUX2 =  ((uint16_t)RC_rxData[14]<<8) | RC_rxData[15];   //ͨ��6  ���Ͻǰ���������ͨ��6 
				LIMIT(Remote.AUX2,1000,2000);
				Remote.AUX3 =  ((uint16_t)RC_rxData[16]<<8) | RC_rxData[17];   //ͨ��7  ���±߰���������ͨ��7 
				LIMIT(Remote.AUX1,1000,2000);
				Remote.AUX4 =  ((uint16_t)RC_rxData[18]<<8) | RC_rxData[19];   //ͨ��8  ���±߰���������ͨ��6  
				LIMIT(Remote.AUX2,1000,2000);		

				
				{
							const float roll_pitch_ratio = 0.04f;
							const float yaw_ratio =  0.0015f;    
					
							pidPitch.desired =-(Remote.pitch-1500)*roll_pitch_ratio;	 //��ң��ֵ��Ϊ���нǶȵ�����ֵ
							pidRoll.desired = -(Remote.roll-1500)*roll_pitch_ratio;
					    if(Remote.yaw>1820)
							{
								pidYaw.desired += 0.75f;	
							}
							else if(Remote.yaw <1180)
							{
								pidYaw.desired -= 0.75f;	
							}						
				}
				remote_unlock();
			
		}
  }
//���3��û�յ�ң�����ݣ����ж�ң���źŶ�ʧ���ɿ����κ�ʱ��ֹͣ���У��������ˡ�
//���������ʹ���߿ɽ����ر�ң�ص�Դ������������3��������رգ��������ˡ�
//�����ر�ң�أ�����ڷ����л�ֱ�ӵ��䣬���ܻ��𻵷�������
	else
	{
	
		
		cnt++;
		if(cnt>800)
		{
			cnt = 0;
			ALL_flag.unlock = 0; 
			NRF24L01_init();
		}
	}
}

/*****************************************************************************************
 *  �����ж�
 * @param[in] 
 * @param[out] 
 * @return     
 ******************************************************************************************/	
void remote_unlock(void)
{
	volatile static uint8_t status=WAITING_1;
	static uint16_t cnt=0;

	if(Remote.thr<1200 &&Remote.yaw<1200)                         //����ң�����½�����
	{
		status = EXIT_255;
	}
	
	switch(status)
	{
		case WAITING_1://�ȴ�����
			if(Remote.thr<1150)           //���������࣬�������->�������->������� ����LED�Ʋ����� ����ɽ���
			{			 
					 status = WAITING_2;				 
			}		
			break;
		case WAITING_2:
			if(Remote.thr>1800)          
			{		
						static uint8_t cnt = 0;
					 	cnt++;		
						if(cnt>5) //��������豣��200ms���ϣ���ֹң�ؿ�����ʼ��δ��ɵĴ�������
						{	
								cnt=0;
								status = WAITING_3;
						}
			}			
			break;
		case WAITING_3:
			if(Remote.thr<1150)          
			{			 
					 status = WAITING_4;				 
			}			
					
			break;			
		case WAITING_4:	//����ǰ׼��	               
				ALL_flag.unlock = 1;
        ALL_flag.unfold_rd=0;
				status = PROCESS_31;
		if(ALL_flag.flight_mode==1)
		{
		    ALL_flag.maintain_hight=1;
		}
		/*      if(ALL_flag.unfold_rd==1&&Remote.thr<1400)  //�ֶ�����ʱ��ı��ָ߶�
			{
           status = WAITING_4; 
			}
			else  ALL_flag.unfold_rd=0;*/
			
				LED.status = AlwaysOn;									
				 break;		
		case PROCESS_31:	//�������״̬
				if(Remote.thr<1020)
				{
					if(cnt++ > 2000)                                     // ����ң�˴������6S�Զ�����
					{								
						status = EXIT_255;								
					}
				}
				else if(!ALL_flag.unlock)                           //Other conditions lock 
				{
					status = EXIT_255;				
				}
				else					
					cnt = 0;
			break;
		case EXIT_255: //��������
			LED.status = AllFlashLight;	                                 //exit
			cnt = 0;
			LED.FlashTime = 100; //100*3ms		
			ALL_flag.unlock = 0;
			status = WAITING_1;
			break;
		default:
			status = EXIT_255;
			break;
	}
}
/***********************END OF FILE*************************************/







