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
//         此程序只能用作学习，如用商业用途。必追究责任！
//          
//
//
#include <stdlib.h>
#include <string.h>
#include "ALL_DATA.h"
#include "ANO_DT.h"
#include "USART.h"
#include "usartwt31.h"

/******************************************************************/
//--------------------------
static uint8_t RatePID[18];
static uint8_t AnglePID[18];
static uint8_t HighPID[18];

static struct{
	uint8_t PID1 :1; //接受到上位机PID组1
	uint8_t PID2 :1; //接受到上位机PID组2
	uint8_t PID3 :1; //接受到上位机PID组3
	uint8_t PID4 :1; //接受到上位机PID组4
	uint8_t PID5 :1; //接受到上位机PID组5
	uint8_t PID6 :1; //接受到上位机PID组6	
	uint8_t CMD2_READ_PID:1; //接受到上位机读取PID的请求
}ANTO_Recived_flag;


/***********************************************************************
 * 
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/	
void ANO_Recive(int8_t *pt)                   //接收到上位机的数据
{
	switch(pt[2])
	{
		case ANTO_RATE_PID:
			ANTO_Recived_flag.PID1 = 1;             //接收到上位机发来的PID数据
			memcpy(RatePID,&pt[4],18);              //先把接收到的数据提出来，防止被下一组PID数据覆盖，这组的PID是给速度环用的
			break;
		case ANTO_ANGLE_PID:                      //这组的PID是给角度环用的
			memcpy(AnglePID,&pt[4],18);
			ANTO_Recived_flag.PID2 = 1;
			break;
		case ANTO_HEIGHT_PID:                     //这组的PID是给高度环用的
			memcpy(HighPID,&pt[4],18);
			ANTO_Recived_flag.PID3 = 1;
			break;
		case ANTO_PID4:
			break;
		case ANTO_PID5:   
			break;
		case ANTO_PID6:
			break;
		case 0x01:                                //上位机发来的CMD1 包含各种校准

			break;
		case 0x02:                                //上位机发来的CMD2 包含请求读取PID等
			{
			   enum                                  //上位请求飞控类型
				{
					READ_PID = 0X01,                    //读取飞控的PID请求
					READ_MODE = 0x02,                   //读取飞行模式
					READ_ROUTE = 0x21,                  //读取航点信息
					READ_VERSION = 0XA0,                //读取飞控版本
					RETURN_DEFAULT_PID = 0xA1           //恢复默认PID
				 }CMD2;

				switch(*(uint8_t*)&pt[4])             //判断上位机发来CMD的内容
				{
					case READ_PID:                      //上位机请求读取飞控PID数据
						ANTO_Recived_flag.CMD2_READ_PID = 1;
						break;
					case READ_MODE: 
						break;
					case READ_ROUTE: 
						break;					
					case READ_VERSION:  
						break;
					case RETURN_DEFAULT_PID:  
						break;					
					default: 
						break;					
				}
			
			}
			break;
		case ANTO_RCDATA: //Immediately deal with 
			break;
		default:
			break;			
	}
	return;
}
/***********************************************************************
 * 
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/
static void ANTO_Send(const enum ANTO_SEND FUNCTION) //发送数据到上位机
{
	uint8_t i;
	uint8_t len=2;
	int16_t Anto[12];
	int8_t *pt = (int8_t*)(Anto);
	PidObject *pidX=0;
	PidObject *pidY=0;
	PidObject *pidZ=0;

	switch(FUNCTION)
	{
		case ANTO_RATE_PID:      //PID1
				 pidX = &pidRateX;
				 pidY = &pidRateY;
				 pidZ = &pidRateZ;
         goto send_pid;		
		case ANTO_ANGLE_PID:      //PID2
				 pidX = &pidRoll;
				 pidY = &pidPitch;
				 pidZ = &pidYaw;
				 goto send_pid;				
		case ANTO_HEIGHT_PID:     //PID3
				 pidX = &pidHeightRate;
				 pidY = &pidHeightHigh;
				 goto send_pid;							
		case ANTO_PID4:	  //PID4
		case ANTO_PID5:	         //PID5
    case ANTO_PID6:
send_pid:
			if(pidX!=NULL)
			{
				Anto[2] = (int16_t)(pidX->kp *1000);
				Anto[3] = (int16_t)(pidX->ki *1000);
				Anto[4] = (int16_t)(pidX->kd *1000);
			}
			if(pidY!=NULL)
			{
				Anto[5] = (int16_t)(pidY->kp *1000);
				Anto[6] = (int16_t)(pidY->ki *1000);
				Anto[7] = (int16_t)(pidY->kd *1000);
			}
			if(pidZ!=NULL)
			{
				Anto[8] = (int16_t)(pidZ->kp *1000);
				Anto[9] = (int16_t)(pidZ->ki *1000);
				Anto[10] = (int16_t)(pidZ->kd *1000);
			}
			len = 18;
			break;
		case ANTO_MOTOR:    //send motor

				len = 8;
			break;	
		case ANTO_RCDATA: //send RC data

			break;
		case ANTO_MPU_MAGIC:     //发送MPU6050和磁力计的数据
			memcpy(&Anto[2],(int8_t*)&MPU6050,sizeof(_st_Mpu));
			memcpy(&Anto[8],(int8_t*)&AK8975,sizeof(_st_Mag));
			len = 18;
			break;
		case ANTO_SENSER2:

			break;
		case ANTO_STATUS:     //send angle
			
				Anto[2] = (int16_t)(-Angle.roll*100);
				Anto[3] = (int16_t)(Angle.pitch*100);
				Anto[4] = (int16_t)(-Angle.yaw*100);
	   
				len = 6;
			break;
		case ANTO_POWER:

				break;
		default:
			break;			
	}
	
	Anto[0] = 0XAAAA;
	Anto[1] = len | FUNCTION<<8;
	pt[len+4] = (int8_t)(0xAA+0xAA);
	for(i=2;i<len+4;i+=2)    //a swap with b;
	{
		pt[i] ^= pt[i+1];
		pt[i+1] ^= pt[i];
		pt[i] ^= pt[i+1];
		pt[len+4] += pt[i] + pt[i+1];
	}
	
	USART3_SendByte(pt,len+5);
}
/***********************************************************************
 * polling  work.
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/
void ANTO_polling(void) //轮询扫描上位机端口
{
	volatile static uint8_t status = 0;
	switch(status)
	{
		case 0:
			
			status = 1;
			break;
		case 1:
			ANTO_Send(ANTO_MPU_MAGIC);
			delay_ms(1);
	  	ANTO_Send(ANTO_STATUS);
			delay_ms(1);

			if(*(uint8_t*)&ANTO_Recived_flag != 0) //一旦接收到上位机的数据，则暂停发送数据到上位机，转而去判断上位机要求飞控做什么。
			{
				status = 2;
			}
		 	break;
		case 2:
			if(*(uint8_t*)&ANTO_Recived_flag == 0)//上位机的发过来的数据都被处理了，则返回专心的发送数据到上位机
			{
				status = 1;
			}
	
			if(ANTO_Recived_flag.CMD2_READ_PID) //判断上位机是否请求发发送飞控PID数据到上位机
			{		
					ANTO_Send(ANTO_RATE_PID);
					delay_ms(1);
					ANTO_Send(ANTO_ANGLE_PID);
					delay_ms(1);
					ANTO_Send(ANTO_HEIGHT_PID);
					delay_ms(1);
					ANTO_Recived_flag.CMD2_READ_PID = 0;
			}
			
			if(*(uint8_t*)&ANTO_Recived_flag & 0x3f) //接收到上位机发来的PID数据
			{
					PidObject *pidX=0;
					PidObject *pidY=0;
					PidObject *pidZ=0;
				  uint8_t *P;
				
					if(ANTO_Recived_flag.PID1)
					{
						 pidX = &pidRateX;
						 pidY = &pidRateY;
						 pidZ = &pidRateZ;
						 P = RatePID;
						 ANTO_Recived_flag.PID1 = 0;
					}
					else if(ANTO_Recived_flag.PID2)
					{
						 pidX = &pidRoll;
						 pidY = &pidPitch;
						 pidZ = &pidYaw;
						 P = AnglePID;	
						 ANTO_Recived_flag.PID2 = 0;                             
					}
					else if(ANTO_Recived_flag.PID3)
					{
						 pidX = &pidHeightRate;
						 pidY = &pidHeightHigh;
						 P = HighPID;	
						 ANTO_Recived_flag.PID3 = 0;     						
					}
					else
					{
						break;
					}
					{
							union {
								uint16_t _16;
								uint8_t _u8[2];
							}data;
							
							if(pidX!=NULL)
							{
								data._u8[1] = P[0]; 
								data._u8[0] = P[1];
								pidX->kp =  data._16 /1000.0f;
								data._u8[1] = P[2]; 
								data._u8[0] = P[3];
								pidX->ki =  data._16 /1000.0f;
								data._u8[1] = P[4]; 
								data._u8[0] = P[5];
								pidX->kd =  data._16 /1000.0f;								
							}
							if(pidY!=NULL)
							{
								data._u8[1] = P[6]; 
								data._u8[0] = P[7];
								pidY->kp =  data._16 /1000.0f;
								data._u8[1] = P[8]; 
								data._u8[0] = P[9];
								pidY->ki =  data._16 /1000.0f;
								data._u8[1] = P[10]; 
								data._u8[0] = P[11];
								pidY->kd =  data._16 /1000.0f;		
							}
							if(pidZ!=NULL)
							{
								data._u8[1] = P[12]; 
								data._u8[0] = P[13];
								pidZ->kp =  data._16 /1000.0f;
								data._u8[1] = P[14]; 
								data._u8[0] = P[15];
								pidZ->ki =  data._16 /1000.0f;
								data._u8[1] = P[16]; 
								data._u8[0] = P[17];
								pidZ->kd =  data._16 /1000.0f;		
							}				
					}				
			}
			break;
		default:
			break;
	}

}

//static unsigned char TxBuffer[256];
//static unsigned char TxCounter=0;
//static unsigned char count=0; 

//void USART3_IRQHandler(void) //串口接收
//{ 	
//  	if(USART_GetITStatus(USART3, USART_IT_TXE) != RESET)
//  {   
//    USART_SendData(USART3, TxBuffer[TxCounter++]); 
//    if(TxCounter == count) 
//		{
//			USART_ITConfig(USART3, USART_IT_TXE, DISABLE);// 全部发送完成
//		}
//    USART_ClearITPendingBit(USART3, USART_IT_TXE); 
//  }
//	else if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
//  {
//		CopeSerial1Data((unsigned char)USART3->DR);//处理数据
//		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
//  }
//	USART_ClearITPendingBit(USART3,USART_IT_ORE);	
//	
//	
//	
//	
//	
//	
//	/*static  int8_t ReciveBuffer[25];
//  static uint8_t count;
//	if ((USART3->SR & USART_IT_ORE))//是否接收寄存器溢出
//	{
//	
//	}	
//  if((USART3->SR & USART_IT_RXNE))
//  {
//		ReciveBuffer[count] = USART3->DR;	
//		switch(count)
//		{
//		case 0:
//			if(ReciveBuffer[0]==(int8_t)0xAA)
//				count++;
//				break;
//		case 1:	
//			if(ReciveBuffer[1]==(int8_t)0xAF)
//					count++;
//			else 
//					count = 0;
//				break;
//		default:
//			if(count < ReciveBuffer[3]+4)
//			{
//				count++;
//				break;
//			}
//			else
//			{
//				uint8_t i;
//				int8_t CheckSum=0;
//							
//				for(i=0;i<count;i++)
//				{
//					CheckSum += ReciveBuffer[i];			
//				}		
//				if(CheckSum == ReciveBuffer[count])  //if the data calculate sum equal to the  final data from PC.
//				{
//						static int8_t CheckSend[7]={0xAA,0XAA,0xEF,2,0,0,0};	
//						
//						CheckSend[4] = ReciveBuffer[2];
//						CheckSend[5] = CheckSum;
//						CheckSend[6] = 0;
//						for(i=0;i<6;i++)
//						{
//							CheckSend[6] += CheckSend[i]; 					
//						}
//						USART3_SendByte(CheckSend,7);
//						ANO_Recive(ReciveBuffer);			//To arrange the data	and give the result to control argument.	
//				}			
//				count = 0;                  //return to the first data point,and retore from the head buffer next time.
//				ReciveBuffer[0] = 0;  //reset the data buffer.
//				ReciveBuffer[1] = 0;
//			}
//			break;							
//		}
//	}*/
//}

/************************END OF FILE********************/
