#include "megnetometer.h"
#include "ALL_DATA.h"





#define MAG3110_IIC_ADDRESS       0x1C//this is the write ID;0x0E is the device IP #define MAG3110_IIC_ADDRESS       0XD2		//设置FLASH 保存地址(必须为偶数，且其值要大于本代码所占用FLASH的大小+0X08000000)



/*
**  STATUS Registers
*/
#define STATUS_00_REG         0x00
//
#define ZYXOW_BIT             Bit._7
#define ZOW_BIT               Bit._6
#define YOR_BIT               Bit._5
#define XOR_BIT               Bit._4
#define ZYXDR_BIT             Bit._3
#define ZDR_BIT               Bit._2
#define YDR_BIT               Bit._1
#define XDR_BIT               Bit._0
//
#define ZYXOW_MASK            0x80
#define ZOW_MASK              0x40
#define YOR_MASK              0x20
#define XOR_MASK              0x10
#define ZYXDR_MASK            0x08
#define ZDR_MASK              0x04
#define YDR_MASK              0x02
#define XDR_MASK              0x01

/*
**  XYZ Data Registers
*/
#define OUT_X_MSB_REG         0x01
#define OUT_X_LSB_REG         0x02
#define OUT_Y_MSB_REG         0x03
#define OUT_Y_LSB_REG         0x04
#define OUT_Z_MSB_REG         0x05
#define OUT_Z_LSB_REG         0x06   

/*
**  WHO_AM_I Device ID Register
*/
#define WHO_AM_I_REG          0x07
#define MAG3110Q_ID           0xC4    

/*
**  SYSMOD System Mode Register
*/
#define SYSMOD_REG            0x08
//   
#define SYSMOD1_BIT           Bit._1
#define SYSMOD0_BIT           Bit._0
// 
#define SYSMOD1_MASK          0x02
#define SYSMOD0_MASK          0x01
#define SYSMOD_MASK           0x03
#define STANDBY_MASK          0x00
#define ACTIVE_MASK           0x01
#define CORRECT_MASK          0x02

/*
**  INT_SOURCE System Interrupt Status Register
*/
#define INT_SOURCE_REG        0x0C
//
#define SRC_ASLP_BIT          Bit._7
#define SRC_FIFO_BIT          Bit._6
#define SRC_TRANS_BIT         Bit._5
#define SRC_LNDPRT_BIT        Bit._4
#define SRC_PULSE_BIT         Bit._3
#define SRC_FF_MT_1_BIT       Bit._2
#define SRC_FF_MT_2_BIT       Bit._1
#define SRC_DRDY_BIT          Bit._0
//
#define SRC_ASLP_MASK         0x80
#define SRC_FIFO_MASK         0x40
#define SRC_TRANS_MASK        0x20
#define SRC_LNDPRT_MASK       0x10
#define SRC_PULSE_MASK        0x08
#define SRC_FF_MT_1_MASK      0x04
#define SRC_FF_MT_2_MASK      0x02
#define SRC_DRDY_MASK         0x01   

/*
**  XYZ Offset Correction Registers
*/
#define OFF_X_MSB             0x09
#define OFF_X_LSB             0x0A
#define OFF_Y_MSB             0x0B 
#define OFF_Y_LSB             0x0C
#define OFF_Z_MSB             0x0D
#define OFF_Z_LSB             0x0E 

#define DIE_TEMP              0x0F 


/*
**  CTRL_REG1 System Control 1 Register
*/
#define CTRL_REG1             0x10
//
#define DR2_BIT               Bit._7
#define DR1_BIT               Bit._6
#define DR0_BIT               Bit._5
#define OS1_BIT               Bit._4
#define OS0_BIT               Bit._3
#define FR_BIT                Bit._2
#define TM_BIT                Bit._1
#define AC_BIT                Bit._0
//
#define DR2_MASK              0x80
#define DR1_MASK              0x40
#define DR0_MASK              0x20
#define OS1_MASK              0x10
#define OS0_MASK              0x08
#define FR_MASK               0x04
#define TM_MASK               0x02
#define AC_MASK               0x01

#define ASLP_RATE_MASK        0xC0
#define DR_MASK               0x38
//                      
#define ASLP_RATE_20MS        0x00
#define ASLP_RATE_80MS        ASLP_RATE0_MASK
#define ASLP_RATE_160MS       ASLP_RATE1_MASK
#define ASLP_RATE_640MS       ASLP_RATE1_MASK+ASLP_RATE0_MASK
//
#define DATA_RATE_1250US      0x00
#define DATA_RATE_2500US      DR0_MASK
#define DATA_RATE_5MS         DR1_MASK
#define DATA_RATE_10MS        DR1_MASK+DR0_MASK
#define DATA_RATE_20MS        DR2_MASK
#define DATA_RATE_80MS        DR2_MASK+DR0_MASK
#define DATA_RATE_160MS       DR2_MASK+DR1_MASK
#define DATA_RATE_640MS       DR2_MASK+DR1_MASK+DR0_MASK

/*
**  CTRL_REG2 System Control 2 Register
*/
#define CTRL_REG2             0x11
//
#define AUTO_MRST_EN_BIT      Bit._7
#define RAW_BIT               Bit._5
#define MAG_RST_BIT           Bit._4  
//
#define AUTO_MRST_EN_MASK     0x80 
#define RAW_MASK              0x20
#define MAG_RST_MASK          0x10

enum
{
  MAG3110_STATUS_00 = 0,          // 0x00
  MAG3110_OUT_X_MSB,              // 0x01
  MAG3110_OUT_X_LSB,              // 0x02
  MAG3110_OUT_Y_MSB,              // 0x03
  MAG3110_OUT_Y_LSB,              // 0x04
  MAG3110_OUT_Z_MSB,              // 0x05
  MAG3110_OUT_Z_LSB,              // 0x06 
  MAG3110_WHO_AM_I,               // 0x07
  MAG3110_SYSMOD,                 // 0x08
  MAG3110_OFF_X_MSB,              // 0x09
  MAG3110_OFF_X_LSB,              // 0x0A
  MAG3110_OFF_Y_MSB,              // 0x0B
  MAG3110_OFF_Y_LSB,              // 0x0C
  MAG3110_OFF_Z_MSB,              // 0x0D
  MAG3110_OFF_Z_LSB,              // 0x0E 
  MAG3110_DIE_TEMP,               // 0x0f
  MAG3110_CTRL_REG1,              // 0x10
  MAG3110_CTRL_REG2,              // 0x11
 
};



//参数定义
uint32_t MAG3110_XOFF=0,MAG3110_ZOFF=0,MAG3110_YOFF=0;
uint32_t MAG3110_XMax=0,MAG3110_YMax=0,MAG3110_XMin=0,MAG3110_YMin=0,MAG3110_ZMax=0,MAG3110_ZMin=0;
uint32_t MAG3110_XData=0,MAG3110_YData=0,MAG3110_ZData=0;
int ang;

 uint16_t VirtAddVarTabb[] = {0xAA00, 0xAA01, 0xAA02, 0xAA03, 0xAA04, 0xAA05, 0xAA06, 0xAA07, 0xAA08, 0xAA09, 
																		 0xAA0A,0xAA0B, 0xAA0C, 0xAA0D, 0xAA0E, 0xAA0F, 0xAA10, 0xAA11, 0xAA12, 0xAA13, 0xAA14, 
																			0xAA15, 0xAA16, 0xAA17, 0xAA18};


 






/*********************************************************\
* Put MAG3110Q into Active Mode
\*********************************************************/
void MAG3110_Active ()
{
  int n;
  n = IIC_Read_One_Byte_MAG(MAG3110_IIC_ADDRESS,CTRL_REG1);
  IIC_Write_One_Byte_MAG(MAG3110_IIC_ADDRESS,CTRL_REG1,(u8)n&0XFC|ACTIVE_MASK);
}

/*********************************************************\
* Put MAG3110Q into Standby Mode
\*********************************************************/
void MAG3110_Standby (void)
{
  int n;

  n = IIC_Read_One_Byte_MAG(MAG3110_IIC_ADDRESS,CTRL_REG1);
	IIC_Write_One_Byte_MAG(MAG3110_IIC_ADDRESS,CTRL_REG1, (u8)n&0xFC|STANDBY_MASK);
}

/*********************************************************\
* Initialize MAG3110Q
\*********************************************************/
void MAG3110_Init (void)
{  
  MAG3110_Standby();   
  IIC_Write_One_Byte_MAG(MAG3110_IIC_ADDRESS,CTRL_REG1, DATA_RATE_5MS);    
  MAG3110_Active();

}
//
uint32_t MAG3110_DataProcess (int MAG3110_XData,int MAG3110_YData,int MAG3110_ZData)
{


uint16_t MAG3110_Ang;

MAG3110_XData -= MAG3110_XOFF;
MAG3110_YData -= MAG3110_YOFF;
	
if (MAG3110_XData == 0)
{
if (MAG3110_YData>0)
{
MAG3110_Ang
= 90;
}
else
{
MAG3110_Ang
= 270;
}
}
else if (MAG3110_YData == 0)
{
if (MAG3110_XData>0)
{
MAG3110_Ang
= 0;
}
else
{
MAG3110_Ang
= 180;
}
}
else if ((MAG3110_XData > 0) && (MAG3110_YData > 0))
{
MAG3110_Ang = (atan ( ( (float)MAG3110_YData) / ( (float) MAG3110_XData ) ) )
* 180 / 3.14;
}
else if ((MAG3110_XData < 0) && (MAG3110_YData > 0))
{
MAG3110_XData = -MAG3110_XData;
MAG3110_Ang = 180
-
(atan ( ( (float)MAG3110_YData) / ( (float)
MAG3110_XData ) ) ) * 180 / 3.14;
}
else if ((MAG3110_XData < 0) && (MAG3110_YData < 0))
{
MAG3110_XData = -MAG3110_XData;
MAG3110_YData = -MAG3110_YData;
MAG3110_Ang = (atan ( ( (float)MAG3110_YData) / ( (float) MAG3110_XData ) ) )
* 180 / 3.14 + 180;
}
else if ((MAG3110_XData > 0) && (MAG3110_YData < 0))
{
MAG3110_YData = -MAG3110_YData;
MAG3110_Ang = 360
-
(atan ( ( (float)MAG3110_YData) / ( (float)
MAG3110_XData ) ) ) * 180 / 3.14;
}

return 	 MAG3110_Ang;
}


/*************************************************************************/
typedef union
{
  unsigned int mword;
  struct
  {
    unsigned char hi;
    unsigned char lo;
  } mbyte;
} tword; 

void MAG3110_STD( _st_Mag *pMag)
// 此函数需多次执行以保证旋转一圈中
{
// 能够采集到真实的最大值和最小值
	tword wx, wy, wz; 
	static   uint8_t	First_Flag=0;
	wx.mbyte.hi = IIC_Read_One_Byte_MAG(MAG3110_IIC_ADDRESS,OUT_X_MSB_REG); //读取X轴高字节
	wx.mbyte.lo = IIC_Read_One_Byte_MAG(MAG3110_IIC_ADDRESS,OUT_X_LSB_REG); //读取X轴低字节
	wy.mbyte.hi = IIC_Read_One_Byte_MAG(MAG3110_IIC_ADDRESS,OUT_Y_MSB_REG); //读取Y轴高字节
	wy.mbyte.lo = IIC_Read_One_Byte_MAG(MAG3110_IIC_ADDRESS,OUT_Y_LSB_REG); //读取Y轴低字节
	wz.mbyte.hi = IIC_Read_One_Byte_MAG(MAG3110_IIC_ADDRESS,OUT_Z_MSB_REG); //读取Z轴高字节
	wz.mbyte.lo = IIC_Read_One_Byte_MAG(MAG3110_IIC_ADDRESS,OUT_Z_LSB_REG); //读取Z轴低字节
	//printf("X:%u  ",wx.mbyte.hi*256+wx.mbyte.lo);
	//printf("Y:%u  ",wy.mbyte.hi*256+wy.mbyte.lo);
	//printf("Z:%d  ",wz.mbyte.hi*256+wz.mbyte.lo);
	
	
	MAG3110_XData=wx.mbyte.hi*256+wx.mbyte.lo;
	MAG3110_YData=wy.mbyte.hi*256+wy.mbyte.lo;
	MAG3110_ZData=wz.mbyte.hi*256+wz.mbyte.lo;
	
	//传出数据
	pMag->magX=MAG3110_XData;//IIC_Read_One_Byte_MAG(MAG3110_IIC_ADDRESS,OUT_X_MSB_REG); 
	pMag->magY=MAG3110_YData;
	pMag->magZ=MAG3110_ZData;
	
	
	if (!First_Flag)
	{
	MAG3110_XMax = MAG3110_XData;
	MAG3110_XMin = MAG3110_XData;
	MAG3110_YMax = MAG3110_YData;
	MAG3110_YMin = MAG3110_YData;
	MAG3110_ZMax = MAG3110_ZData;
	MAG3110_ZMin = MAG3110_ZData;
		
	First_Flag = 1;
	}
	if (MAG3110_XData > MAG3110_XMax)
	{
	MAG3110_XMax =  MAG3110_XData;
	}
	else if (MAG3110_XData < MAG3110_XMin)
	{
	MAG3110_XMin =  MAG3110_XData;
	}
	if (MAG3110_YData > MAG3110_YMax)
	{
	MAG3110_YMax =  MAG3110_YData;
	}
	else if (MAG3110_YData < MAG3110_YMin)
	{
	MAG3110_YMin =  MAG3110_YData;
	if (MAG3110_ZData > MAG3110_ZMax)
	{
	MAG3110_ZMax =  MAG3110_ZData;
	}
	else if (MAG3110_ZData < MAG3110_ZMin)
	{
	MAG3110_ZMin =  MAG3110_ZData;
	}
  }
	MAG3110_XOFF = (MAG3110_XMax + MAG3110_XMin) / 2;
	MAG3110_YOFF = (MAG3110_YMax + MAG3110_YMin) / 2;
	MAG3110_ZOFF = (MAG3110_ZMax + MAG3110_ZMin) / 2;
	
 /*   printf("\r\nMAG3110_XMax：%d ",MAG3110_XMax);
	printf("MAG3110_XMin：%d\r\n",MAG3110_XMin);
    printf("MAG3110_XOFF：%d\r\n",MAG3110_XOFF);

    printf("\r\nMAG3110_YMax：%d  ",MAG3110_YMax);
	printf("MAG3110_YMin：%d\r\n ",MAG3110_YMin);
    printf("AG3110_YOFF：%d\r\n",MAG3110_YOFF);*/

	ang=MAG3110_DataProcess(wx.mbyte.hi*256+wx.mbyte.lo,wy.mbyte.hi*256+wy.mbyte.lo,wz.mbyte.hi*256+wz.mbyte.lo);
}
//获取z轴磁力计信息
uint32_t get_z_mag(void)
{ tword  wz;
	uint32_t MAG3110_ZData;
	wz.mbyte.hi = IIC_Read_One_Byte_MAG(MAG3110_IIC_ADDRESS,OUT_Z_MSB_REG); //读取Z轴高字节
	wz.mbyte.lo = IIC_Read_One_Byte_MAG(MAG3110_IIC_ADDRESS,OUT_Z_LSB_REG); //读取Z轴低字节
	
	MAG3110_ZData=wz.mbyte.hi*256+wz.mbyte.lo;
	

	
return MAG3110_ZData;
}	
//获得标准z轴信息
int set_std_zmag()
{ 
	int have_std,zo,zn;
	u16 eeprom_first_flag=0;
	STMFLASH_Read(VirtAddVarTabb[0],&eeprom_first_flag,4);
	if(ALL_flag.flight_mode==0)//改为如按下取样键
	{
	
		zo= (u16)get_z_mag();
		 delay_ms(2);
		zn=(u16)get_z_mag();
		if(zn==zo){STMFLASH_Write(VirtAddVarTabb[1],(u16*)zn,4);}
		else if(zn>zo){STMFLASH_Write(VirtAddVarTabb[1],(u16*)zn,4);}
		else if(zo>zn){STMFLASH_Write(VirtAddVarTabb[1],(u16*)zo,4);}
			STMFLASH_Write(VirtAddVarTabb[1],(u16*)1,4);;
		have_std=1;return have_std;
		}
	if(eeprom_first_flag==0){have_std=0;return have_std;}
	if (eeprom_first_flag==1){have_std=1;return have_std;}
	return have_std;
	 
}

