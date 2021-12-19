#ifndef _MEGNETOMETER_H__
#define _MEGNETOMETER_H__
#endif

#include "stm32f10x.h"
#include "IIC_MAG.h"
#include "math.h"
//#include "fish_Drv_EEPROM.h"
#include "delayy.h"
#include "stmflash.h"
#include "ALL_DATA.h"








void MAG3110_Active (void);
void MAG3110_Standby (void);
void MAG3110_Init (void);
uint32_t MAG3110_DataProcess (int MAG3110_XData,int MAG3110_YData,int MAG3110_ZData);
void MAG3110_STD( _st_Mag *pMag);
uint32_t get_z_mag(void);
int set_std_zmag(void);

