//------------------------------------------------------
//File:			Battery.h
//Project:	Balancer Robot
//Created:	19.09.2019
//Autor:		M.Lacher
//------------------------------------------------------

#include "stm32f3xx_hal.h"

#ifndef Battery
#define Battery

extern void Battery_InitModule(void);
extern void Battery_SetBatteryLowInterruptPointer(void (*interruptPointer)(void));
extern float Battery_GetVoltageLevel(void);


#endif
