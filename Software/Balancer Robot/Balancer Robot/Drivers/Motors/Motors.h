//------------------------------------------------------
//File:			Motors.h
//Project:	Balancer Robot
//Created:	19.09.2019
//Autor:		M.Lacher
//------------------------------------------------------

/*
This module controls the 2 motors of the Balancer Robot.
4 Pins are configured as PWM to control the motors speed.
With the function Motors_SetValue, a value between -1000 and 1000 
can be given to both motors individually.
*/

#include "stm32f3xx_hal.h"

#ifndef Motors
#define Motors

typedef enum
{
	Motor_Left = 0,
	Motor_Right = 1
}E_Motors;

extern void Motors_InitModule(void);
extern void Motors_SetValue(E_Motors motor, int16_t value);
extern void Motors_Enable(void);
extern void Motors_Disable(void);

#endif
