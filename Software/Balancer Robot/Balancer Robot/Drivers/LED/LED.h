//------------------------------------------------------
//File:			LED.h
//Project:	Balancer Robot
//Created:	19.09.2019
//Autor:		M.Lacher
//------------------------------------------------------

/*
This module controls the onboard Error and Status LED.
The GPIO module is used to control the two responsible IO's
*/

#include "stm32f3xx_hal.h"

#ifndef LED
#define LED

typedef enum
{
	LED_OFF = 0,
	LED_ON = 1,
	LED_FLASHING_2,
	LED_FLASHING_4,
	LED_BLINKING_1HZ,
	LED_BLINKING_5HZ
}E_LEDState;

extern void LED_InitModule(void);
extern void LED_SetErrorState(E_LEDState state);
extern void LED_SetModeState(E_LEDState state);

#endif
