//------------------------------------------------------
//File:			ErrorHandler.c
//Project:	Balancer Robot
//Created:	03.10.2019
//Autor:		M.Lacher
//------------------------------------------------------

#include "ErrorHandler.h"
#include "LED.h"

void ErrorHandler_InitModule(void)
{
	
}

void ErrorHandler_HardFault(void)
{
	LED_SetErrorState(LED_BLINKING_5HZ);
	while(1)
	{
		
	}
}

void ErrorHandler_SetWarning(void)
{
	LED_SetErrorState(LED_BLINKING_1HZ);
}

void ErrorHandler_ResetWarning(void)
{
	LED_SetErrorState(LED_OFF);
}

