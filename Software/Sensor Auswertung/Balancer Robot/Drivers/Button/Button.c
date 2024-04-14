//------------------------------------------------------
//File:			Button.c
//Project:	Balancer Robot
//Created:	04.10.2019
//Autor:		M.Lacher
//------------------------------------------------------

#include "Button.h"
#include "TimingHandler.h"
#include <stdlib.h>
#include "GPIO.h"

#define ButtonPeriod 100

void CheckRoutine(void);

void(*ButtonPushedCB)(void) = NULL;
TimerHandle *routineTimer = NULL;

void Button_InitModule(void)
{
	routineTimer = TimingHandler_CreatePeriodicTimer(CheckRoutine);
	TimingHandler_StartTimer(routineTimer, ButtonPeriod); 
}

void Button_SetButtonPushedCallback(void(*ButtonPushedCallback)(void))
{
	ButtonPushedCB = ButtonPushedCallback;
}

void CheckRoutine(void)
{
	static bool buttonOld = false;
	bool button = false;
	if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(Button_GPIO_Port, Button_Pin))
	{
		button = true;
	}
	if(buttonOld == false && button == true && ButtonPushedCB != NULL)			//Negative edge detection
	{
		ButtonPushedCB();
	}
	buttonOld = button;	
}