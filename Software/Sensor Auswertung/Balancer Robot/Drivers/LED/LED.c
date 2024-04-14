//------------------------------------------------------
//File:			LED.c
//Project:	Balancer Robot
//Created:	19.09.2019
//Autor:		M.Lacher
//------------------------------------------------------

/*
This module controls the onboard Error and Status LED.
The GPIO module is used to control the two responsible IO's
*/

#include "LED.h"
#include "TimingHandler.h"

#define MAX_TICK_COUNT 30

TimerHandle *StateMachineTimer;
E_LEDState errorState = LED_OFF;
E_LEDState modeState = LED_OFF;
uint16_t tickCount = 0;

void StateMachineRoutine(void);

void LED_InitModule()
{
	StateMachineTimer = TimingHandler_CreatePeriodicTimer(StateMachineRoutine);
	TimingHandler_StartTimer(StateMachineTimer, 100);
	HAL_GPIO_WritePin(LED_Error_GPIO_Port, LED_Error_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_Mode_GPIO_Port, LED_Mode_Pin, GPIO_PIN_RESET);
}

void LED_SetErrorState(E_LEDState state)
{
	errorState = state;
}

void LED_SetModeState(E_LEDState state)
{
	modeState = state;
}

void StateMachineRoutine()
{
	tickCount++;
	if(tickCount >= MAX_TICK_COUNT)
	{
		tickCount = 0;
	}
	switch(errorState)
	{
		case LED_OFF:
			HAL_GPIO_WritePin(LED_Error_GPIO_Port, LED_Error_Pin, GPIO_PIN_RESET);
			break;
		
		case LED_ON:
			HAL_GPIO_WritePin(LED_Error_GPIO_Port, LED_Error_Pin, GPIO_PIN_SET);
			break;
		
		case LED_FLASHING_2:
			if(tickCount == 0 || tickCount == 4)
			{
				HAL_GPIO_WritePin(LED_Error_GPIO_Port, LED_Error_Pin, GPIO_PIN_SET);
			}
			if(tickCount == 2 || tickCount == 6)
			{
				HAL_GPIO_WritePin(LED_Error_GPIO_Port, LED_Error_Pin, GPIO_PIN_RESET);
			}
			break;
			
		case LED_FLASHING_4:
			if(tickCount == 0 || tickCount == 4 || tickCount == 8 || tickCount == 12)
			{
				HAL_GPIO_WritePin(LED_Error_GPIO_Port, LED_Error_Pin, GPIO_PIN_SET);
			}
			if(tickCount == 2 || tickCount == 6 || tickCount == 10 || tickCount == 14)
			{
				HAL_GPIO_WritePin(LED_Error_GPIO_Port, LED_Error_Pin, GPIO_PIN_RESET);
			}
			break;
		
		case LED_BLINKING_1HZ:
			if(tickCount == 0 || tickCount == 10 || tickCount == 20)
			{
				HAL_GPIO_WritePin(LED_Error_GPIO_Port, LED_Error_Pin, GPIO_PIN_SET);
			}
			if(tickCount == 5 || tickCount == 15 || tickCount == 25)
			{
				HAL_GPIO_WritePin(LED_Error_GPIO_Port, LED_Error_Pin, GPIO_PIN_RESET);
			}
			break;
			
		case LED_BLINKING_5HZ:
			if(tickCount % 2 == 0)
			{
				HAL_GPIO_WritePin(LED_Error_GPIO_Port, LED_Error_Pin, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(LED_Error_GPIO_Port, LED_Error_Pin, GPIO_PIN_RESET);
			}
			break;
		
		default:
			break;
	}
	
	switch(modeState)
	{
		case LED_OFF:
			HAL_GPIO_WritePin(LED_Mode_GPIO_Port, LED_Mode_Pin, GPIO_PIN_RESET);
			break;
		
		case LED_ON:
			HAL_GPIO_WritePin(LED_Mode_GPIO_Port, LED_Mode_Pin, GPIO_PIN_SET);
			break;
		
		case LED_FLASHING_2:
			if(tickCount == 0 || tickCount == 4)
			{
				HAL_GPIO_WritePin(LED_Mode_GPIO_Port, LED_Mode_Pin, GPIO_PIN_SET);
			}
			if(tickCount == 2 || tickCount == 6)
			{
				HAL_GPIO_WritePin(LED_Mode_GPIO_Port, LED_Mode_Pin, GPIO_PIN_RESET);
			}
			break;
			
		case LED_FLASHING_4:
			if(tickCount == 0 || tickCount == 4 || tickCount == 8 || tickCount == 12)
			{
				HAL_GPIO_WritePin(LED_Mode_GPIO_Port, LED_Mode_Pin, GPIO_PIN_SET);
			}
			if(tickCount == 2 || tickCount == 6 || tickCount == 10 || tickCount == 14)
			{
				HAL_GPIO_WritePin(LED_Mode_GPIO_Port, LED_Mode_Pin, GPIO_PIN_RESET);
			}
			break;
		
		case LED_BLINKING_1HZ:
			if(tickCount == 0 || tickCount == 10 || tickCount == 20)
			{
				HAL_GPIO_WritePin(LED_Mode_GPIO_Port, LED_Mode_Pin, GPIO_PIN_SET);
			}
			if(tickCount == 5 || tickCount == 15 || tickCount == 25)
			{
				HAL_GPIO_WritePin(LED_Mode_GPIO_Port, LED_Mode_Pin, GPIO_PIN_RESET);
			}
			break;
			
		case LED_BLINKING_5HZ:
			if(tickCount % 2 == 0)
			{
				HAL_GPIO_WritePin(LED_Mode_GPIO_Port, LED_Mode_Pin, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(LED_Mode_GPIO_Port, LED_Mode_Pin, GPIO_PIN_RESET);
			}
			break;
		
		default:
			break;
	}
}
