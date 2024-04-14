//------------------------------------------------------
//File:			TimingHandler.c
//Project:	BalancerRobot
//Created:	03.10.2019
//Autor:		M.Lacher
//------------------------------------------------------

#include "TimingHandler.h"
#include "stm32f3xx_hal.h"

TIM_HandleTypeDef Timer6;
static TimerHandle timerHandles[NumberOfPossibleHandles];	//Buffer list for timer handles. Number of possible handles defined in header file
static uint64_t sysTick = 0;

void TimerRoutine(void);
TimerHandle *PlaceHandleInList(TimerHandle handle);

//*******************************************************************************
//Public Functions

void TimingHandler_InitModule()
{
	TIM_MasterConfigTypeDef sMasterConfig;
  Timer6.Instance = TIM6;
  Timer6.Init.Prescaler = 8;
  Timer6.Init.CounterMode = TIM_COUNTERMODE_UP;
  Timer6.Init.Period = 1000;
  Timer6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&Timer6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&Timer6, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	HAL_TIM_Base_Init(&Timer6);
	HAL_TIM_Base_Start_IT(&Timer6);
}

TimerHandle *TimingHandler_CreatePeriodicTimer(void (*callbackFunction)(void))
{
	TimerHandle handle;
	handle.isPeriodic = true;
	handle.isUsed = true;
	handle.callbackFunction = callbackFunction;
	handle.count = 0;
	return PlaceHandleInList(handle);
}
TimerHandle *TimingHandler_CreateSingleShotTimer(void (*callbackFunction)(void))
{
	TimerHandle handle;
	handle.isPeriodic = false;
	handle.isUsed = true;
	handle.callbackFunction = callbackFunction;
	handle.count = 0;
	return PlaceHandleInList(handle);
}
void TimingHandler_StartTimer(TimerHandle *handle, uint16_t period)
{
	handle->period = period;
	handle->isRunning = true;
}
void TimingHandler_StopTimer(TimerHandle *handle)
{
	handle->isRunning = false;
}
void TimingHandler_KillTimer(TimerHandle *handle)
{
	handle->isRunning = false;
	handle->isUsed = false;
}
bool TimingHandler_IsTimerRunning(TimerHandle *handle)
{
	return handle->isRunning;
}

uint64_t TimingHandler_GetSysTick()
{
	return sysTick;
}

//*******************************************************************************
//Private Functions

void TIM6_DAC1_IRQHandler(void)	//Used for TiminRoutine 1ms
{
	HAL_TIM_IRQHandler(&Timer6);
	TimerRoutine();
}

void TimerRoutine()
{
	int index;
	for(index = 0; index < NumberOfPossibleHandles; index++)				//Iterate over all possible timer slots 
	{
		if(timerHandles[index].isRunning)															//Check if timer is running
		{
			timerHandles[index].count++;																//Update the count value
			if(timerHandles[index].count >= timerHandles[index].period) //Count value higher as period -> Timer expired
			{
				timerHandles[index].count = 0;														//Reset count value
				timerHandles[index].callbackFunction();										//Execute callback function
				if(timerHandles[index].isPeriodic == false)								//Kill timer if not periodic
				{
					TimingHandler_KillTimer(&timerHandles[index]);
				}
			}
		}
	}
	//sysTick++;
}

TimerHandle *PlaceHandleInList(TimerHandle handle)
{
	int index;
	for(index = 0; index < NumberOfPossibleHandles; index++)
	{
		if(timerHandles[index].isUsed == false)
		{
			timerHandles[index] = handle;
			return &timerHandles[index];
		}
	}
	return &timerHandles[index];
}
