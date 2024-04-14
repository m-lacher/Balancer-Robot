//------------------------------------------------------
//File:			TimingHandler.h
//Project:	BalancerRobot
//Created:	03.10.2019
//Autor:		M.Lacher
//------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>

#ifndef TimingHandler
#define TimingHandler

#define NumberOfPossibleHandles 20

typedef struct
{
	bool isUsed;											//false means, that the struct isn't used anymore. The TimingHandler only overrides structures with this parameter set to false;
	bool isRunning;										//Indicates if the timerHandle is running
	uint16_t count;
	uint16_t period;
	bool isPeriodic;
	void (*callbackFunction)(void);		//Pointer to callback Function
}TimerHandle;
	
extern void TimingHandler_InitModule(void);
extern TimerHandle *TimingHandler_CreatePeriodicTimer(void (*callbackFunction)(void));
extern TimerHandle *TimingHandler_CreateSingleShotTimer(void (*callbackFunction)(void));
extern void TimingHandler_StartTimer(TimerHandle *handle, uint16_t period);
extern void TimingHandler_StopTimer(TimerHandle *handle);
extern void TimingHandler_KillTimer(TimerHandle *handle);
extern bool TimingHandler_IsTimerRunning(TimerHandle *handle);
extern uint64_t TimingHandler_GetSysTick(void);

#endif
