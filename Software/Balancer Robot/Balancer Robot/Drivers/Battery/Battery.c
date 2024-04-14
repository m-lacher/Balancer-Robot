//------------------------------------------------------
//File:			Battery.c
//Project:	Balancer Robot
//Created:	19.09.2019
//Autor:		M.Lacher
//------------------------------------------------------

#include "Battery.h"
#include "TimingHandler.h"
#include <stdlib.h>

#define BUFFER_SIZE 10
#define MIN_VOLTAGE 10.8	//4 * 2.7V 

ADC_HandleTypeDef ADC;

static void MX_ADC1_Init(void);
static void EmptyFunctionPointer(void);
static void ADCRoutine(void);

static void (*callbackPointer)(void) = NULL;
static TimerHandle *ADConversionTimer = NULL;
static uint16_t dataBuffer[BUFFER_SIZE];
static uint16_t dataIndex = 0;
static float batteryVoltage = 0;

void Battery_InitModule()
{
	MX_ADC1_Init();
	HAL_ADC_Start_IT(&ADC);
	ADConversionTimer = TimingHandler_CreatePeriodicTimer(ADCRoutine);
	TimingHandler_StartTimer(ADConversionTimer, 100);
}

extern void Battery_SetBatteryLowInterruptPointer(void (*interruptPointer)(void))
{
	callbackPointer = interruptPointer;
}

extern float Battery_GetVoltageLevel()
{
	return batteryVoltage;
}

void MX_ADC1_Init(void)
{

  ADC_MultiModeTypeDef multimode;
  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  ADC.Instance = ADC1;
  ADC.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  ADC.Init.Resolution = ADC_RESOLUTION_12B;
  ADC.Init.ScanConvMode = ADC_SCAN_DISABLE;
  ADC.Init.ContinuousConvMode = DISABLE;
  ADC.Init.DiscontinuousConvMode = DISABLE;
  ADC.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  ADC.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  ADC.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  ADC.Init.NbrOfConversion = 1;
  ADC.Init.DMAContinuousRequests = DISABLE;
  ADC.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  ADC.Init.LowPowerAutoWait = DISABLE;
  ADC.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&ADC) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the ADC multi-mode 
    */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&ADC, &multimode) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&ADC, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void ADCRoutine()
{
	int index;
	uint16_t total = 0;
	float voltage;
	for(index=0; index<BUFFER_SIZE; index++)
	{
		total += dataBuffer[index];
	}
	voltage = total;
	voltage /= BUFFER_SIZE;
	voltage /= 4096;
	voltage *= 14.2;
	batteryVoltage = voltage;
	if(voltage <= MIN_VOLTAGE && callbackPointer != NULL)
	{
		callbackPointer();
	}
	HAL_ADC_Stop_IT(&ADC);
	HAL_ADC_Start_IT(&ADC);
	HAL_ADC_PollForConversion(&ADC, 10);
}

//TBD, funktioniert noch nicht
void ADC1_2_IRQHandler(void)
{
	HAL_ADC_IRQHandler(&ADC);
	dataBuffer[dataIndex] = HAL_ADC_GetValue(&ADC);
	dataIndex++;
	if(dataIndex >= BUFFER_SIZE)
	{
		dataIndex = 0;
	}
}
