//------------------------------------------------------
//File:			Motors.c
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

#include "Motors.h"

//PWM1 = TIM2_CH4
//PWM2 = TIM3_CH2
//PWM3 = TIM2_CH1
//PWM4 = TIM3_CH1

#define PWM1_Timer 	timer2
#define PWM2_Timer 	timer3
#define PWM3_Timer 	timer3
#define PWM4_Timer 	timer17


#define PWM1_CH			TIM_CHANNEL_4
#define PWM2_CH			TIM_CHANNEL_2
#define PWM3_CH			TIM_CHANNEL_1
#define PWM4_CH			TIM_CHANNEL_1


#define MAX_VALUE 1000
#define MIN_VALUE -1000
#define OFFSET_VALUE 0

TIM_HandleTypeDef timer2;
TIM_HandleTypeDef timer3;
TIM_HandleTypeDef timer17;

static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM17_Init(void);

void Motors_InitModule()
{
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM17_Init();
	HAL_TIM_PWM_Start(&PWM1_Timer, PWM1_CH);
	HAL_TIM_PWM_Start(&PWM2_Timer, PWM2_CH);
	HAL_TIM_PWM_Start(&PWM3_Timer, PWM3_CH);
	HAL_TIM_PWM_Start(&PWM4_Timer, PWM4_CH);
	Motors_Enable();
}

void Motors_SetValue(E_Motors motor, int16_t value)
{
	if(value > MAX_VALUE)																				//Adjust value to be in Min-Max range
	{
		value = MAX_VALUE;
	}
	else if(value < MIN_VALUE)
	{
		value = MIN_VALUE;
	}
	switch(motor)
	{
		case Motor_Left:
			if(value > 0)																						//PWM1 on, PWM2 off
			{
				__HAL_TIM_SET_COMPARE(&PWM2_Timer, PWM2_CH, 0);				//Set CCR PWM2 to 0
				__HAL_TIM_SET_COMPARE(&PWM1_Timer, PWM1_CH, value + OFFSET_VALUE);		//Set CCR PWM1 to value
			}
			else if(value < 0)																			//PWM2 on, PWM1 off
			{
				__HAL_TIM_SET_COMPARE(&PWM1_Timer, PWM1_CH, 0);				//Set CCR PWM1 to 0
				__HAL_TIM_SET_COMPARE(&PWM2_Timer, PWM2_CH, value * (-1) + OFFSET_VALUE);		//Set CCR PWM2 to value
			}
			else																										//value = 0, turn off both PWM
			{
				__HAL_TIM_SET_COMPARE(&PWM2_Timer, PWM2_CH, 0);				//Set CCR PWM2 to 0
				__HAL_TIM_SET_COMPARE(&PWM1_Timer, PWM1_CH, 0);				//Set CCR PWM1 to 0
			}
			break;
		case Motor_Right:
			if(value > 0)																						//PWM1 on, PWM2 off
			{
				__HAL_TIM_SET_COMPARE(&PWM4_Timer, PWM4_CH, 0);				//Set CCR PWM4 to 0
				__HAL_TIM_SET_COMPARE(&PWM3_Timer, PWM3_CH, value + OFFSET_VALUE);		//Set CCR PWM3 to value
			}
			else if(value < 0)																			//PWM4 on, PWM3 off
			{
				__HAL_TIM_SET_COMPARE(&PWM3_Timer, PWM3_CH, 0);				//Set CCR PWM3 to 0
				__HAL_TIM_SET_COMPARE(&PWM4_Timer, PWM4_CH, value *(-1) + OFFSET_VALUE);		//Set CCR PWM4 to value
			}
			else																										//value = 0, turn off both PWM
			{
				__HAL_TIM_SET_COMPARE(&PWM4_Timer, PWM4_CH, 0);				//Set CCR PWM4 to 0
				__HAL_TIM_SET_COMPARE(&PWM3_Timer, PWM3_CH, 0);				//Set CCR PWM3 to 0
			}
			break;
		default:
			break;
	}
}

void Motors_Enable(void)
{
	HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_SET);
}
void Motors_Disable(void)
{
	HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_RESET);
}

void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  timer2.Instance = TIM2;
  timer2.Init.Prescaler = 0;
  timer2.Init.CounterMode = TIM_COUNTERMODE_UP;
  timer2.Init.Period = MAX_VALUE;
  timer2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  timer2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&timer2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&timer2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&timer2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&timer2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&timer2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&timer2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&timer2);

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  timer3.Instance = TIM3;
  timer3.Init.Prescaler = 0;
  timer3.Init.CounterMode = TIM_COUNTERMODE_UP;
  timer3.Init.Period = MAX_VALUE;
  timer3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  timer3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&timer3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&timer3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&timer3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&timer3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&timer3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&timer3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  HAL_TIM_MspPostInit(&timer3);
}

static void MX_TIM17_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  timer17.Instance = TIM17;
  timer17.Init.Prescaler = 0;
  timer17.Init.CounterMode = TIM_COUNTERMODE_UP;
  timer17.Init.Period = MAX_VALUE;
  timer17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  timer17.Init.RepetitionCounter = 0;
  timer17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&timer17) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&timer17) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&timer17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&timer17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&timer17);

}