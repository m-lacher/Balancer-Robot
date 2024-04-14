//------------------------------------------------------
//File:			main.c
//Project:	Balancer Robot
//Created:	19.09.2019
//Autor:		M.Lacher
//------------------------------------------------------

#include "main.h"
#include "stm32f3xx_hal.h"
#include <stdbool.h>

#include "System.h"
#include "GPIO.h"
#include "LED.h"
#include "Motors.h"
#include "Battery.h"
#include "I2C.h"
#include "MPU6050.h"
#include "Controller.h"
#include "TimingHandler.h"
#include "ErrorHandler.h"
#include "Button.h"
#include "DataProcessor.h"


void MPU6050DataReadyCallback(void);
void MODERoutine(void);
void ButtonRoutine(void);

bool MPU6050InterruptFlag = false;
bool motorsRunningFlag = false;

int main(void)
{
	//Local variables
	int index = 0;
	bool direction = false;
	motorsRunningFlag = false;
	
	//Intialize all Modules
	System_Initialize();
	ErrorHandler_InitModule();
	TimingHandler_InitModule();
	GPIO_InitModule();
	LED_InitModule();
	Button_InitModule();
	Motors_InitModule();	
	Battery_InitModule();
	I2C_InitModule();
	MPU6050_InitModule();
	DataProcessor_InitModule();
	Controller_InitModule();
	
	//Initialize Button Callback Function
	Button_SetButtonPushedCallback(ButtonRoutine);
	TimingHandler_StartTimer(TimingHandler_CreatePeriodicTimer(MODERoutine), 500);
	LED_SetModeState(LED_OFF);
	
	//Initialize Sensor
	MPU6050_InitTypeDef initStruct;
	MPU6050_SensorInitTypeDef accelerometer;
	MPU6050_SensorInitTypeDef gyro;
	
	initStruct.ClockDiv = 80;																				// 8kHz / 80 = 100Hz
	initStruct.ClockSource = CLKSEL_INTERNAL_OSCILLATOR;						//Use internal oszillator as clock source
	initStruct.OperatingMode = OPMODE_STANDARD;											//Standard operation mode
	
	accelerometer.FullScaleRange = ACCEL_G_4;												//Accelerometer range +/- 4G
	gyro.FullScaleRange = GYRO_DPS_500;															//Gyro range +/- 500 �/s
	
	MPU6050_InitDevice(&initStruct);													
	MPU6050_InitGyro(&gyro, AXIS_GYRO_X | AXIS_GYRO_Y | AXIS_GYRO_Z);
	MPU6050_InitAccel(&accelerometer, AXIS_ACCEL_X | AXIS_ACCEL_Y | AXIS_ACCEL_Z);
	MPU6050_SetDigitalLowPassFilter(BANDWIDTH_10_HZ);
	MPU6050_EnableInterrupt(INT_DATA_READY);
	MPU6050_SetInterruptCallback(MPU6050DataReadyCallback);
	

	//Main Loop
  while (1)
  {		
		if(MPU6050InterruptFlag == true)															//Data ready?
		{	
			MPU6050_S_Measurements values, processedValues;
			MPU6050_ReadValues(&values);
			processedValues = DataProcessor_Process(values);						//Process Values
			int16_t motorValue = Controller_Process(values);		//Calculate Motor Values (Controller Module)
			if(motorsRunningFlag == true)																//Motors Enabled?
			{
				Motors_SetValue(Motor_Left, motorValue);									//Control left motor
				Motors_SetValue(Motor_Right, motorValue);									//Control right motor
			}
			else
			{
				Motors_SetValue(Motor_Left, 0);														//Turn off left motor
				Motors_SetValue(Motor_Right, 0);													//Turn off right motor
			}
			MPU6050InterruptFlag = false;
		}
	}
}

void MPU6050DataReadyCallback(void)																//Callback from MPU6050 Hardware Interrupt
{
	MPU6050InterruptFlag = true;
}

void MODERoutine()
{
	
}

void ButtonRoutine()																							//Button Routine, switches motor enalbed state
{
	if(motorsRunningFlag == false)
	{
		LED_SetModeState(LED_ON);
		motorsRunningFlag = true;
	}
	else
	{
		LED_SetModeState(LED_OFF);
		motorsRunningFlag = false;
	}
}

void _Error_Handler(char *file, int line)													//Error Handling
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
		LED_SetErrorState(LED_ON);
		HAL_Delay(100);
		LED_SetErrorState(LED_OFF);
		HAL_Delay(100);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/