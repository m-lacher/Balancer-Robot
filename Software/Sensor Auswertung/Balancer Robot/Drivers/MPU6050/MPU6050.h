//------------------------------------------------------
//File:			MPU6050.h
//Project:	Balancer Robot
//Created:	20.09.2019
//Autor:		M.Lacher
//------------------------------------------------------

/*
This module is used to Operate with the MPU-6050 Sensor.
It provides funtions to configure the sensor and reading the measurements.
This module is using the I2C.h Interface to communicate with the Sensor.
*/

#include "stm32f3xx_hal.h"
#include "I2C.h"

#ifndef MPU6050
#define MPU6050

#define GYRO_DPS_250			(0 << 3)
#define GYRO_DPS_500			(1 << 3)
#define GYRO_DPS_1000			(2 << 3)
#define GYRO_DPS_2000			(3 << 3)

#define ACCEL_G_2					(0 << 3)
#define ACCEL_G_4					(1 << 3)
#define ACCEL_G_8					(2 << 3)
#define ACCEL_G_16				(3 << 3)

#define CLKSEL_INTERNAL_OSCILLATOR	(0 << 0)
#define CLKSEL_PLL_X_AXIS_GYRO			(1 << 0)
#define CLKSEL_PLL_Y_AXIS_GYRO			(2 << 0)
#define CLKSEL_PLL_Z_AXIS_GYRO			(3 << 0)
#define CLKSEL_PLL_EXT_32kHZ				(4 << 0)
#define CLKSEL_PLL_EXT_19MHZ				(5 << 0)
#define CLKSEL_STOPPED							(7 << 0)

#define OPMODE_STANDARD		(0 << 5)
#define OPMODE_CYCLE			(1 << 5)
#define OPMODE_SLEEP			(2 << 5)

#define AXIS_ACCEL_X			(1 << 5)
#define AXIS_ACCEL_Y			(1 << 4)
#define AXIS_ACCEL_Z			(1 << 3)
#define AXIS_GYRO_X				(1 << 2)
#define AXIS_GYRO_Y				(1 << 1)
#define AXIS_GYRO_Z				(1 << 0)

#define INT_FIFO_OVERFLOW (1 << 4)
#define INT_I2C_MST				(1 << 3)
#define INT_DATA_READY		(1 << 0)

#define BANDWIDTH_260_HZ 	(0 << 0)
#define	BANDWIDTH_184_HZ	(1 << 0)
#define BANDWIDTH_94_HZ		(2 << 0)
#define BANDWIDTH_44_HZ		(3 << 0)
#define BANDWIDTH_21_HZ		(4 << 0)
#define BANDWIDTH_10_HZ		(5 << 0)
#define BANDWIDTH_5_HZ		(6 << 0)
typedef enum
{
	EXT_0 = 0,
	EXT_1,
	EXT_2,
	ACCEL,
	GYRO_Z,
	GYRO_Y,
	GYRO_X,
	TEMP
}E_MPU6050_Sensors;

typedef enum
{
	INTERNAL_OSCILLATOR,
	PLL_X_AXIS_GYRO,
	PLL_Y_AXIS_GYRO,
	PLL_Z_AXIS_GYRO,
	PLL_EXT_1,
	PLL_EXT_2,
	STOPPED = 7
}E_MPU6050_ClockSource;

typedef enum 
{
	STANDARD,
	CYCLE,
	SLEEP
}E_MPU6050_OperatingMode;

typedef struct
{
	uint8_t	ClockDiv;
	uint8_t ClockSource;
	uint8_t OperatingMode;
}MPU6050_InitTypeDef;

typedef struct
{
	uint8_t FullScaleRange;
}MPU6050_SensorInitTypeDef;

typedef struct
{
	int16_t GYRO_X;
	int16_t GYRO_Y;
	int16_t GYRO_Z;
	int16_t ACCEL_X;
	int16_t ACCEL_Y;
	int16_t ACCEL_Z;
	int16_t TEMP;
}MPU6050_S_Measurements;


extern void MPU6050_InitModule(void);
extern void MPU6050_InitDevice(MPU6050_InitTypeDef *initStruct);
extern void MPU6050_InitGyro(MPU6050_SensorInitTypeDef *initStruct, uint8_t axis);
extern void MPU6050_InitAccel(MPU6050_SensorInitTypeDef *initStruct, uint8_t axis);
extern void MPU6050_SetDigitalLowPassFilter(uint8_t bandwidth);
extern void MPU6050_EnableInterrupt(uint8_t interrupt);
extern void MPU6050_DisableInterrupt(uint8_t interrupt);
extern void MPU6050_ReadValues(MPU6050_S_Measurements *sensorValues);
extern void MPU6050_SetInterruptCallback(void (*callbackFunctionPointer)(void));
extern void MPU6050_ClearInterruptCallback(void);


//TBD
extern void MPU6050_SetSampleRate(uint16_t sampleRate);
extern void MPU6050_SetClockSource(E_MPU6050_ClockSource);
extern void MPU6050_AddToFIFO(E_MPU6050_Sensors sensor);
extern void MPU6050_RemoveFromFIFO(E_MPU6050_Sensors sensor);


#endif
