//------------------------------------------------------
//File:			Controller.c
//Project:	Balancer Robot
//Created:	27.09.2019
//Autor:		M.Lacher
//------------------------------------------------------

#include "Controller.h"
#include <math.h>

#define BUFFER_SIZE 10
#define P_FACTOR 1000
#define I_FACTOR 0
#define D_FACTOR 0
#define REACTION_VALUE 4000																											//1422 equals 10°, when sensor max is 4G
#define SENSOR_AXIS_DISTANCE	0.2
#define SENSOR_TIME_SLICE 0.01
#define Z_MAX_VALUE 0.93
#define START_VALUE 0.25
#define STOP_VALUE 0.15

typedef enum
{
	None = 0,
	Negativ = -1,
	Positiv = 1
}Direction;

int16_t setPoint_z = 0;
static MPU6050_S_Measurements dataBuffer[BUFFER_SIZE];

void Buffer_Add(MPU6050_S_Measurements value);

void Controller_InitModule()
{
	
}
void Controller_SetPoint(uint16_t setPoint)
{
	setPoint_z = setPoint;
}

Direction getDirection(MPU6050_S_Measurements *values)
{
	if(values->ACCEL_Y > 0)
	{
		return Positiv;
	}
	return Negativ;
}

extern int16_t Controller_Process(MPU6050_S_Measurements values)
{
	static Direction direction = None;
	static float forceY_old = 0;
	int16_t returnValue;
	float forceZ = values.ACCEL_Z;
	forceZ /= 32767;
	forceZ *= 4;
	float forceY = sqrt(((Z_MAX_VALUE * Z_MAX_VALUE)-(forceZ * forceZ)));
	if(forceY > START_VALUE && forceY_old <= START_VALUE)
	{
		direction = getDirection(&values);
	}
	else if(forceY < STOP_VALUE && forceY_old >= STOP_VALUE)
	{
		direction = None;
	}
	forceY_old = forceY;
	returnValue = (int16_t)(forceY * P_FACTOR * direction);
	return returnValue;
}

//extern int16_t Controller_Process(MPU6050_S_Measurements values)
//{
//	float value = 0;
//	static float P_Value = 0;
//	static float I_Value = 0;
//	static float D_Value = 0;
//	static float force_Accel_Y_Old = 0;
//	static float velocity_Axis_Y_old = 0;
//	
//	float force_Accel_Y = values.ACCEL_Y;
//	force_Accel_Y /= 32767;
//	force_Accel_Y *= 4 * 9.81;																												// divided by 2^15 - 1 and multiplied by 4G
//	float velocity_Axis_Y = values.GYRO_Y;
//	velocity_Axis_Y /= 32767;
//	velocity_Axis_Y *= 500; 																													//divided by 2^15 - 1 and multiplied by 500°/s
//	velocity_Axis_Y = (velocity_Axis_Y / 360) * SENSOR_AXIS_DISTANCE * 3.14159 * 2; 	//divided by 360°, multiplied with circumference
//	float acceleration_axis_Y = (velocity_Axis_Y - velocity_Axis_Y_old) / SENSOR_TIME_SLICE;
//	force_Accel_Y -= acceleration_axis_Y;

//	P_Value = force_Accel_Y * P_FACTOR;
////	I_Value += force_Accel_Y * I_FACTOR;
////	if(force_Accel_Y >= 0)
////	{
////		D_Value = (force_Accel_Y_Old - force_Accel_Y) * D_FACTOR;
////	}
////	else
////	{
////		D_Value = (force_Accel_Y_Old - force_Accel_Y) * D_FACTOR * (-1);
////	}
//	value = P_Value;
//	velocity_Axis_Y_old = velocity_Axis_Y;
//	force_Accel_Y_Old = force_Accel_Y;
//	
////	value = values.ACCEL_Z;
////	value = 1 / value * P_FACTOR;
//	
//	if(value < -100 || value > 100)
//	{
//		return (int16_t)value;
//	}
//	return 0;
//}
