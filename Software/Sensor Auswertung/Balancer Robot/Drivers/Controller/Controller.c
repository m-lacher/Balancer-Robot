//------------------------------------------------------
//File:			Controller.c
//Project:	Balancer Robot
//Created:	27.09.2019
//Autor:		M.Lacher
//------------------------------------------------------

#include "Controller.h"

#define BUFFER_SIZE 10
#define P_FACTOR 100
#define REACTION_VALUE 4000		//1422 equals 10°, when sensor max is 4G
#define SENSOR_AXIS_DISTANCE	0.2
#define SENSOR_TIME_SLICE 0.01

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

extern int16_t Controller_Process(MPU6050_S_Measurements values)
{
	static float velocity_Axis_Y_old = 0;
	float value = 0;
	float force_Accel_Y = values.ACCEL_Y;
	force_Accel_Y /= 32767;
	force_Accel_Y *= 4 * 9.81;		// divided by 2^15 - 1 and multiplied by 4G
	float velocity_Axis_Y = values.GYRO_Y;
	velocity_Axis_Y /= 32767;
	velocity_Axis_Y *= 500; 					//divided by 2^15 - 1 and multiplied by 500°/s
	velocity_Axis_Y = (velocity_Axis_Y / 360) * SENSOR_AXIS_DISTANCE * 3.14159 * 2; //divided by 360°, multiplied with circumference
	float acceleration_axis_Y = (velocity_Axis_Y - velocity_Axis_Y_old) / SENSOR_TIME_SLICE;
	force_Accel_Y -= acceleration_axis_Y;

	value = force_Accel_Y * P_FACTOR;
	velocity_Axis_Y_old = velocity_Axis_Y;
	if(value < -100 || value > 100)
	{
		return (int16_t)value;
	}
	return 0;
}
