//------------------------------------------------------
//File:			DataProcessor.c
//Project:	Balancer Robot
//Created:	18.10.2019
//Autor:		M.Lacher
//------------------------------------------------------

#include "DataProcessor.h"

#define DATABUFFERSIZE 20

MPU6050_S_Measurements dataBuffer[DATABUFFERSIZE];
static int dataIndex = 0;

void DataProcessor_InitModule()
{
	//Create a dataStructure to initialize dataBuffer
	MPU6050_S_Measurements initValue;
	initValue.ACCEL_X = 0;
	initValue.ACCEL_Y = 0;
	initValue.ACCEL_Z = 0;
	initValue.GYRO_X = 0;
	initValue.GYRO_Y = 0;
	initValue.GYRO_Z = 0;
	
	int index;
	for(index=0; index < DATABUFFERSIZE; index++)
	{
		dataBuffer[index] = initValue;
	}
}

//Calculates dynamic mean
MPU6050_S_Measurements DataProcessor_Process(MPU6050_S_Measurements measurement)
{
	MPU6050_S_Measurements returnValue;
	int32_t accelX=0, accelY=0, accelZ=0, gyroX=0, gyroY=0, gyroZ=0;
	int index;
	
	dataBuffer[dataIndex] = measurement;
	dataIndex++;
	if(dataIndex >= DATABUFFERSIZE)
	{
		dataIndex = 0;
	}
	
	for(index=0; index<DATABUFFERSIZE; index++)
	{
		accelX += dataBuffer[index].ACCEL_X;
		accelY += dataBuffer[index].ACCEL_Y;
		accelZ += dataBuffer[index].ACCEL_Z;
		gyroX += dataBuffer[index].GYRO_X;
		gyroY += dataBuffer[index].GYRO_Y;
		gyroZ += dataBuffer[index].GYRO_Z;
	}	
	returnValue.ACCEL_X = accelX / DATABUFFERSIZE;
	returnValue.ACCEL_Y = accelY / DATABUFFERSIZE;
	returnValue.ACCEL_Z = accelZ / DATABUFFERSIZE;
	returnValue.GYRO_X = gyroX / DATABUFFERSIZE;
	returnValue.GYRO_Y = gyroY / DATABUFFERSIZE;
	returnValue.GYRO_Z = gyroZ / DATABUFFERSIZE;
	
	return returnValue;
}