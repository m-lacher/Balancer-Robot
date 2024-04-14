//------------------------------------------------------
//File:			DataProcessor.h
//Project:	Balancer Robot
//Created:	18.10.2019
//Autor:		M.Lacher
//------------------------------------------------------

#include "MPU6050.h"

#ifndef DataProcessor
#define DataProcessor

extern void DataProcessor_InitModule(void);
extern MPU6050_S_Measurements DataProcessor_Process(MPU6050_S_Measurements measurement);


#endif
