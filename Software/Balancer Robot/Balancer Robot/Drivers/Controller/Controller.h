//------------------------------------------------------
//File:			Controller.h
//Project:	Balancer Robot
//Created:	27.09.2019
//Autor:		M.Lacher
//------------------------------------------------------

#ifndef Controller
#define Controller

#include <stdint.h>
#include "MPU6050.h"

extern void Controller_InitModule(void);
extern void Controller_SetPoint(uint16_t setPoint);
extern int16_t Controller_Process(MPU6050_S_Measurements values);

#endif
