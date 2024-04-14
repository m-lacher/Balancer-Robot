//------------------------------------------------------
//File:			I2C.h
//Project:	Balancer Robot
//Created:	19.09.2019
//Autor:		M.Lacher
//------------------------------------------------------

/*
This module defines the interface for the I2C bus.
It's a low lever driver for the MPU6050 modul.
*/

#include "stm32f3xx_hal.h"

#ifndef I2C
#define I2C

extern void I2C_InitModule(void);
extern void I2C_Write(uint8_t addr, uint8_t *data, uint16_t len);
extern void I2C_Read(uint8_t addr, uint8_t *buffer, uint16_t len);
extern void I2C_WriteRegister(uint8_t addr, uint8_t regAddr, uint8_t value);
extern uint8_t I2C_ReadRegister(uint8_t addr, uint8_t regAddr);
extern void I2C_SetBit(uint8_t addr, uint8_t registerAddr, uint8_t bit);
extern void I2C_ClearBit(uint8_t addr, uint8_t registerAddr, uint8_t bit);

#endif
