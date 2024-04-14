//------------------------------------------------------
//File:			I2C.c
//Project:	Balancer Robot
//Created:	19.09.2019
//Autor:		M.Lacher
//------------------------------------------------------

/*
This module defines the interface for the I2C bus.
It's a low lever driver for the MPU6050 modul.
*/

#include "I2C.h"
#include "ErrorHandler.h"

#define I2C_TIMEOUT 1000

I2C_HandleTypeDef I2C_DEV;

static void MX_I2C1_Init(void);
static void CheckStatus(HAL_StatusTypeDef status);

void I2C_InitModule()
{
	MX_I2C1_Init();
}

void I2C_Write(uint8_t addr, uint8_t *data, uint16_t len)								//Writes data to I2C slave device
{
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&I2C_DEV, addr << 1, data, len, I2C_TIMEOUT);						//shift addr, bit0 = 0 -> Write
	CheckStatus(status);
}

void I2C_Read(uint8_t addr, uint8_t *buffer, uint16_t len)							//Reads data from I2C slave device
{
	HAL_StatusTypeDef status = HAL_I2C_Master_Receive(&I2C_DEV, (addr << 1) | 0x01, buffer, len, I2C_TIMEOUT);	//shift addr, bit0 = 1 -> Read
	CheckStatus(status);
}

void I2C_WriteRegister(uint8_t addr, uint8_t regAddr, uint8_t value)		//Write single register
{
	uint8_t data[2] = {regAddr, value};
	I2C_Write(addr, data, 2);
}

uint8_t I2C_ReadRegister(uint8_t addr, uint8_t regAddr)									//Read single register
{
	uint8_t regValue[1];
	I2C_Write(addr, &regAddr, 1);
	I2C_Read(addr, regValue, 1);
	return regValue[0];
}

void I2C_SetBit(uint8_t addr, uint8_t regAddr, uint8_t bit)							//Set single bit
{
	uint8_t regValue[1];
	uint8_t command[2];
	I2C_Write(addr, &regAddr, 1);
	I2C_Read(addr, regValue, 1);
	command[1] = regValue[0] | (1 << bit);
	command[0] = regAddr;
	I2C_Write(addr, command, 2);
}

void I2C_ClearBit(uint8_t addr, uint8_t regAddr, uint8_t bit)						//Clear single bit
{
	uint8_t regValue[1];
	uint8_t command[2];
	I2C_Write(addr, &regAddr, 1);
	I2C_Read(addr, regValue, 1);
	command[1] = regValue[0] & (~(1<<bit));
	command[0] = regAddr;
	I2C_Write(addr, command, 2);
}

static void CheckStatus(HAL_StatusTypeDef status)												//Check if comunication was sucessfull
{
	switch(status)
	{
		case HAL_OK:
			ErrorHandler_ResetWarning();
			break;
		case HAL_BUSY:
			break;
		case HAL_ERROR:
			ErrorHandler_HardFault();
			break;
		case HAL_TIMEOUT:
			ErrorHandler_SetWarning();																				//signals the user that the communication failed
			break;
		default:
			break;
	}
}

static void MX_I2C1_Init(void)
{
	I2C_DEV.Instance = I2C1;
  I2C_DEV.Init.Timing = 0x0000020B;
  I2C_DEV.Init.OwnAddress1 = 0;
  I2C_DEV.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  I2C_DEV.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  I2C_DEV.Init.OwnAddress2 = 0;
  I2C_DEV.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  I2C_DEV.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  I2C_DEV.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&I2C_DEV) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  if (HAL_I2CEx_ConfigAnalogFilter(&I2C_DEV, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  if (HAL_I2CEx_ConfigDigitalFilter(&I2C_DEV, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}