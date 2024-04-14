//-------------------------------------------------------------------------------
//File		:	I2C_Driver_STM32F303K8.h
//Creator	:	M.Lacher
//Created	:	18.05.2018
//Function:	Header file for Hardware use of Nucleo STM32F303K8
//-------------------------------------------------------------------------------

#include "stm32f3xx_hal.h"

void i2c_write(char addr, uint8_t *data, int lentgh);
void i2c_read(char addr, uint8_t *buffer, int lentgh);

typedef void (*I2C_Write)(char addr, uint8_t *data, int lentgh);
typedef void (*I2C_Read)(char addr, uint8_t *buffer, int lentgh);

typedef struct i2c
{
	I2C_Write write;
	I2C_Read read;
}I2C;

extern I2C *getDriver();
extern void initHardware();
extern void writeToTerminal(uint8_t *data, uint16_t len);
