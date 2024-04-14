//------------------------------------------------------
//File:			MPU6050.c
//Project:	Balancer Robot
//Created:	20.09.2019
//Autor:		M.Lacher
//------------------------------------------------------

/*
This module is used to Operate with the MPU-6050 Sensor.
It provides funtions to configure the sensor and reading the measurements.
This module is using the I2C.h Interface to communicate with the Sensor.
*/

#include "MPU6050.h"

#define DEVICE_ADDR				0x69

//Register addresses
#define SELF_TEST_X 			0x0d
#define SELF_TEST_Y 			0x0e
#define SELF_TEST_Z				0x0f
#define SELF_TEST_A				0x10
#define SMPLRT_DIV				0x19
#define CONFIG						0x1a
#define GYRO_CONFIG				0x1b
#define ACCEL_CONFIG 			0x1c
#define FIFO_EN						0x23
#define I2C_MST_CTRL			0x24
#define I2C_SLV0_ADDR			0x25
#define I2C_SLV0_REG			0x26
#define I2C_SLV0_CTRL			0x27
#define I2C_SLV1_ADDR			0x28
#define I2C_SLV1_REG			0x29
#define I2C_SLV1_CTRL			0x2a
#define I2C_SLV2_ADDR			0x2b
#define I2C_SLV2_REG			0x2c
#define I2C_SLV2_CTRL			0x2d
#define I2C_SLV3_ADDR			0x2e
#define I2C_SLV3_REG			0x2f
#define I2C_SLV3_CTRL			0x30
#define I2C_SLV4_ADDR			0x31
#define I2C_SLV4_REG			0x32
#define I2C_SLV4_DO				0x33
#define I2C_SLV4_CTRL			0x34
#define I2C_SLV4_DI				0x35
#define I2C_MST_STATUS		0x36
#define INT_PIN_CFG				0x37
#define INT_ENABLE				0x38
#define INT_STATUS				0x3a
#define ACCEL_XOUT_H			0x3b
#define ACCEL_XOUT_L			0x3c
#define ACCEL_YOUT_H			0x3d
#define ACCEL_YOUT_L			0x3e
#define ACCEL_ZOUT_H			0x3f
#define ACCEL_ZOUT_L			0x40
#define TEMP_OUT_H				0x41
#define TEMP_OUT_L				0x42
#define GYRO_XOUT_H				0x43
#define	GYRO_XOUT_L				0x44
#define GYRO_YOUT_H				0x45
#define	GYRO_YOUT_L				0x46
#define GYRO_ZOUT_H				0x47
#define	GYRO_ZOUT_L				0x48
#define EXT_SENS_DATA_00 	0x49
#define EXT_SENS_DATA_01 	0x4a
#define EXT_SENS_DATA_02 	0x4b
#define EXT_SENS_DATA_03 	0x4c
#define EXT_SENS_DATA_04 	0x4d
#define EXT_SENS_DATA_05 	0x4e
#define EXT_SENS_DATA_06 	0x4f
#define EXT_SENS_DATA_07 	0x50
#define EXT_SENS_DATA_08 	0x51
#define EXT_SENS_DATA_09 	0x52
#define EXT_SENS_DATA_10 	0x53
#define EXT_SENS_DATA_11 	0x54
#define EXT_SENS_DATA_12 	0x55
#define EXT_SENS_DATA_13 	0x56
#define EXT_SENS_DATA_14 	0x57
#define EXT_SENS_DATA_15 	0x58
#define EXT_SENS_DATA_16 	0x59
#define EXT_SENS_DATA_17 	0x5a
#define EXT_SENS_DATA_18 	0x5b
#define EXT_SENS_DATA_19 	0x5c
#define EXT_SENS_DATA_20 	0x5d
#define EXT_SENS_DATA_21 	0x5e
#define EXT_SENS_DATA_22 	0x5f
#define EXT_SENS_DATA_23 	0x60
#define I2C_SLV0_DO				0x63
#define I2C_SLV1_DO				0x64
#define I2C_SLV2_DO				0x65
#define I2C_SLV3_DO				0x66
#define I2C_MST_DELAY_CT	0x67
#define SIGNAL_PATH_RESET	0x68
#define USER_CTRL					0x6a
#define PWR_MGMT_1				0x6b
#define PWR_MGMT_2				0x6c
#define FIFO_COUNT_H			0x72
#define FIFO_COUNT_L			0x73
#define FIFO_R_W					0x74
#define WHO_AM_I					0x75

static void (*mpu6050_callbackFunctionPointer)(void);
void EmptyCallbackFunction(void);


//*********** Public Functions **************************************************************
void MPU6050_InitModule(void)
{
	mpu6050_callbackFunctionPointer = EmptyCallbackFunction;													//Make sure pointer isn't empty
	HAL_GPIO_WritePin(AD0_GPIO_Port, AD0_Pin, GPIO_PIN_SET);													//Set AD0 Pin to 0 -> ADDR 0x68
	
	HAL_NVIC_SetPriority(EXTI4_IRQn, 10, 0);
  HAL_NVIC_DisableIRQ(EXTI4_IRQn);
}
	
void MPU6050_InitDevice(MPU6050_InitTypeDef *initStruct)
{
	I2C_WriteRegister(DEVICE_ADDR, SMPLRT_DIV, initStruct->ClockDiv);									//Write to Samplerate divide register
	uint8_t pwr_mgmt_1_value  = initStruct->OperatingMode | initStruct->ClockSource;
	I2C_WriteRegister(DEVICE_ADDR, PWR_MGMT_1, pwr_mgmt_1_value);											//Write to power managment 1 register
}

void MPU6050_InitGyro(MPU6050_SensorInitTypeDef *initStruct, uint8_t axis)
{
	I2C_WriteRegister(DEVICE_ADDR, GYRO_CONFIG, initStruct->FullScaleRange);					//Set Full scale range of gyro
	uint8_t pwr_mgmt_2_value = I2C_ReadRegister(DEVICE_ADDR, PWR_MGMT_2);							//Read power managment 2 register
	pwr_mgmt_2_value &= ~(axis);																											//Clear sensor bit 0 = On, 1 = Standby
	I2C_WriteRegister(DEVICE_ADDR, PWR_MGMT_2, pwr_mgmt_2_value);											//Write power managment 2 register
}

void MPU6050_InitAccel(MPU6050_SensorInitTypeDef *initStruct, uint8_t axis)
{
	I2C_WriteRegister(DEVICE_ADDR, ACCEL_CONFIG, initStruct->FullScaleRange);					//Set Full scale range of acceleromter
	uint8_t pwr_mgmt_2_value = I2C_ReadRegister(DEVICE_ADDR, PWR_MGMT_2);							//Read power managment 2 register
	pwr_mgmt_2_value &= ~(axis);																											//Clear sensor bit 0 = On, 1 = Standby
	I2C_WriteRegister(DEVICE_ADDR, PWR_MGMT_2, pwr_mgmt_2_value);											//Write power managment 2 register
}

void MPU6050_SetDigitalLowPassFilter(uint8_t bandwidth)
{
	uint8_t configRegister = I2C_ReadRegister(DEVICE_ADDR, CONFIG);
	configRegister &= 0xf8;																														//Clear bit 0-2
	configRegister |= bandwidth << 0;																									//set DLPF_CFG value
	I2C_WriteRegister(DEVICE_ADDR, CONFIG, configRegister);
}

void MPU6050_EnableInterrupt(uint8_t interrupt)
{
	uint8_t interruptEnableRegister = I2C_ReadRegister(DEVICE_ADDR, INT_ENABLE);			//Read interrupt enable register
	interruptEnableRegister |= interrupt;																							//Set interrupt bit
	I2C_WriteRegister(DEVICE_ADDR, INT_ENABLE, interruptEnableRegister);							//Write interrupt enable register
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);																										//Enable GPIO interrupt
}

void MPU6050_DisableInterrupt(uint8_t interrupt)
{
	uint8_t interruptEnableRegister = I2C_ReadRegister(DEVICE_ADDR, INT_ENABLE);			//Read interrupt enable register
	interruptEnableRegister &= ~(interrupt);																					//Clear	interrupt bit
	I2C_WriteRegister(DEVICE_ADDR, INT_ENABLE, interruptEnableRegister);							//Write interrupt enable register
	HAL_NVIC_DisableIRQ(EXTI4_IRQn);																									//Disable GPIO interrupt
}

void MPU6050_ReadValues(MPU6050_S_Measurements *sensorValues)
{
	uint8_t sensorData[14];																														//Buffer for sensor data
	uint8_t startAddr = ACCEL_XOUT_H;
	I2C_Write(DEVICE_ADDR, &startAddr, 1);																						//Write start address of data
	I2C_Read(DEVICE_ADDR, sensorData, sizeof(sensorData));														//Burst read all sensor bytes
	sensorValues->ACCEL_X = (sensorData[0] << 8) | sensorData[1];											//Calculate values
	sensorValues->ACCEL_Y = (sensorData[2] << 8) | sensorData[3];
	sensorValues->ACCEL_Z = (sensorData[4] << 8) | sensorData[5];
	sensorValues->TEMP = (sensorData[6] << 8) | sensorData[7];
	sensorValues->GYRO_X = (sensorData[8] << 8) | sensorData[9];
	sensorValues->GYRO_Y = (sensorData[10] << 8) | sensorData[11];
	sensorValues->GYRO_Z = (sensorData[12] << 8) | sensorData[13];
}

void MPU6050_SetInterruptCallback(void (*callbackFunctionPointer)(void))
{
	mpu6050_callbackFunctionPointer = callbackFunctionPointer;
}

void MPU6050_ClearInterruptCallback()
{
	mpu6050_callbackFunctionPointer = EmptyCallbackFunction;
}

//*********** Private Functions **************************************************************
void EmptyCallbackFunction(void)
{
	//Empty callback
}
void EXTI4_IRQHandler(void)																													//Int pin interrupt routine
{
	uint8_t statusRegister = I2C_ReadRegister(DEVICE_ADDR, INT_STATUS);								//clear interrupt bits
	mpu6050_callbackFunctionPointer();																								//execute callback function
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);																							//Standard Interrupt handler
}
