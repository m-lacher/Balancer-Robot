//-------------------------------------------------------------------------------
//File		:	MPU-6050.h
//Creator	:	M.Lacher
//Created	:	23.05.2018
//Function:	Header file for MPU-6050 driver
//-------------------------------------------------------------------------------

#include "I2C_Driver_STM32F303K8.h"

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

enum operatingMode
{
	STANDARD,
	CYCLE,
	SLEEP
};

enum sensors
{
	EXT0,
	EXT1,
	EXT2,
	ACCEL,
	GYROZ,
	GYROY,
	GYROX,
	TEMP
};

enum clockSource
{
	INTERNAL_OSCILLATOR,
	PLL_X_AXIS_GYRO,
	PLL_Y_AXIS_GYRO,
	PLL_Z_AXIS_GYRO,
	PLL_EXT_1,
	PLL_EXT_2,
	STOPPED = 7
};

enum axis
{
	X,
	Y,
	Z
};

typedef struct 
{
	void(*enableAxis)(char axis);
	void(*disableAxis)(char axis);
	void(*reset)();
	void(*setFullScaleRange)(uint8_t gMax);
}Accelerometer;

typedef struct
{
	void(*enableAxis)(char axis);
	void(*disableAxis)(char axis);
	void(*reset)();
	void(*setFullScaleRange)(uint16_t dpsMax);
}Gyroscope;

typedef struct
{
	void(*enableSensor)();
	void(*disableSensor)();
	void(*reset)();
}Temperature;

typedef struct
{
	void(*addToBufferList)(uint8_t sens);
	void(*removeFromBufferList)(uint8_t sens);
	uint16_t(*getDataSize)();
	void(*getData)(uint8_t * buffer, uint16_t len);
	void(*enable)();
	void(*disable)();
	void(*clearBuffer)();
}FIFO;

typedef struct
{
	I2C *i2c;
	char address;
	Accelerometer *accelerometer;
	Gyroscope *gyroscope;
	Temperature *temperature;
	FIFO *fifo;
	void (*setOperatingMode)(char mode);
	void (*reset)();
	void (*setClockSource)(char clockSource);
	void (*setSampleRate)(uint16_t sampleRate);
}MPU6050;

    
//Control Functions
void f_setOperatingMode(char mode);
void f_reset();
void f_setClockSource(char clockSource);
void f_setSampleRate(uint16_t sampleRate);

//Acceleromter Functions
void f_accel_enableAxis(char axis);
void f_accel_disableAxis(char axis);
void f_accel_reset();
void f_accel_setFullScaleRange(uint8_t gMax);

//Gyroscope Functions
void f_gyro_enableAxis(char axis);
void f_gyro_disableAxis(char axis);
void f_gyro_reset();
void f_gyro_setFullScaleRange(uint16_t dpsMax);

//Temperature Functions
void f_temp_enableSensor();
void f_temp_disableSensor();
void f_temp_reset();

//FIFO Functions
void f_addToBuffer(uint8_t sens);
void f_removeFromBuffer(uint8_t sens);
uint16_t f_getDataSize();
void f_getFifoData(uint8_t * buffer, uint16_t len);
void f_fifo_enable();
void f_fifo_disable();
void f_clearBuffer();

//I2C Functions
void setBit(uint8_t regAddr, char bit);
void clearBit(uint8_t regAddr, char bit);  

//Initialize Driver
MPU6050 *initializeDevice(I2C *driver, char address);

