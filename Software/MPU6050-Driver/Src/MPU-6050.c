//-------------------------------------------------------------------------------
//File		:	MPU-6050.c
//Creator	:	M.Lacher
//Created	:	23.05.2018
//Function:	Source file for MPU-6050 driver
//-------------------------------------------------------------------------------

#include "MPU-6050.h"

MPU6050 m_mpu6050;
Accelerometer m_accelerometer = 
	{
		f_accel_enableAxis, 
		f_accel_disableAxis,
		f_accel_reset,
		f_accel_setFullScaleRange
	};
	
Gyroscope m_gyroscope = 
	{
		f_gyro_enableAxis,
		f_gyro_disableAxis,
		f_gyro_reset,
		f_gyro_setFullScaleRange
	}; 

Temperature m_temperature = 
{
	f_temp_enableSensor,
	f_temp_disableSensor,
	f_temp_reset
};

FIFO m_fifo = 
{
	f_addToBuffer,
	f_removeFromBuffer,
	f_getDataSize,
	f_getFifoData,
	f_fifo_enable,
	f_fifo_disable,
	f_clearBuffer,
};

MPU6050 *initializeDevice(I2C *driver, char address)
{
	m_mpu6050.i2c = driver;
	m_mpu6050.address = address;
	m_mpu6050.accelerometer = &m_accelerometer;
	m_mpu6050.gyroscope = &m_gyroscope;
	m_mpu6050.temperature = &m_temperature;
	m_mpu6050.fifo = &m_fifo;
	m_mpu6050.setOperatingMode = f_setOperatingMode;
	m_mpu6050.reset = f_reset;
	m_mpu6050.setClockSource = f_setClockSource;
	m_mpu6050.setSampleRate = f_setSampleRate;
	return &m_mpu6050;
}

//---------- Control Functions --------------------------------------------------------------------
void f_setOperatingMode(char mode)
{
	switch(mode)
	{
		case STANDARD:
			clearBit(PWR_MGMT_1, 6);		//Deletes SLEEP Bit
			clearBit(PWR_MGMT_1, 5);		//Deletes CYCLE Bit
			break;
		
		case CYCLE:
			clearBit(PWR_MGMT_1, 6);		//Deletes SLEEP Bit
			setBit(PWR_MGMT_1, 5);			//Sets CYCLE Bit
			break;
		
		case SLEEP:
			setBit(PWR_MGMT_1, 6);			//Sets SLEEP Bit
			break;
		
		default:
			break;
	}
}
void f_reset()		//Resets all register to default
{
	setBit(PWR_MGMT_1, 7); //Automatically clears to 0
}

void f_setClockSource(char clockSource)
{
	if(!(clockSource == 6 || clockSource < 0 || clockSource > 7))
	{
		uint8_t reg = PWR_MGMT_1;
		uint8_t val;
		uint8_t data[2];
		m_mpu6050.i2c->write(m_mpu6050.address, &reg, 1);
		m_mpu6050.i2c->read(m_mpu6050.address, &val, 1);
		data[0] = reg;
		data[1] = (val & 0xf8) | clockSource;
		m_mpu6050.i2c->write(m_mpu6050.address, data, 2);
	}
}

void f_setSampleRate(uint16_t sampleRate)
{
	if (sampleRate <= 8000)
	{
		uint8_t divFactor = 8000 / sampleRate;
		uint8_t data [2] = {SMPLRT_DIV, divFactor};
		m_mpu6050.i2c->write(m_mpu6050.address, data, 2);
	}
}



//---------- Accelerometer Functions --------------------------------------------------------------------
void f_accel_enableAxis(char axis)		//Activates the chosen axis
{
	clearBit(PWR_MGMT_2, 2-axis);
}
void f_accel_disableAxis(char axis)		//Sets the chosen axis to standby
{
	setBit(PWR_MGMT_2, 2-axis);
}
void f_accel_reset()									//Resets the analog and digital signal paths
{
	uint8_t data[2] = {SIGNAL_PATH_RESET, 0x02};
	m_mpu6050.i2c->write(m_mpu6050.address, data, 2);
	data[1] = 0x00;
	m_mpu6050.i2c->write(m_mpu6050.address, data, 2);
}
void f_accel_setFullScaleRange(uint8_t gMax)
{
	if(gMax < 4)
	{
		clearBit(ACCEL_CONFIG, 3);
		clearBit(ACCEL_CONFIG, 4);
	}
	else if(gMax < 8)
	{
		setBit(ACCEL_CONFIG, 3);
		clearBit(ACCEL_CONFIG, 4);
	}
	else if(gMax < 16)
	{
		clearBit(ACCEL_CONFIG, 3);
		setBit(ACCEL_CONFIG, 4);
	}
	else
	{
		setBit(ACCEL_CONFIG, 3);
		setBit(ACCEL_CONFIG, 4);
	}
}

//---------- Gyroscope Functions --------------------------------------------------------------------
void f_gyro_enableAxis(char axis)		//Activates the chosen axis
{
	clearBit(PWR_MGMT_2, 5-axis);
}
void f_gyro_disableAxis(char axis)	//Sets the chosen axis to standby
{
	setBit(PWR_MGMT_2, 5-axis);
}
void f_gyro_reset()									//Resets the analog and digital signal paths
{
	uint8_t data[2] = {SIGNAL_PATH_RESET, 0x04};
	m_mpu6050.i2c->write(m_mpu6050.address, data, 2);
	data[1] = 0x00;
	m_mpu6050.i2c->write(m_mpu6050.address, data, 2);
}
void f_gyro_setFullScaleRange(uint16_t dps)	//Sets the Full Scale Range
{
	if(dps < 500)								//250°/s
	{
		clearBit(GYRO_CONFIG, 3);
		clearBit(GYRO_CONFIG, 4);
	}
	else if(dps < 1000)					//500°/s
	{
		setBit(GYRO_CONFIG, 3);
		clearBit(GYRO_CONFIG, 4);
	}
	else if(dps < 2000)					//1000°/s
	{
		clearBit(GYRO_CONFIG, 3);
		setBit(GYRO_CONFIG, 4);
	}
	else												//2000°/s
	{
		setBit(GYRO_CONFIG, 3);
		setBit(GYRO_CONFIG, 4);
	}
}

//---------- Temperature Functions --------------------------------------------------------------------
void f_temp_enableSensor()				//Activates the sensor
{
	clearBit(PWR_MGMT_1, 3);
}
void f_temp_disableSensor()				//Sets the sensor to standby
{
	setBit(PWR_MGMT_1, 3);
}
void f_temp_reset()								//Resets the analog and digital signal paths
{
	uint8_t data[2] = {SIGNAL_PATH_RESET, 0x01};
	m_mpu6050.i2c->write(m_mpu6050.address, data, 2);
	data[1] = 0x00;
	m_mpu6050.i2c->write(m_mpu6050.address, data, 2);
}

//---------- FIFO Functions --------------------------------------------------------------------

void f_addToBuffer(uint8_t sens)		//Adds the chosen Sensor/Axis to the FIFO list
{
	setBit(FIFO_EN, sens);
}
void f_removeFromBuffer(uint8_t sens)	//Removes the chosen Sensor/Axis from the FIFO list
{
	clearBit(FIFO_EN, sens);
}
uint16_t f_getDataSize()
{
	uint8_t addr = FIFO_COUNT_H;
	uint8_t highByte = 0, lowByte = 0;
	uint16_t result;
	m_mpu6050.i2c->write(m_mpu6050.address, &addr, 1);
	m_mpu6050.i2c->read(m_mpu6050.address, &highByte, 1);
	addr = FIFO_COUNT_L;
	m_mpu6050.i2c->write(m_mpu6050.address, &addr, 1);
	m_mpu6050.i2c->read(m_mpu6050.address, &lowByte, 1);
	result = highByte;
	result = (result << 8) | lowByte;
	return result;
}
void f_fifo_enable()
{
	setBit(USER_CTRL, 6);
}
void f_fifo_disable()
{
	clearBit(USER_CTRL, 6);
}
void f_clearBuffer()				//Clears the Buffer. FIFO has to be enabled again!
{
	m_fifo.disable();
	setBit(USER_CTRL, 2);
}
void f_getFifoData(uint8_t * buffer, uint16_t len)
{
	uint8_t addr = FIFO_R_W;
	m_mpu6050.i2c->write(m_mpu6050.address, &addr, 1);
	m_mpu6050.i2c->read(m_mpu6050.address, buffer, len);
}

//---------- I2C Functions --------------------------------------------------------------------
void setBit(uint8_t regAddr, char bit)		//Sets 1 bit in a register of MPU-6050
{
	uint8_t reg;
	uint8_t data[2];
	m_mpu6050.i2c->write(m_mpu6050.address, &regAddr, 1);
	m_mpu6050.i2c->read(m_mpu6050.address, &reg, 1);
	data[0] = regAddr;
	data[1] = reg | (0x01 << bit);
	m_mpu6050.i2c->write(m_mpu6050.address, data, 2);
}
void clearBit(uint8_t regAddr, char bit)		//Clears 1 bit in a register of MPU-6050
{
	uint8_t reg;
	uint8_t data[2];
	m_mpu6050.i2c->write(m_mpu6050.address, &regAddr, 1);
	m_mpu6050.i2c->read(m_mpu6050.address, &reg, 1);
	data[0] = regAddr;
	data[1] = reg & (0xff ^(0x01 << bit));
	m_mpu6050.i2c->write(m_mpu6050.address, data, 2);
}
