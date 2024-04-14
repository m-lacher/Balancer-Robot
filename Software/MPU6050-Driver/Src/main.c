//-------------------------------------------------------------------------------
//File		:	MPU-6050.h
//Creator	:	M.Lacher
//Created	:	23.05.2018
//Function:	Header file for MPU-6050 driver
//-------------------------------------------------------------------------------

#include "main.h"
#include "MPU-6050.h"

#define ADDR 0x68

int main(void)
{
	uint16_t len = 0;
	uint8_t data[100];
	uint8_t msg[150];
	int size = 0;
	
	int16_t accelX, accelY, accelZ, gyroX, gyroY, gyroZ;
	
	initHardware();															//Initializes the nucleo board
	I2C *i2c = getDriver(); 										//Gets an I2C pointer
	MPU6050 *driver;					 									//Creates a driver pointer
	driver = initializeDevice(i2c,ADDR);				//Initializes the driver
	
	driver->setOperatingMode(STANDARD);					//Standard operating mode
	driver->gyroscope->enableAxis(X);						//Activates X-axis of gyro
	driver->gyroscope->enableAxis(Y);						//Activates Y-axis of gyro
	driver->gyroscope->enableAxis(Z);						//Activates Z-axis of gyro
	driver->accelerometer->enableAxis(X);
	driver->accelerometer->enableAxis(Y);
	driver->accelerometer->enableAxis(Z);
	
	driver->setClockSource(PLL_X_AXIS_GYRO);		//Uses the X-axis of gyro as clock source
	driver->setSampleRate(50);									//Sets sample rate to 100 samples/sec
	driver->gyroscope->setFullScaleRange(100);	//Max range gyro 1000°/s
	driver->accelerometer->setFullScaleRange(4);//Max value 4G
	driver->fifo->addToBufferList(GYROX);				//Actives fifo for gyroscope x-axis
	driver->fifo->addToBufferList(GYROY);				//Actives fifo for gyroscope y-axis
	driver->fifo->addToBufferList(GYROZ);				//Actives fifo for gyroscope z-axis
	driver->fifo->addToBufferList(ACCEL);
	driver->fifo->clearBuffer();								//Clears all data in the fifo
	driver->fifo->enable();											//Enables I2C operations on fifo   
	
  while (1)   
	{
		len = driver->fifo->getDataSize();				//Reads the datasize of the fifo
		if(len >= 12)
		{
			driver->fifo->getData(data, len);				//Reads the data of the fifo

			accelX = (data[0] << 8) + data[1];
			accelY = (data[2] << 8) + data[3];
			accelZ = (data[4] << 8) + data[5];
			gyroX = (data[6] << 8) + data[7];
			gyroY = (data[8] << 8) + data[9];
			gyroZ = (data[10] << 8) + data[11];
			
			size = snprintf(msg, sizeof(msg), "%i\t%i\t%i\t%i\t%i\t%i \r\n", accelX, accelY, accelZ, gyroX, gyroY, gyroZ);
			writeToTerminal(msg, size);
		}
	}
}
 