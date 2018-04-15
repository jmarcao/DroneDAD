/*
 * GyroscopeDriver.c
 *
 * Created: 4/14/2018 6:48:16 PM
 *  Author: John
 */ 

/*
NOTE: This was adapted from the sparkfun driver for the LSM6DS3.
Marshall Taylor @ SparkFun Electronics
May 20, 2015
https://github.com/sparkfun/LSM6DS3_Breakout
https://github.com/sparkfun/SparkFun_LSM6DS3_Arduino_Library

It has been modifed and adapted to work with the SAMD21/SAMW25
*/

#include "GyroscopeDriver.h"

struct SensorSettings settings;

extern struct i2c_master_module i2c_master_instance;

status_t beginCore(void)
{
	status_t returnError = IMU_SUCCESS;

	//Spin for a few ms
	volatile uint8_t temp = 0;
	for( uint16_t i = 0; i < 10000; i++ ) {
		temp++;
	}

	//Check the ID register to determine if the operation was a success.
	uint8_t readCheck;
	readRegister(&readCheck, LSM6DS3_ACC_GYRO_WHO_AM_I_REG);
	if( readCheck != 0x69 ) {
		returnError = IMU_HW_ERROR;
	}

	return returnError;
}

//****************************************************************************//
//
//  ReadRegisterRegion
//
//  Parameters:
//    *outputPointer -- Pass &variable (base address of) to save read data to
//    offset -- register to read
//    length -- number of bytes to read
//
//  Note:  Does not know if the target memory space is an array or not, or
//    if there is the array is big enough.  if the variable passed is only
//    two bytes long and 3 bytes are requested, this will over-write some
//    other memory!
//
//****************************************************************************//
status_t readRegisterRegion(uint8_t *outputPointer , uint8_t offset, uint8_t length)
{
	status_t status;
	for(int i = 0; i < length; i++) {
		status = readRegister(outputPointer + (i * sizeof(uint8_t*)), offset + i);
		if(status != IMU_SUCCESS) {
			return status;
		}
	}
	
	return status;
}

//****************************************************************************//
//
//  ReadRegister
//
//  Parameters:
//    *outputPointer -- Pass &variable (address of) to save read data to
//    offset -- register to read
//
//****************************************************************************//
status_t readRegister(uint8_t* outputPointer, uint8_t offset) {
	//Return value
	uint8_t result;
	uint8_t numBytes = 1;
	
	enum status_code i2c_status;
	lsm6ds3_wr_packet.address = LSM6DS3_SLAVE_ADDR;
	lsm6ds3_wr_packet.data_length = 1;
	lsm6ds3_wr_buffer[0] = offset;
	lsm6ds3_wr_packet.data = lsm6ds3_wr_buffer;
	i2c_status = i2c_master_write_packet_wait_no_stop(&i2c_master_instance, &lsm6ds3_wr_packet);
	delay_us(DELAY_VALUE_US);
	if(i2c_status != STATUS_OK) {
		//uhoh
		int i = 0;
	}
	else {
		lsm6ds3_rd_packet.address = LSM6DS3_SLAVE_ADDR;
		lsm6ds3_rd_packet.data_length = 1;
		i2c_status = i2c_master_read_packet_wait_no_stop(&i2c_master_instance, &lsm6ds3_rd_packet);
		delay_us(DELAY_VALUE_US);
		if(i2c_status != STATUS_OK) {
			// Uhoh
			int i = 0;
		}
		int i = 0;
	}

	*outputPointer = lsm6ds3_rd_packet.data[0];
	return IMU_SUCCESS;
}

//****************************************************************************//
//
//  readRegisterInt16
//
//  Parameters:
//    *outputPointer -- Pass &variable (base address of) to save read data to
//    offset -- register to read
//
//****************************************************************************//
status_t readRegisterInt16( int16_t* outputPointer, uint8_t offset )
{
	status_t status;
	uint8_t myBuffer[2];
	status = readRegisterRegion(&myBuffer[0], offset, 2);  //Does memory transfer
	int16_t output = (int16_t)myBuffer[0] | (int16_t)(myBuffer[1] << 8);
	
	*outputPointer = output;
	return status;
}

//****************************************************************************//
//
//  writeRegister
//
//  Parameters:
//    offset -- register to write
//    dataToWrite -- 8 bit data to write to register
//
//****************************************************************************//
status_t writeRegister(uint8_t offset, uint8_t dataToWrite) {
	enum status_code i2c_status;
	lsm6ds3_wr_packet.address = LSM6DS3_SLAVE_ADDR;
	lsm6ds3_wr_packet.data_length = 2;
	lsm6ds3_wr_buffer[0] = offset;
	lsm6ds3_wr_buffer[1] = dataToWrite;
	lsm6ds3_wr_packet.data = lsm6ds3_wr_buffer;
	i2c_status = i2c_master_write_packet_wait_no_stop(&i2c_master_instance, &lsm6ds3_wr_packet);
	delay_us(DELAY_VALUE_US);
	if(i2c_status != STATUS_OK) {
		//uhoh
		int i = 0;
	}
	
	return IMU_SUCCESS;
}

status_t embeddedPage( void )
{
	status_t returnError = writeRegister( LSM6DS3_ACC_GYRO_RAM_ACCESS, 0x80 );
	
	return returnError;
}

status_t basePage( void )
{
	status_t returnError = writeRegister( LSM6DS3_ACC_GYRO_RAM_ACCESS, 0x00 );
	
	return returnError;
}


//****************************************************************************//
//
//  Main user class -- wrapper for the core class + maths
//
//  Construct with same rules as the core ( uint8_t busType, uint8_t inputArg )
//
//****************************************************************************//
void setDefaultSettings()
{
	//Construct with these default settings
	settings.gyroEnabled = 1;  //Can be 0 or 1
	settings.gyroRange = 2000;   //Max deg/s.  Can be: 125, 245, 500, 1000, 2000
	settings.gyroSampleRate = 416;   //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
	settings.gyroBandWidth = 400;  //Hz.  Can be: 50, 100, 200, 400;
	settings.gyroFifoEnabled = 1;  //Set to include gyro in FIFO
	settings.gyroFifoDecimation = 1;  //set 1 for on /1

	settings.accelEnabled = 1;
	settings.accelODROff = 1;
	settings.accelRange = 16;      //Max G force readable.  Can be: 2, 4, 8, 16
	settings.accelSampleRate = 416;  //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664, 13330
	settings.accelBandWidth = 100;  //Hz.  Can be: 50, 100, 200, 400;
	settings.accelFifoEnabled = 1;  //Set to include accelerometer in the FIFO
	settings.accelFifoDecimation = 1;  //set 1 for on /1

	settings.tempEnabled = 1;

	//Select interface mode
	settings.commMode = 1;  //Can be modes 1, 2 or 3

	//FIFO control data
	settings.fifoThreshold = 3000;  //Can be 0 to 4096 (16 bit bytes)
	settings.fifoSampleRate = 10;  //default 10Hz
	settings.fifoModeWord = 0;  //Default off

	allOnesCounter = 0;
	nonSuccessCounter = 0;
}

//****************************************************************************//
//
//  Configuration section
//
//  This uses the stored SensorSettings to start the IMU
//  Use statements such as "myIMU.settings.commInterface = SPI_MODE;" or
//  "myIMU.settings.accelEnabled = 1;" to configure before calling .begin();
//
//****************************************************************************//
status_t begin()
{
	//Check the settings structure values to determine how to setup the device
	uint8_t dataToWrite = 0;  //Temporary variable

	//Begin the inherited core.  This gets the physical wires connected
	status_t returnError = beginCore();

	//Setup the accelerometer******************************
	dataToWrite = 0; //Start Fresh!
	if ( settings.accelEnabled == 1) {
		//Build config reg
		//First patch in filter bandwidth
		switch (settings.accelBandWidth) {
			case 50:
			dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_50Hz;
			break;
			case 100:
			dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_100Hz;
			break;
			case 200:
			dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_200Hz;
			break;
			default:  //set default case to max passthrough
			case 400:
			dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_400Hz;
			break;
		}
		//Next, patch in full scale
		switch (settings.accelRange) {
			case 2:
			dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_2g;
			break;
			case 4:
			dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_4g;
			break;
			case 8:
			dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_8g;
			break;
			default:  //set default case to 16(max)
			case 16:
			dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_16g;
			break;
		}
		//Lastly, patch in accelerometer ODR
		switch (settings.accelSampleRate) {
			case 13:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_13Hz;
			break;
			case 26:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_26Hz;
			break;
			case 52:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_52Hz;
			break;
			default:  //Set default to 104
			case 104:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_104Hz;
			break;
			case 208:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_208Hz;
			break;
			case 416:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_416Hz;
			break;
			case 833:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_833Hz;
			break;
			case 1660:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_1660Hz;
			break;
			case 3330:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_3330Hz;
			break;
			case 6660:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_6660Hz;
			break;
			case 13330:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_13330Hz;
			break;
		}
	}
	else
	{
		//dataToWrite already = 0 (powerdown);
	}

	//Now, write the patched together data
	writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);

	//Set the ODR bit
	readRegister(&dataToWrite, LSM6DS3_ACC_GYRO_CTRL4_C);
	dataToWrite &= ~((uint8_t)LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED);
	if ( settings.accelODROff == 1) {
		dataToWrite |= LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED;
	}
	writeRegister(LSM6DS3_ACC_GYRO_CTRL4_C, dataToWrite);

	//Setup the gyroscope**********************************************
	dataToWrite = 0; //Start Fresh!
	if ( settings.gyroEnabled == 1) {
		//Build config reg
		//First, patch in full scale
		switch (settings.gyroRange) {
			case 125:
			dataToWrite |= LSM6DS3_ACC_GYRO_FS_125_ENABLED;
			break;
			case 245:
			dataToWrite |= LSM6DS3_ACC_GYRO_FS_G_245dps;
			break;
			case 500:
			dataToWrite |= LSM6DS3_ACC_GYRO_FS_G_500dps;
			break;
			case 1000:
			dataToWrite |= LSM6DS3_ACC_GYRO_FS_G_1000dps;
			break;
			default:  //Default to full 2000DPS range
			case 2000:
			dataToWrite |= LSM6DS3_ACC_GYRO_FS_G_2000dps;
			break;
		}
		//Lastly, patch in gyro ODR
		switch (settings.gyroSampleRate) {
			case 13:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_13Hz;
			break;
			case 26:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_26Hz;
			break;
			case 52:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_52Hz;
			break;
			default:  //Set default to 104
			case 104:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_104Hz;
			break;
			case 208:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_208Hz;
			break;
			case 416:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_416Hz;
			break;
			case 833:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_833Hz;
			break;
			case 1660:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_1660Hz;
			break;
		}
	}
	else
	{
		//dataToWrite already = 0 (powerdown);
	}
	//Write the byte
	writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, dataToWrite);

	return returnError;
}

//****************************************************************************//
//
//  Accelerometer section
//
//****************************************************************************//
int16_t readRawAccelX( void )
{
	int16_t output = 0;
	status_t errorLevel = readRegisterInt16( &output, LSM6DS3_ACC_GYRO_OUTX_L_XL );
	if( errorLevel != IMU_SUCCESS )
	{
		if( errorLevel == IMU_ALL_ONES_WARNING )
		{
			allOnesCounter++;
		}
		else
		{
			nonSuccessCounter++;
		}
	}
	return output;
}
float readFloatAccelX( void )
{
	float output = calcAccel(readRawAccelX());
	return output;
}

int16_t readRawAccelY( void )
{
	int16_t output;
	status_t errorLevel = readRegisterInt16( &output, LSM6DS3_ACC_GYRO_OUTY_L_XL );
	if( errorLevel != IMU_SUCCESS )
	{
		if( errorLevel == IMU_ALL_ONES_WARNING )
		{
			allOnesCounter++;
		}
		else
		{
			nonSuccessCounter++;
		}
	}
	return output;
}
float readFloatAccelY( void )
{
	float output = calcAccel(readRawAccelY());
	return output;
}

int16_t readRawAccelZ( void )
{
	int16_t output;
	status_t errorLevel = readRegisterInt16( &output, LSM6DS3_ACC_GYRO_OUTZ_L_XL );
	if( errorLevel != IMU_SUCCESS )
	{
		if( errorLevel == IMU_ALL_ONES_WARNING )
		{
			allOnesCounter++;
		}
		else
		{
			nonSuccessCounter++;
		}
	}
	return output;
}
float readFloatAccelZ( void )
{
	float output = calcAccel(readRawAccelZ());
	return output;
}

float calcAccel( int16_t input )
{
	float output = (float)input * 0.061 * (settings.accelRange >> 1) / 1000;
	return output;
}

//****************************************************************************//
//
//  Gyroscope section
//
//****************************************************************************//
int16_t readRawGyroX( void )
{
	int16_t output;
	status_t errorLevel = readRegisterInt16( &output, LSM6DS3_ACC_GYRO_OUTX_L_G );
	if( errorLevel != IMU_SUCCESS )
	{
		if( errorLevel == IMU_ALL_ONES_WARNING )
		{
			allOnesCounter++;
		}
		else
		{
			nonSuccessCounter++;
		}
	}
	return output;
}
float readFloatGyroX( void )
{
	float output = calcGyro(readRawGyroX());
	return output;
}

int16_t readRawGyroY( void )
{
	int16_t output;
	status_t errorLevel = readRegisterInt16( &output, LSM6DS3_ACC_GYRO_OUTY_L_G );
	if( errorLevel != IMU_SUCCESS )
	{
		if( errorLevel == IMU_ALL_ONES_WARNING )
		{
			allOnesCounter++;
		}
		else
		{
			nonSuccessCounter++;
		}
	}
	return output;
}
float readFloatGyroY( void )
{
	float output = calcGyro(readRawGyroY());
	return output;
}

int16_t readRawGyroZ( void )
{
	int16_t output;
	status_t errorLevel = readRegisterInt16( &output, LSM6DS3_ACC_GYRO_OUTZ_L_G );
	if( errorLevel != IMU_SUCCESS )
	{
		if( errorLevel == IMU_ALL_ONES_WARNING )
		{
			allOnesCounter++;
		}
		else
		{
			nonSuccessCounter++;
		}
	}
	return output;
}
float readFloatGyroZ( void )
{
	float output = calcGyro(readRawGyroZ());
	return output;
}

float calcGyro( int16_t input )
{
	uint8_t gyroRangeDivisor = settings.gyroRange / 125;
	if ( settings.gyroRange == 245 ) {
		gyroRangeDivisor = 2;
	}

	float output = (float)input * 4.375 * (gyroRangeDivisor) / 1000;
	return output;
}

//****************************************************************************//
//
//  Temperature section
//
//****************************************************************************//
int16_t readRawTemp( void )
{
	int16_t output;
	readRegisterInt16( &output, LSM6DS3_ACC_GYRO_OUT_TEMP_L );
	return output;
}

float readTempC( void )
{
	float output = (float)readRawTemp() / 16; //divide by 16 to scale
	output += 25; //Add 25 degrees to remove offset

	return output;

}

float readTempF( void )
{
	float output = (float)readRawTemp() / 16; //divide by 16 to scale
	output += 25; //Add 25 degrees to remove offset
	output = (output * 9) / 5 + 32;

	return output;
}

//****************************************************************************//
//
//  FIFO section
//
//****************************************************************************//
void fifoBegin( void ) {
	//CONFIGURE THE VARIOUS FIFO SETTINGS
	//
	//
	//This section first builds a bunch of config words, then goes through
	//and writes them all.

	//Split and mask the threshold
	uint8_t thresholdLByte = settings.fifoThreshold & 0x00FF;
	uint8_t thresholdHByte = (settings.fifoThreshold & 0x0F00) >> 8;
	//Pedo bits not configured (ctl2)

	//CONFIGURE FIFO_CTRL3
	uint8_t tempFIFO_CTRL3 = 0;
	if (settings.gyroFifoEnabled == 1)
	{
		//Set up gyro stuff
		//Build on FIFO_CTRL3
		//Set decimation
		tempFIFO_CTRL3 |= (settings.gyroFifoDecimation & 0x07) << 3;

	}
	if (settings.accelFifoEnabled == 1)
	{
		//Set up accelerometer stuff
		//Build on FIFO_CTRL3
		//Set decimation
		tempFIFO_CTRL3 |= (settings.accelFifoDecimation & 0x07);
	}

	//CONFIGURE FIFO_CTRL4  (nothing for now-- sets data sets 3 and 4
	uint8_t tempFIFO_CTRL4 = 0;


	//CONFIGURE FIFO_CTRL5
	uint8_t tempFIFO_CTRL5 = 0;
	switch (settings.fifoSampleRate) {
		default:  //set default case to 10Hz(slowest)
		case 10:
		tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_10Hz;
		break;
		case 25:
		tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_25Hz;
		break;
		case 50:
		tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_50Hz;
		break;
		case 100:
		tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_100Hz;
		break;
		case 200:
		tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_200Hz;
		break;
		case 400:
		tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_400Hz;
		break;
		case 800:
		tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_800Hz;
		break;
		case 1600:
		tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_1600Hz;
		break;
		case 3300:
		tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_3300Hz;
		break;
		case 6600:
		tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_6600Hz;
		break;
	}
	//Hard code the fifo mode here:
	tempFIFO_CTRL5 |= settings.fifoModeWord = 6;  //set mode:

	//Write the data
	writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL1, thresholdLByte);
	//Serial.println(thresholdLByte, HEX);
	writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL2, thresholdHByte);
	//Serial.println(thresholdHByte, HEX);
	writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL3, tempFIFO_CTRL3);
	writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL4, tempFIFO_CTRL4);
	writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL5, tempFIFO_CTRL5);

}
void fifoClear( void ) {
	//Drain the fifo data and dump it
	while( (fifoGetStatus() & 0x1000 ) == 0 ) {
		fifoRead();
	}

}
int16_t fifoRead( void ) {
	//Pull the last data from the fifo
	uint8_t tempReadByte = 0;
	uint16_t tempAccumulator = 0;
	readRegister(&tempReadByte, LSM6DS3_ACC_GYRO_FIFO_DATA_OUT_L);
	tempAccumulator = tempReadByte;
	readRegister(&tempReadByte, LSM6DS3_ACC_GYRO_FIFO_DATA_OUT_H);
	tempAccumulator |= ((uint16_t)tempReadByte << 8);

	return tempAccumulator;
}

uint16_t fifoGetStatus( void ) {
	//Return some data on the state of the fifo
	uint8_t tempReadByte = 0;
	uint16_t tempAccumulator = 0;
	readRegister(&tempReadByte, LSM6DS3_ACC_GYRO_FIFO_STATUS1);
	tempAccumulator = tempReadByte;
	readRegister(&tempReadByte, LSM6DS3_ACC_GYRO_FIFO_STATUS2);
	tempAccumulator |= (tempReadByte << 8);

	return tempAccumulator;

}
void fifoEnd( void ) {
	// turn off the fifo
	writeRegister(LSM6DS3_ACC_GYRO_FIFO_STATUS1, 0x00);  //Disable
}

void lsm6ds3_init() {
	status_t status;
	status = beginCore();
	status = setDefaultSettings();
	status = begin();
}