/*
 * MPU9150Driver.h
 *
 * Created: 4/18/2018 9:55:27 PM
 *  Author: John
 */ 


#ifndef MPU9150DRIVER_H_
#define MPU9150DRIVER_H_

#define MPU_9150_SLAVE_ADDR (0x68)

#define DATA_LENGTH 16
static uint8_t mpu_wr_buffer[DATA_LENGTH];
static uint8_t mpu_rd_buffer[DATA_LENGTH];

extern struct i2c_master_module i2c_master_instance;

static struct i2c_master_packet mpu_wr_packet = {
	.address          = MPU_9150_SLAVE_ADDR,
	.data_length      = DATA_LENGTH,
	.data             = mpu_wr_buffer,
	.ten_bit_address  = false,
	.high_speed       = false,
	.hs_master_code   = 0x00,
};

static struct i2c_master_packet mpu_rd_packet = {
	.address          = MPU_9150_SLAVE_ADDR,
	.data_length      = DATA_LENGTH,
	.data             = mpu_rd_buffer,
	.ten_bit_address  = false,
	.high_speed       = false,
	.hs_master_code   = 0x00,
};

// Struct for holding the data read from the LSM6DS3
struct mpu9150_output_data {
	// Acceleration
	int16_t ax;
	int16_t ay;
	int16_t az;
	// Gyroscope
	int16_t gx;
	int16_t gy;
	int16_t gz;
};

static float calc_pitch(struct mpu9150_output_data* data) {
	
}

static bool detect_mpu9150() {
	bool ret = false;
	// Wake up the device
	mpu_wr_packet.address     = MPU_9150_SLAVE_ADDR;
	mpu_wr_packet.data_length = 1;
	mpu_wr_buffer[0]          = 0x00;
	mpu_wr_packet.data        = mpu_wr_buffer;
	mpu_rd_packet.address = MPU_9150_SLAVE_ADDR;
	enum status_code i2c_status = i2c_master_write_packet_wait(&i2c_master_instance, &mpu_wr_packet);
	if(i2c_status == STATUS_OK) {
		printf("MPU Present!\r\n");
		ret = true;
	}
	else {
		printf("MPU NOT Present.\r\n");
	}
	i2c_master_send_stop(&i2c_master_instance);
	
	return ret;
}

static void get_mpu9150_reading(struct mpu9150_output_data* data) {
	enum status_code i2c_status;
	
	// Wake up the device
	mpu_wr_packet.address     = MPU_9150_SLAVE_ADDR;
	mpu_wr_packet.data_length = 1;
	mpu_wr_buffer[0]          = 0x6B;
	mpu_wr_packet.data        = mpu_wr_buffer;
	mpu_rd_packet.address = MPU_9150_SLAVE_ADDR;
	i2c_status = i2c_master_write_packet_wait_no_stop(&i2c_master_instance, &mpu_wr_packet);
	if( i2c_status == STATUS_OK ) {
		i2c_status = i2c_master_read_packet_wait(&i2c_master_instance, &mpu_rd_packet);
	}
	else {
		printf("i2c failed (%d)\r\n", i2c_status);
	}
	i2c_master_send_stop(&i2c_master_instance);
		
	mpu_rd_packet.data[0] = mpu_rd_packet.data[0] & (0 << 6);
		
	// Write new sleep bit (off)
	mpu_wr_packet.address     = MPU_9150_SLAVE_ADDR;
	mpu_wr_packet.data_length = 2;
	mpu_wr_buffer[0]          = 0x6B;
	mpu_wr_buffer[1]          = mpu_rd_packet.data[0];
	mpu_wr_packet.data        = mpu_wr_buffer;
	mpu_rd_packet.address = MPU_9150_SLAVE_ADDR;
	i2c_status = i2c_master_write_packet_wait_no_stop(&i2c_master_instance, &mpu_wr_packet);
	if(i2c_status != STATUS_OK) {
		printf("i2c failed (%d)\r\n", i2c_status);
	}
	i2c_master_send_stop(&i2c_master_instance);
		
	// Read back the Power management Register
	mpu_wr_packet.address     = MPU_9150_SLAVE_ADDR;
	mpu_wr_packet.data_length = 1;
	mpu_wr_buffer[0]          = 0x6B;
	mpu_wr_packet.data        = mpu_wr_buffer;
	mpu_rd_packet.address = MPU_9150_SLAVE_ADDR;
	i2c_status = i2c_master_write_packet_wait_no_stop(&i2c_master_instance, &mpu_wr_packet);
	if( i2c_status == STATUS_OK ) {
		i2c_status = i2c_master_read_packet_wait(&i2c_master_instance, &mpu_rd_packet);
	}
	else {
		printf("i2c failed (%d)\r\n", i2c_status);
	}
	i2c_master_send_stop(&i2c_master_instance);
		
	// Get the data.
	mpu_wr_packet.address     = MPU_9150_SLAVE_ADDR;
	mpu_wr_packet.data_length = 1;
	mpu_wr_buffer[0]          = 0x3B; //MPU6050_RA_ACCEL_XOUT_H;
	mpu_wr_packet.data        = mpu_wr_buffer;
	mpu_rd_packet.address = MPU_9150_SLAVE_ADDR;
	i2c_status = i2c_master_write_packet_wait_no_stop(&i2c_master_instance, &mpu_wr_packet);
	if( i2c_status == STATUS_OK ) {
		i2c_status = i2c_master_read_packet_wait(&i2c_master_instance, &mpu_rd_packet);
	}
	else {
		printf("i2c failed (%d)\r\n", i2c_status);
	}
	i2c_master_send_stop(&i2c_master_instance);
		
	int16_t ax, ay, az, gx, gy, gz;
	data->ax = (((int16_t)mpu_rd_packet.data[0]) << 8) | mpu_rd_packet.data[1];
	data->ay = (((int16_t)mpu_rd_packet.data[2]) << 8) | mpu_rd_packet.data[3];
	data->az = (((int16_t)mpu_rd_packet.data[4]) << 8) | mpu_rd_packet.data[5];
	data->gx = (((int16_t)mpu_rd_packet.data[8]) << 8) | mpu_rd_packet.data[9];
	data->gy = (((int16_t)mpu_rd_packet.data[10]) << 8) | mpu_rd_packet.data[11];
	data->gz = (((int16_t)mpu_rd_packet.data[12]) << 8) | mpu_rd_packet.data[13];
	
	
	printf("===Reading===\r\n");
	printf("Ax = %d\r\nAy = %d\r\nAz = %d\r\nGx = %d\r\nGy = %d\r\nGz = %d\r\n", 
		data->ax, data->ay, data->az, data->gx, data->gy, data->gz);
}





#endif /* MPU9150DRIVER_H_ */