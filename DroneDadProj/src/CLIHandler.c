/*
 * CLIHandler.c
 *
 * Created: 1/24/2018 7:34:10 PM
 *  Author: John
 */ 

#include "asf.h"

#include <string.h>

#include "CLIHandler.h"
#include "Version.h"
#include "adc_temp.h"

const char* CMD_HELP = "help";
const char* CMD_VER_BL = "ver_bl";
const char* CMD_VER_APP = "ver_app";
const char* CMD_GPIO_SET = "gpio_set";
const char* CMD_GPIO_CLEAR = "gpio_clear";
const char* CMD_GPIO_GET = "gpio_get";
const char* CMD_MAC = "mac";
const char* CMD_IP = "ip";
const char* CMD_READ_GYRO = "read_gyro";
const char* CMD_READ_ACCEL = "read_accel";
const char* CMD_SET_ACT = "set_act";
const char* CMD_CLEAR_ACT = "clear_act";
const char* CMD_ADC_GET = "adc_get";
const char* CMD_MCU_TEMP = "mcu_temp";
const char* CMD_I2C_SCAN = "i2c_scan";

extern struct adc_module adc_inst;
extern struct i2c_master_module i2c_master_instance;

void init_cmd_list() {
	cmd_list = (struct option_list) {
		{{ CMD_HELP, "", "Display all commands" } ,
		{ CMD_VER_BL, "", "Print bootloader firmware information." } ,
		{ CMD_VER_APP, "", "Print application version information." } ,
		{ CMD_GPIO_SET, "[port] [pin_num]", "Set GPIO pin at a given port high." } ,
		{ CMD_GPIO_CLEAR, "[port] [pin_num]", "Set GPIO pin at a given port low." } ,
		{ CMD_GPIO_GET, "[port] [pin_num]", "Get state of a GPIO pin.(Enter B 2 or B 3)" } ,
		{ CMD_MAC, "", "Print the MAC address." } ,
		{ CMD_IP, "", "Print the IP Address." } ,
		{ CMD_READ_GYRO, "[reading count] [interval in ms]", "Read from the Gyroscope sensor." } ,
		//{ CMD_READ_ACCEL, "[reading count] [interval in ms]", "Read from the Accelerometer sensor." } ,
		{ CMD_SET_ACT, "", "Turn the actuator (LED) on." } ,
		{ CMD_CLEAR_ACT, "", "Turn the actuator (LED) off." } ,
		{ CMD_ADC_GET, "[port] [pin_num]", "Get the ADC value of an input pin." } ,
		{ CMD_MCU_TEMP, "", "Print the temperature reading of the on-board MCU temperature sensor." } ,
		{ CMD_I2C_SCAN, "", "Print out list of addresses of I2C devices on bus." }}
	};
}

// TODO: Put this in a cleaner place
#define DATA_LENGTH 8
static uint8_t wr_buffer[DATA_LENGTH] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07
};

#define READ_DATA_LENGTH 14
static uint8_t rd_buffer[DATA_LENGTH];

#define SLAVE_ADDRESS (0x52 >> 1)
/// 0x52 0x53 0x29

#define MPU_6050_SLAVE_ADDR (0x68)

static struct i2c_master_packet wr_packet = {
	.address          = SLAVE_ADDRESS,
	.data_length      = DATA_LENGTH,
	.data             = wr_buffer,
	.ten_bit_address  = false,
	.high_speed       = false,
	.hs_master_code   = 0x00,
};

static struct i2c_master_packet rd_packet = {
	.address          = SLAVE_ADDRESS,
	.data_length      = READ_DATA_LENGTH,
	.data             = rd_buffer,
	.ten_bit_address  = false,
	.high_speed       = false,
	.hs_master_code   = 0x00,
};

static void disable_adc(void) {
	adc_disable(&adc_inst);
}

static void configure_adc(void) {
	struct adc_config config;
	
	adc_get_config_defaults(&config);
	config.clock_source = GCLK_GENERATOR_1;
	config.reference = ADC_REFERENCE_INTVCC1;
	config.clock_prescaler = ADC_CTRLB_PRESCALER_DIV16;
	config.resolution = ADC_RESOLUTION_12BIT;
	adc_init(&adc_inst, ADC, &config);
	adc_enable(&adc_inst);
}

// TODO: If arg1 or arg2 are non-digit, we need to throw an error.
void handle_user_input(char* input) {
	char* cmd = strtok(input, " ");

	if(strcmp(CMD_HELP, cmd) == 0) { 
		handle_help(); 
	}
	else if(strcmp(CMD_VER_BL, cmd) == 0) { 
		handle_ver_bl(); 
	}
	else if(strcmp(CMD_VER_APP, cmd) == 0) { 
		handle_ver_app();
	}
	else if(strcmp(CMD_GPIO_SET, cmd) == 0) { 
		char* arg1 = strtok(NULL, " ");
		char* arg2 = strtok(NULL, " ");
		
		if(arg1 == 0 || arg2 == 0) {
			printf("Not enough arguments! Try \"help\".\r\n");
			return;
		}

		char* port = arg1;
		int pin_num = atoi(arg2);

		if ((strcmp(port,"A") != 0) && (strcmp(port,"B") != 0)){
			printf("Enter a valid Port! Try \"help\".\r\n");
		}

		if (pin_num <= 0 && pin_num > 32) {
			printf("Enter a valid Pin Number! Try \"help\".\r\n");
		}
		
		handle_gpio_set(*port, pin_num);
	}
	else if(strcmp(CMD_GPIO_CLEAR, cmd) == 0) { 
		char* arg1 = strtok(NULL, " ");
		char* arg2 = strtok(NULL, " ");
		
		if(arg1 == 0 || arg2 == 0) {
			printf("Not enough arguments! Try \"help\".\r\n");
			return;
		}

		char* port = arg1;
		int pin_num = atoi(arg2);

		if ((strcmp(port,"A") != 0) && (strcmp(port,"B") != 0)){
			printf("Enter a valid Port! Try \"help\".\r\n");
		}

		if (pin_num <= 0 && pin_num > 32) {
			printf("Enter a valid Pin Number! Try \"help\".\r\n");
		}
		
		handle_gpio_clear(*port, pin_num); 
	}
	else if(strcmp(CMD_GPIO_GET, cmd) == 0) {
		char* arg1 = strtok(NULL, " ");
		char* arg2 = strtok(NULL, " ");
		
		if(arg1 == 0 || arg2 == 0) {
			printf("Not enough arguments! Try \"help\".\r\n");
			return;
		}

		char* port = arg1;
		int pin_num = atoi(arg2);

		if ((strcmp(port,"A") != 0) && (strcmp(port,"B") != 0)){
			printf("Enter a valid Port! Try \"help\".\r\n");
		}

		if (pin_num <= 0 && pin_num > 32) {
			printf("Enter a valid Pin Number! Try \"help\".\r\n");
		}

		handle_gpio_get(*port, pin_num);
	}
	else if(strcmp(CMD_MAC, cmd) == 0) { 
		handle_mac(); 
	}
	else if(strcmp(CMD_IP, cmd) == 0) { 
		handle_ip(); 
	}
	else if(strcmp(CMD_READ_GYRO, cmd) == 0) {
		char* arg1 = strtok(NULL, " ");
		char* arg2 = strtok(NULL, " ");
		
		if(arg1 == 0 || arg2 == 0) {
			printf("Not enough arguments! Try \"help\".\r\n");
			return;
		}
				
		int num_readings = atoi(arg1);
		int interval_ms = atoi(arg2);
		
		handle_read_gyro(num_readings, interval_ms); 
	}
	else if(strcmp(CMD_READ_ACCEL, cmd) == 0) {
		char* arg1 = strtok(NULL, " ");
		char* arg2 = strtok(NULL, " ");
		
		if(arg1 == 0 || arg2 == 0) {
			printf("Not enough arguments! Try \"help\".\r\n");
			return;
		}
				
		int num_readings = atoi(arg1);
		int interval_ms = atoi(arg2);
		
		handle_read_accel(num_readings, interval_ms); 
	}
	else if(strcmp(CMD_ADC_GET, cmd) == 0) {
		char* arg1 = strtok(NULL, " ");
		char* arg2 = strtok(NULL, " ");
		
		if(arg1 == 0 || arg2 == 0) {
			printf("Not enough arguments! Try \"help\".\r\n");
			return;
		}

		char* port = arg1;
		int pin_num = atoi(arg2);

		if ((strcmp(port,"A") != 0) && (strcmp(port,"B") != 0)){
			printf("Enter a valid Port! Try \"help\".\r\n");
		}

		if (pin_num <= 0 && pin_num > 32) {
			printf("Enter a valid Pin Number! Try \"help\".\r\n");
		}
		
		handle_adc_get(port, pin_num); 
	}
	else if(strcmp(CMD_MCU_TEMP, cmd) == 0) { 
		handle_mcu_temp(); 
	}
	else if(strcmp(CMD_I2C_SCAN, cmd) == 0) { 
		handle_i2c_scan(); 
	}
	else if(strcmp(CMD_SET_ACT, cmd) == 0) {
		// Just a GPIO.
		handle_gpio_set('B', PIN_PB02);	
	}
	else if(strcmp(CMD_CLEAR_ACT, cmd) == 0) {
		// Just a GPIO.		
		handle_gpio_clear('B', PIN_PB02);	
	}
	else {
		printf("Invalid command! Type \"help\" for a list of available commands.\r\n");
	}
}

void handle_help() {
	printf("Command List:\r\n");
	for(int i = 0; i < OPTION_COUNT; i++) {
		printf("\t%s %s\r\n\t\t%s\r\n", cmd_list.options[i].command, 
			cmd_list.options[i].params,
			cmd_list.options[i].description);
	}
}


void handle_ver_bl() {
	printf("Bootloader Version: %s\r\n", BL_VERSION_STRING);
}

void handle_ver_app() {
	printf("Application Version: %s\r\n", APP_VERSION_STRING);
}

void handle_gpio_set(char port, int pin_num) {
	// TODO: Must implement this feature for at least two pins.
	
	// Hard-coded testing
	uint8_t pin = PIN_PB02;
	bool level = HIGH;
	port_pin_set_output_level(pin, level);
	
	//printf("Not implemented yet!\r\n");
}

void handle_gpio_clear(char port, int pin_num) {
	// TODO: Must implement this feature for at least two pins.
	
	// Hard-coded testing
	uint8_t pin = PIN_PB02;
	bool level = LOW;
	port_pin_set_output_level(pin, level);
		
	//printf("Not implemented yet!\r\n");
}

void handle_gpio_get(char port, int pin_num) {
	// TODO: Must implement this feature for at least two pins.
		uint8_t pin = PIN_PB02;
		bool level;

	// Hard-coded testing
	if (port == "B" && pin_num == 2){
		level = port_pin_get_output_level(pin);
	}

	else if (port == "B" && pin_num == 3){
		uint8_t pin = PIN_PB03;
		level = port_pin_get_output_level(pin);
	}
	
	printf("The level set is %d\r\n", level);

}

void handle_mac() {
	char* dummy_mac = "01:23:45:67:89:ab";
	printf("MAC Address: %s\r\n", dummy_mac);
}

void handle_ip() {
	char* dummy_ip = "255.255.255.255";
	printf("IP Address: %s\r\n", dummy_ip);
}

void handle_read_gyro(int num_readings, int interval_ms) {
	enum status_code i2c_status;
	
	for(int i = 0; i < num_readings; i++) {
		// Wake up the device
		wr_packet.address     = MPU_6050_SLAVE_ADDR;
		wr_packet.data_length = 1;
		wr_buffer[0]          = 0x6B;
		wr_packet.data        = wr_buffer;
		rd_packet.address = MPU_6050_SLAVE_ADDR;
		i2c_status = i2c_master_write_packet_wait_no_stop(&i2c_master_instance, &wr_packet);
		if( i2c_status == STATUS_OK ) {
			i2c_status = i2c_master_read_packet_wait(&i2c_master_instance, &rd_packet);
		}
		i2c_master_send_stop(&i2c_master_instance);
	
		rd_packet.data[0] = rd_packet.data[0] & (0 << 6);
	
		// Write new sleep bit (off)
		wr_packet.address     = MPU_6050_SLAVE_ADDR;
		wr_packet.data_length = 2;
		wr_buffer[0]          = 0x6B;
		wr_buffer[1]          = rd_packet.data[0];
		wr_packet.data        = wr_buffer;
		rd_packet.address = MPU_6050_SLAVE_ADDR;
		i2c_status = i2c_master_write_packet_wait_no_stop(&i2c_master_instance, &wr_packet);
		i2c_master_send_stop(&i2c_master_instance);
	
		// Read back the Power management Register 
		wr_packet.address     = MPU_6050_SLAVE_ADDR;
		wr_packet.data_length = 1;
		wr_buffer[0]          = 0x6B;
		wr_packet.data        = wr_buffer;
		rd_packet.address = MPU_6050_SLAVE_ADDR;
		i2c_status = i2c_master_write_packet_wait_no_stop(&i2c_master_instance, &wr_packet);
		if( i2c_status == STATUS_OK ) {
			i2c_status = i2c_master_read_packet_wait(&i2c_master_instance, &rd_packet);
		}
		i2c_master_send_stop(&i2c_master_instance);
		
		// Get the data.
		wr_packet.address     = MPU_6050_SLAVE_ADDR;
		wr_packet.data_length = 1;
		wr_buffer[0]          = 0x3B; //MPU6050_RA_ACCEL_XOUT_H;
		wr_packet.data        = wr_buffer;
		rd_packet.address = MPU_6050_SLAVE_ADDR;
		i2c_status = i2c_master_write_packet_wait_no_stop(&i2c_master_instance, &wr_packet);
		if( i2c_status == STATUS_OK ) {
			i2c_status = i2c_master_read_packet_wait(&i2c_master_instance, &rd_packet);
		}
	
		int16_t ax, ay, az, gx, gy, gz;
		ax = (((int16_t)rd_packet.data[0]) << 8) | rd_packet.data[1];
		ay = (((int16_t)rd_packet.data[2]) << 8) | rd_packet.data[3];
		az = (((int16_t)rd_packet.data[4]) << 8) | rd_packet.data[5];
		gx = (((int16_t)rd_packet.data[8]) << 8) | rd_packet.data[9];
		gy = (((int16_t)rd_packet.data[10]) << 8) | rd_packet.data[11];
		gz = (((int16_t)rd_packet.data[12]) << 8) | rd_packet.data[13];
	
		printf("===Reading %d===\r\n", i);
		printf("Ax = %d\r\nAy = %d\r\nAz = %d\r\nGx = %d\r\nGy = %d\r\nGz = %d\r\n", ax, ay, az, gx, gy, gz);
		i2c_master_send_stop(&i2c_master_instance);
		delay_ms(interval_ms);
	}
}

void handle_read_accel(int num_readings, int interval_ms) {
	printf("Use read_gyro!\r\n");
}

void handle_adc_get(char port, int pin_num) {
	disable_adc();
	configure_adc();
	
	adc_start_conversion(&adc_inst);
	uint16_t result;
	
	do {
		/* Wait */
	} while(adc_read(&adc_inst, &result) == STATUS_BUSY);
	
	printf("ADC Result: %d\r\n", result);
}

void handle_mcu_temp() {
	disable_adc();
	
	system_voltage_reference_enable(SYSTEM_VOLTAGE_REFERENCE_TEMPSENSE);
	
	configure_adc_temp();
	
	adc_start_conversion(&adc_inst);
	uint16_t result;
	
	do {
		/* Wait */
	} while(adc_read(&adc_inst, &result) == STATUS_BUSY);
	
	int temp = calculate_temperature(result);
	
	printf("Temperature Result: %d \r\n", temp);
}

void handle_i2c_scan() {
	
	  for(char slave_address = 1; slave_address <= 127; slave_address++) {
		  enum status_code i2c_status;
		  wr_packet.address     = slave_address;
		  wr_packet.data_length = 1;
		  wr_buffer[0]          = 0x00;
		  wr_packet.data        = wr_buffer;
		  i2c_status = i2c_master_write_packet_wait_no_stop(&i2c_master_instance, &wr_packet);
		  printf("Addr is %d \r\n", slave_address);
		  if( i2c_status == STATUS_OK ) {
			  //i2c_status = i2c_master_read_packet_wait(&i2c_master_instance, &rd_packet);
			 printf("The slave address is %.2x \r\n", slave_address);
		  }
		  i2c_master_send_stop(&i2c_master_instance);
	 }
	
}
