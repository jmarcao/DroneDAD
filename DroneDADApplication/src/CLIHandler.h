/*
 * CLIHandler.h
 *
 * Created: 1/24/2018 7:31:49 PM
 *  Author: John
 */ 

#ifndef CLIHANDLER_H_
#define CLIHANDLER_H_

#include "SerialConsole.h"
#include "GyroscopeDriver.h"

#define OPTION_COUNT 13

struct option {
	const char* command;
	const char* params;
	const char* description;
};

struct option_list {
	struct option options[OPTION_COUNT];
} cmd_list;

/* Max size of UART buffer. */
#define INPUT_BUFFER_SIZE 256

/** UART buffer. */
static char uart_buffer[INPUT_BUFFER_SIZE];

/** Written size of UART buffer. */
static int uart_buffer_written = 0;

/** A buffer of character from the serial. */
static uint16_t uart_ch_buffer;

void init_cmd_list();

void handle_user_input(char* input);

void handle_help();
void handle_ver_bl();
void handle_ver_app();
void handle_gpio_set(char port, int pin_num);
void handle_gpio_clear(char port, int pin_num);
void handle_gpio_get(char port, int pin_num);
void handle_mac();
void handle_ip();
void handle_read_gyro(int num_readings, int interval_ms);
void handle_adc_get(char port, int pin_num);
void handle_mcu_temp();
void handle_i2c_scan();
void handle_led_set(int ledBank, int mode);
void handle_get_data();

/**
 * \brief Callback of USART input.
 *
 * \param[in] module USART module structure.
 */
static void uart_callback(const struct usart_module *const module) {
	/* If input string is bigger than buffer size limit, ignore the excess part. */
	if (uart_buffer_written < INPUT_BUFFER_SIZE) {
		printf("%c", uart_ch_buffer);
		if(uart_ch_buffer == '\x7f') {
			uart_buffer_written--;
		}
		else {
			uart_buffer[uart_buffer_written++] = uart_ch_buffer & 0xFF;
		}
	}
}

static void set_uart_callback() {
	usart_register_callback(&cdc_uart_module, (usart_callback_t)uart_callback, USART_CALLBACK_BUFFER_RECEIVED);
}

static void check_usart_buffer() {
	int i;
	
	/* Publish the input string when newline was received or input string is bigger than buffer size limit. */
	if (uart_buffer_written >= INPUT_BUFFER_SIZE) {
		printf("Too many characters!\r\n");
		printf("Try Again.");
	} else {
		for (i = 0; i < uart_buffer_written; i++) {
			/* Find newline character ('\n' or '\r\n') and publish the previous string . */
			if (uart_buffer[i] == '\n' || uart_buffer[i] == '\r' ) {
				// Get rid of newline/linefeed
				uart_buffer[i] = '\0';
				if(i == 0) { // ie: User hit enter with no text
					printf("\r\nDroneDAD > ");
				}
				else { // There is text to handle
					handle_user_input(uart_buffer);
				}
				/* Move remain data to start of the buffer. */
				if (uart_buffer_written > i + 1) {
					memmove(uart_buffer, uart_buffer + i + 1, uart_buffer_written - i - 1);
					uart_buffer_written = uart_buffer_written - i - 1;
					} else {
					uart_buffer_written = 0;
				}

				break;
			}
		}
	}
}

/*
 * CLIHandler.c
 *
 * Created: 1/24/2018 7:34:10 PM
 *  Author: John
 */ 

#include "asf.h"

#include <string.h>

#include "CLIHandler.h"
#include "adc_temp.h"
#include "NVMStorage.h"
#include "FlashStorage.h"
#include "dd_mqtt.h"

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
const char* CMD_LED_SET = "led_set";
const char* CMD_GET_DATA = "get_data";

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
		{ CMD_LED_SET, "[bank {0,1}] [mode {on, off, dim0, dim1]", "Set the LED banks." } ,
		{ CMD_SET_ACT, "", "Turn the actuator (LED) on." } ,
		{ CMD_CLEAR_ACT, "", "Turn the actuator (LED) off." } ,
		{ CMD_MCU_TEMP, "", "Print the temperature reading of the on-board MCU temperature sensor." } ,
		{ CMD_I2C_SCAN, "", "Print out list of addresses of I2C devices on bus." } ,
		{ CMD_GET_DATA, "", "Prints out the current Roll, Pitch, and Stall Angle."}}
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
	printf("Handling user input: %s\r\n", input);
	char* cmd = strtok(input, " ");
	
	printf("cmd: %s\r\n", cmd);

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
	else if(strcmp(CMD_LED_SET, cmd) == 0) {
		char* arg1 = strtok(NULL, " ");
		char* arg2 = strtok(NULL, " ");
		
		if(arg1 == 0 || arg2 == 0) {
			printf("Not enough arguments! Try \"help\".\r\n");
			return;
		}
		
		if(!((strcmp(arg1, "0") == 0) || (strcmp(arg1, "1") == 0))) {
			printf("Invalid bank argument!\r\n");
			return;
		}
		
		if(!((strcmp(arg2, "on") == 0) ||
			 (strcmp(arg2, "off") == 0) ||
			 (strcmp(arg2, "dim0") == 0) ||
			 (strcmp(arg2, "dim1") == 0)))
		{
			printf("Invalid setting argument!\r\n");
			return;
		}

		uint8_t bank = 0;
		uint8_t mode = 0;
		
		if(strcmp(arg1, "0") == 0) {
			bank = LED_BANK_1_REG;
		}
		else if(strcmp(arg1, "1") == 0) {
			bank = LED_BANK_2_REG;
		}
		
		if(strcmp(arg2, "on") == 0) {
			mode = LED_ON;
		}
		else if(strcmp(arg2, "off") == 0) {
			mode = LED_OFF;
		}
		else if(strcmp(arg2, "dim0") == 0) {
			mode = LED_DIM0;
		}
		else if(strcmp(arg2, "dim1") == 0) {
			mode = LED_DIM1;
		}
		
		handle_led_set(bank, mode);
	}
	else if(strcmp(cmd, CMD_GET_DATA) == 0) {
		handle_get_data();
	}
	else {
		printf("Invalid command! Type \"help\" for a list of available commands.\r\n");
	}
	
	printf("DroneDAD > ");
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
	struct boot_status bs;
	get_boot_status(&bs);
	printf("Bootloader Version: %d.%d.%d\r\n", bs.signature[5], bs.signature[6], bs.signature[7]); // TODO
}

void handle_ver_app() {
	struct application_metadata am;
	get_application_metadata(0, &am);
	printf("Application Version: %x\r\n", am.crc);
}

void handle_gpio_set(char port, int pin_num) {
	// Hard-coded testing
	uint8_t pin = PIN_PB02;
	bool level = HIGH;
	port_pin_set_output_level(pin, level);
}

void handle_gpio_clear(char port, int pin_num) {
	// Hard-coded testing
	uint8_t pin = PIN_PB02;
	bool level = LOW;
	port_pin_set_output_level(pin, level);
}

void handle_gpio_get(char port, int pin_num) {
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
	char mac_addr[6];
	m2m_wifi_get_mac_address(&mac_addr);

	printf("MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
		mac_addr[0], mac_addr[1], mac_addr[2],
		mac_addr[3], mac_addr[4], mac_addr[5]);
}

void handle_ip() {
	printf("Wi-Fi IP is %u.%u.%u.%u\r\n",
			ip_addr[0], ip_addr[1], ip_addr[2], ip_addr[3]);
}

void handle_read_gyro(int num_readings, int interval_ms) {
	for(int i = 0; i < num_readings; i++) {
		
		struct lsm6ds3_output_data data;
		lsm6ds3_readAllData(&data);
		
		printf("===Reading===\r\n");
		printf("Ax: %f\r\n", data.ax);
		printf("Ay: %f\r\n", data.ay);
		printf("Az: %f\r\n", data.az);
		printf("Gx: %f\r\n", data.gx);
		printf("Gy: %f\r\n", data.gy);
		printf("Gz: %f\r\n", data.gz);
		
		delay_ms(interval_ms);
	}
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
	//disable_adc();
	
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
		  if( i2c_status == STATUS_OK ) {
			  //i2c_status = i2c_master_read_packet_wait(&i2c_master_instance, &rd_packet);
			 printf("The slave address is %.2x \r\n", slave_address);
		  }
		  if(i2c_status == STATUS_ERR_TIMEOUT) {
			  printf("Timeout!");
		  }
		  i2c_master_send_stop(&i2c_master_instance);
	 }
	 int i = 0;
	
}

void handle_led_set(int ledBank, int mode) {
	lp3944_set_led_bank(ledBank, mode);
}

void handle_get_data() {
	printf("Stall Angle %d\r\n", stallAngle);
	printf("Pitch: %d\r\nRoll: %d\r\n", abs(int_pitch_deg), abs(int_roll_deg));
}

#endif