/*
Application code
*/

#include <asf.h>
#include <at25dfx_hal.h>
#include <crc32.h>
#include <errno.h>

#include "main.h"

#include "driver/include/m2m_wifi.h"
#include "socket/include/socket.h"
#include "iot/http/http_client.h"

#include "LEDDriver.h"
#include "GyroscopeDriver.h"
#include "HttpDownloader.h"
#include "FlashStorage.h"
#include "SerialConsole.h"
#include "dd_mqtt.h"
#include "CLIHandler.h"
#include "adc_temp.h"

void dd_app_example_test(void) {
	// Init Button
	struct port_config config_port_pin;
	port_get_config_defaults(&config_port_pin);
	config_port_pin.direction = PORT_PIN_DIR_INPUT;
	port_pin_set_config(PIN_PB23, &config_port_pin);

	// Init LED
	struct port_config config_port_pin2;
	port_get_config_defaults(&config_port_pin2);
	config_port_pin2.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(PIN_PA23, &config_port_pin2);

	printf("Begining test of Flash RW...\r\n");
	for(int i = 0; i < AT25DFX_BUFFER_SIZE; i++) {
		write_buffer[i] = i;
	}
	/* Insert application code here, after the board has been initialized. */
	at25dfx_chip_wake(&at25dfx_chip);
	
	if (at25dfx_chip_check_presence(&at25dfx_chip) != STATUS_OK) {
		// Handle missing or non-responsive device
		printf("Flash chip missing!\r\n");
		exit(1);
	}
	at25dfx_chip_read_buffer(&at25dfx_chip, 0x0000, read_buffer, AT25DFX_BUFFER_SIZE);
	at25dfx_chip_set_sector_protect(&at25dfx_chip, 0x10000, false);
	at25dfx_chip_erase_block(&at25dfx_chip, 0x10000, AT25DFX_BLOCK_SIZE_4KB);
	at25dfx_chip_write_buffer(&at25dfx_chip, 0x10000, write_buffer, AT25DFX_BUFFER_SIZE);
	at25dfx_chip_set_global_sector_protect(&at25dfx_chip, true);
	at25dfx_chip_read_buffer(&at25dfx_chip, 0x10000, read_buffer, AT25DFX_BUFFER_SIZE);
	at25dfx_chip_sleep(&at25dfx_chip);
	
	uint32_t read_crc_res = 0;
	uint32_t write_crc_res = 0;
	enum status_code crcstat;
	crcstat = crc32_recalculate(&read_buffer[0], AT25DFX_BUFFER_SIZE, &read_crc_res);
	crcstat = crc32_recalculate(&write_buffer[0], AT25DFX_BUFFER_SIZE, &write_crc_res);
	printf("CRC32 Result of read: %x\r\n", read_crc_res);
	printf("CRC32 Result of write: %x\r\n", write_crc_res);
	
	printf("Starting test of NVM RW...\r\n");
	// NVM
	uint8_t write_page_buffer[NVMCTRL_PAGE_SIZE];
	uint8_t read_page_buffer[NVMCTRL_PAGE_SIZE];
	for (uint32_t i = 0; i < NVMCTRL_PAGE_SIZE; i++) {
		write_page_buffer[i] = i;
	}
	enum status_code error_code;
	do
	{
		uint32_t addr = 100 * NVMCTRL_ROW_PAGES * NVMCTRL_PAGE_SIZE;
		addr -= 1;
		addr += 1;
		error_code = nvm_erase_row(addr);
	} while (error_code == STATUS_BUSY);
	do
	{
		error_code = nvm_write_buffer( 100 * NVMCTRL_ROW_PAGES * NVMCTRL_PAGE_SIZE, write_page_buffer, NVMCTRL_PAGE_SIZE);
	} while (error_code == STATUS_BUSY);
	for (uint32_t i = 0; i < NVMCTRL_PAGE_SIZE; i++) {
		read_page_buffer[i] = 0;
	}
	do
	{
		error_code = nvm_read_buffer(100 * NVMCTRL_ROW_PAGES * NVMCTRL_PAGE_SIZE, read_page_buffer, NVMCTRL_PAGE_SIZE);
	} while (error_code == STATUS_BUSY);
	
	read_crc_res = 0;
	write_crc_res = 0;
	crcstat = crc32_recalculate(&read_page_buffer[0], NVMCTRL_PAGE_SIZE, &read_crc_res);
	crcstat = crc32_recalculate(&write_page_buffer[0], NVMCTRL_PAGE_SIZE, &write_crc_res);
	printf("CRC32 Result of read: %x\r\n", read_crc_res);
	printf("CRC32 Result of write: %x\r\n", write_crc_res);
	
	printf("RW Tests done... press the debug button for the LED.\r\n");
	while(1) {
		// LED light up with Button Press.
		bool level = port_pin_get_input_level(PIN_PB23);
		if(level == false) {
			port_pin_set_output_level(PIN_PA23, false);
		}
		else {
			port_pin_set_output_level(PIN_PA23,  true);
		}
	}
}

void initDroneDAD() {
	// Basic System Components
	system_init();
	uartConsole_init();
	i2c_init();
	delay_init();
	
	// Storage
	at25dfx_init();
	nvm_init();
	
	// Sensor/Actuator
	lp3944_init();
	lsm6ds3_init();
	
	// Wifi
	//wifiState_init();
	//configure_timer();
	nm_bsp_init();
	
	// MQTT
	configure_mqtt();
}

int main(void) {
	// Run all Basic Initialization
	initDroneDAD();
/*
	set_led(LP3944_LED1, LED_ON);
	
	struct lsm6ds3_output_data data;
	while(1) {
		if(lsm6ds3_dataReady()) {
			lsm6ds3_readAllData(&data);
			printf("===Reading===\r\n");
			printf("Ax = %f\r\nAy = %f\r\nAz = %f\r\nGx = %f\r\nGy = %f\r\nGz = %f\r\n", data.ax, data.ay, data.az, data.gx, data.gy, data.gz);
		}
	}
*/
		
	tstrWifiInitParam param;
	int8_t ret;

	printf("=== DroneDAD ===\r\n");
	
	/*
	init_cmd_list(); // Creates the help struct.
	char input[256];
		
	printf("=== DroneDAD CLI Interface ===\r\n");
	while (1) {
		printf("> ");
		scanf("%[^\r\n]%*c", input);
		handle_user_input(input);
	}
	*/
	
	dd_mqtt_loop();
	
	handleUpdateRequest();
	
	unsigned char test[0x1000];
	dd_flash_read_data(0x2000, &test[0], 0x1000);

	printf("main: done.\r\n");
	while (1) {
	} /* Loop forever. */

	return 0;
}
