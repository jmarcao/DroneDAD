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

#include "HttpDownloader.h"
#include "LEDDriver.h"
#include "GyroscopeDriver.h"
#include "FlashStorage.h"
#include "SerialConsole.h"
#include "dd_mqtt.h"
#include "CLIHandler.h"
#include "adc_temp.h"

void initDroneDAD() {
	// Basic System Components
	system_init();
	uartConsole_init();
	init_cmd_list();
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

void main_process_loop() {
	set_uart_callback();
	usart_enable_callback(&cdc_uart_module, USART_CALLBACK_BUFFER_RECEIVED);
	
	while(1) {
		mqtt_process();
		usart_read_job(&cdc_uart_module, &uart_ch_buffer);
		check_usart_buffer();
	}
}

int main(void) {
	// Run all Basic Initialization
	initDroneDAD();
		
	tstrWifiInitParam param;
	int8_t ret;

	printf("=== DroneDAD\r\n");
	printf("=== Compiled %s\r\n", __TIMESTAMP__);
	
	set_all_leds(LED_DIM1); // Initial LED state.
	
	// Start MQTT Services.
	mqtt_start();
	
	main_process_loop();

	printf("main: done.\r\n");
	while (1) {
	} /* Loop forever. */

	return 0;
}
