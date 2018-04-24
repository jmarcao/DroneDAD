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
#include "CLIHandler.h"
#include "dd_mqtt.h"
#include "adc_temp.h"

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
		
	tstrWifiInitParam param;
	int8_t ret;

	printf("=== DroneDAD ===\r\n");
	
	set_all_leds(LED_DIM1); // Initial LED state.
	
	/*
	struct lsm6ds3_output_data data;
	while(1) {
		if(lsm6ds3_dataReady()) {
			lsm6ds3_readAllData(&data);
			printf("ax=%d\t", (int16_t)data.ax);
			printf("ay=%d\t", (int16_t)data.ay);
			printf("az=%d\t", (int16_t)data.az);
			printf("gx=%d\t", (int16_t)data.gx);
			printf("gy=%d\t", (int16_t)data.gy); 
			printf("gz=%d\r\n", (int16_t)data.gz);
		}
	}
	*/
	
	// Enter the mqtt loop. Ideally we would add CLI via callback here.
	handleUpdateRequest();
	// dd_mqtt_loop();

	printf("main: done.\r\n");
	while (1) {
	} /* Loop forever. */

	return 0;
}
