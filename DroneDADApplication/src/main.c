/*
Application code
*/

#include <asf.h>

int main (void)
{
	system_init();

	/* Insert application code here, after the board has been initialized. */

// Button
struct port_config config_port_pin;
port_get_config_defaults(&config_port_pin);
config_port_pin.direction = PORT_PIN_DIR_INPUT;
port_pin_set_config(PIN_PB23, &config_port_pin);

// LED
struct port_config config_port_pin2;
port_get_config_defaults(&config_port_pin2);
config_port_pin2.direction = PORT_PIN_DIR_OUTPUT;
port_pin_set_config(PIN_PA23, &config_port_pin2);

/* Insert application code here, after the board has been initialized. */
while(1) {
	bool level = port_pin_get_input_level(PIN_PB23);
	if(level == false) {
		port_pin_set_output_level(PIN_PA23, false);
	}
	else {
		port_pin_set_output_level(PIN_PA23,  true);
	}
}
}
