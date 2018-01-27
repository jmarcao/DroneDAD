/*
 * CLIHandler.h
 *
 * Created: 1/24/2018 7:31:49 PM
 *  Author: John
 */ 

#ifndef CLIHANDLER_H_
#define CLIHANDLER_H_

#define OPTION_COUNT 13

struct option {
	const char* command;
	const char* params;
	const char* description;
};

struct option_list {
	struct option options[OPTION_COUNT];
} cmd_list;

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
void handle_read_accel(int num_readings, int interval_ms);
void handle_adc_get(char port, int pin_num);
void handle_mcu_temp();
void handle_i2c_scan();

#endif /* CLIHANDLER_H_ */