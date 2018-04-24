/*
 * LEDDriver.c
 *
 * Created: 4/9/2018 9:30:08 PM
 *  Author: John
 */ 

/*
Register Table

Register  Name         Function
0x0       Input 1      RO LED0-7 Input Register
0x1       Reg. 1       RO None
0x2       PSC0         RW Frequency Prescaler 1
0x3       PWM0         RW PWM Register 0 (Duty Cycle)
0x4       PSC1         RW Frequency Prescaler 2
0x5       PWM1         RW PWM Register 1 (Duty Cycle)
0x6       LS0          RW LED0-3 Selector
0x7       LS1          RW LED4-7 Selector
0x8       Reg. 8       RW None
0x9       Reg. 9       RW Nonw
*/

#ifndef LED_DRIVER_H
#define LED_DRIVER_H

#include <asf.h>

// Our I2C bus requires a delay after each write.
#define DELAY_VALUE_US 50

#define LED_OFF  0x0
#define LED_ON   0x1
#define LED_DIM0 0x2
#define LED_DIM1 0x3

#define ANGLE_OK LED_DIM1
#define ANGLE_WARN LED_DIM0
#define ANGLE_CRITICAL LED_ON

#define LED_BANK_1_REG 0x6
#define LED_BANK_2_REG 0x7

#define LED_0_OFFSET 0x00
#define LED_1_OFFSET 0x02
#define LED_2_OFFSET 0x04
#define LED_3_OFFSET 0x06
#define LED_4_OFFSET 0x00
#define LED_5_OFFSET 0x02
#define LED_6_OFFSET 0x04
#define LED_7_OFFSET 0x06

#define DIM0_REG  0x2
#define DUTY0_REG 0x3

#define DIM1_REG  0x4
#define DUTY1_REG 0x5

#define MAX_DIM_PERIOD 1600
#define MAX_PERCENTAGE 100

#define LP3944_SLAVE_ADDR 0x60
#define LP3944_WRITE_DATA_LENGTH 2
#define LP3944_READ_DATA_LENGTH 1
#define LP3944_REG_ADDR_FIELD 0
#define LP3944_DATA_FIELD 1

typedef struct {
	uint8_t led_bank_reg;
	uint8_t led_offset;
} lp3944_led_data;

static const lp3944_led_data LP3944_LED0 = {
	.led_bank_reg = LED_BANK_1_REG,
	.led_offset = LED_0_OFFSET
};

static const lp3944_led_data LP3944_LED1 = {
	.led_bank_reg = LED_BANK_1_REG,
	.led_offset = LED_1_OFFSET
};

static const lp3944_led_data LP3944_LED2 = {
	.led_bank_reg = LED_BANK_1_REG,
	.led_offset = LED_2_OFFSET
};

static const lp3944_led_data LP3944_LED3 = {
	.led_bank_reg = LED_BANK_1_REG,
	.led_offset = LED_3_OFFSET
};

static const lp3944_led_data LP3944_LED4 = {
	.led_bank_reg = LED_BANK_2_REG,
	.led_offset = LED_4_OFFSET
};

static const lp3944_led_data LP3944_LED5 = {
	.led_bank_reg = LED_BANK_2_REG,
	.led_offset = LED_5_OFFSET
};

static const lp3944_led_data LP3944_LED6 = {
	.led_bank_reg = LED_BANK_2_REG,
	.led_offset = LED_6_OFFSET
};

static const lp3944_led_data LP3944_LED7 = {
	.led_bank_reg = LED_BANK_2_REG,
	.led_offset = LED_7_OFFSET
};

struct i2c_master_module i2c_master_instance;
static void i2c_init(void)
{
	/* Initialize config structure and software module */
	struct i2c_master_config config_i2c_master;
	i2c_master_get_config_defaults(&config_i2c_master);
	/* Change buffer timeout to something longer */
	config_i2c_master.buffer_timeout    = 65535;
	config_i2c_master.pinmux_pad0       = PINMUX_PA08C_SERCOM0_PAD0;
	config_i2c_master.pinmux_pad1       = PINMUX_PA09C_SERCOM0_PAD1;
	config_i2c_master.generator_source  = GCLK_GENERATOR_0;
	/* Initialize and enable device with config */
	while(i2c_master_init(&i2c_master_instance, SERCOM0, &config_i2c_master) != STATUS_OK);
	i2c_master_enable(&i2c_master_instance);
}


static uint8_t lp3944_wr_buffer[LP3944_WRITE_DATA_LENGTH];
static struct i2c_master_packet lp3944_wr_packet = {
	.address          = LP3944_SLAVE_ADDR,
	.data_length      = LP3944_WRITE_DATA_LENGTH,
	.data             = lp3944_wr_buffer,
	.ten_bit_address  = false,
	.high_speed       = false,
	.hs_master_code   = 0x00,
};

static uint8_t lp3944_rd_buffer[1];
static struct i2c_master_packet lp3944_rd_packet = {
	.address          = LP3944_SLAVE_ADDR,
	.data_length      = 1,
	.data             = lp3944_rd_buffer,
	.ten_bit_address  = false,
	.high_speed       = false,
	.hs_master_code   = 0x00,
};

static void _lp3944_i2c_write(uint8_t addr, uint8_t value){
	enum status_code i2c_status;
	lp3944_wr_packet.address = LP3944_SLAVE_ADDR;
	lp3944_wr_packet.data_length = LP3944_WRITE_DATA_LENGTH;
	lp3944_wr_buffer[LP3944_REG_ADDR_FIELD] = addr;
	lp3944_wr_buffer[LP3944_DATA_FIELD] = value;
	lp3944_wr_packet.data = lp3944_wr_buffer;
	delay_us(DELAY_VALUE_US);
	i2c_status = i2c_master_write_packet_wait(&i2c_master_instance, &lp3944_wr_packet);
	if(i2c_status != STATUS_OK) {
		// Uhoh
		printf("Failure in %s when writing to i2c (%d)\r\n", __func__, i2c_status);
	}
}

static void _lp3944_i2c_read(uint8_t addr, uint8_t* value) {
	enum status_code i2c_status;
	lp3944_wr_packet.address = LP3944_SLAVE_ADDR;
	lp3944_wr_packet.data_length = LP3944_READ_DATA_LENGTH;
	lp3944_wr_buffer[LP3944_REG_ADDR_FIELD] = addr;
	lp3944_wr_packet.data = lp3944_wr_buffer;
	delay_us(DELAY_VALUE_US);
	i2c_status = i2c_master_write_packet_wait_no_stop(&i2c_master_instance, &lp3944_wr_packet);
	if(i2c_status != STATUS_OK) {
		//uhoh
		int i = 0;
	}
	else {
		lp3944_rd_packet.address = LP3944_SLAVE_ADDR;
		lp3944_rd_packet.data_length = LP3944_READ_DATA_LENGTH;
		delay_us(DELAY_VALUE_US);
		i2c_status = i2c_master_read_packet_wait_no_stop(&i2c_master_instance, &lp3944_rd_packet);
		if(i2c_status != STATUS_OK) {
			// Uhoh
			int i = 0;
		}
		int i = 0;
	}
	
	delay_us(DELAY_VALUE_US);
	i2c_master_send_stop(&i2c_master_instance);
	
	(*value) = lp3944_rd_packet.data[0];
}

static void lp3944_set_led(lp3944_led_data led, uint8_t value) {
	uint8_t new_value = 0;
	uint8_t previous_value = 0;
	
	// Get the previous configuration
	_lp3944_i2c_read(led.led_bank_reg, &previous_value);
	
	// Clear previous value for LED.
	previous_value &= ~(0x3 << led.led_offset);
	// Store new value for LED
	new_value = (value << led.led_offset) | previous_value;
	
	// Write the updated configuration.
	_lp3944_i2c_write(led.led_bank_reg, new_value);
}

static void lp3944_set_led_bank(uint8_t bank, uint8_t value) {
	uint8_t new_value = 0;

	// Store new value for LED
	new_value = (value << 0x0) | new_value;
	new_value = (value << 0x2) | new_value;
	new_value = (value << 0x4) | new_value;
	new_value = (value << 0x6) | new_value;
	
	// Write the updated configuration.
	_lp3944_i2c_write(bank, new_value);
}

static void _set_duty_cycle(uint8_t percentage, uint8_t pwm_reg) {
	if(percentage > MAX_PERCENTAGE) {
		// Not allowed.
		return;
	}
	
	uint8_t reg_value = ((percentage * 256) / 100); // See datasheet
	_lp3944_i2c_write(pwm_reg, reg_value);
}

static void lp3944_set_duty_cycle_0(uint8_t percentage) {
	_set_duty_cycle(percentage, DUTY0_REG);
}

static void lp3944_set_duty_cycle_1(uint8_t percentage) {
	_set_duty_cycle(percentage, DUTY1_REG);
}

static void _set_dim_rate(uint16_t dim_period_ms, uint8_t dim_reg) {
	if(dim_period_ms > MAX_DIM_PERIOD) {
		// Not allowed
		return;
	}
	
	// We cast down to a uin8_t from uin16_t.
	// This is safe because our max input is 1600, which equates to a reg_value of 255
	uint8_t reg_value = (((dim_period_ms * 160) / 1000) - 1); // See datasheet
	_lp3944_i2c_write(dim_reg, reg_value);
}

static void lp3944_set_dim_period_0(uint16_t dim_period_ms) {
	_set_dim_rate(dim_period_ms, DIM0_REG);
}

static void lp3944_set_dim_period_1(uint16_t dim_period_ms) {
	_set_dim_rate(dim_period_ms, DIM1_REG);
}

static void lp3944_reset() {
	port_pin_set_output_level(PIN_PB02, false);
	port_pin_set_output_level(PIN_PB02, true);
}

static void set_all_leds(uint8_t value) {
	lp3944_set_led_bank(LED_BANK_1_REG, value);
	lp3944_set_led_bank(LED_BANK_2_REG, value);
}

static void lp3944_init() {
	// Set Reset pin High
	struct port_config config_port_pin;
	port_get_config_defaults(&config_port_pin);
	config_port_pin.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(PIN_PB02, &config_port_pin);
	port_pin_set_output_level(PIN_PB02, true);
	
	// Set DIM0 (Warning) Values
	lp3944_set_dim_period_0(100);
	lp3944_set_duty_cycle_0(50);
	
	// Set DIM1 (Normal Operation) Values
	lp3944_set_dim_period_1(1000);
	lp3944_set_duty_cycle_1(10);
	
	// Set all LEDs off.
	lp3944_set_led(LP3944_LED0, LED_OFF);
	lp3944_set_led(LP3944_LED1, LED_OFF);
	lp3944_set_led(LP3944_LED2, LED_OFF);
	lp3944_set_led(LP3944_LED3, LED_OFF);
	lp3944_set_led(LP3944_LED4, LED_OFF);
	lp3944_set_led(LP3944_LED5, LED_OFF);
	lp3944_set_led(LP3944_LED6, LED_OFF);
	lp3944_set_led(LP3944_LED7, LED_OFF);
}

#endif