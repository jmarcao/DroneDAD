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

#define LED_OFF  0x0
#define LED_ON   0x1
#define LED_DIM0 0x2
#define LED_DIM1 0x3

#define LED_BANK_MASK 0x8
#define LED_BANK_1_REG 0x6
#define LED_0 0x00
#define LED_1 0x02
#define LED_2 0x04
#define LED_3 0x06

#define LED_BANK_2_REG 0x7
#define LED_4 0x8
#define LED_5 0x9
#define LED_6 0xA
#define LED_7 0xB

#define DIM0_REG  0x2
#define DUTY0_REG 0x3

#define DIM1_REG  0x4
#define DUTY1_REG 0x5

#define MAX_DIM_PERIOD 1600
#define MAX_PERCENTAGE 100

#define LP3944_SLAVE_ADDR 0x60
#define LP3944_DATA_LENGTH 0x1
static uint8_t lp3944_wr_buffer[LP3944_DATA_LENGTH];
static struct i2c_master_packet lp3944_wr_packet = {
	.address          = LP3944_SLAVE_ADDR,
	.data_length      = LP3944_DATA_LENGTH,
	.data             = lp3944_wr_buffer,
	.ten_bit_address  = false,
	.high_speed       = false,
	.hs_master_code   = 0x00,
};

void _lp3944_i2c_write(uint8_t addr, uint8_t value){
	enum status_code i2c_status;
	lp3944_wr_buffer[0] = value;
	i2c_status = i2c_master_write_packet_wait(&i2c_master_instance, &lp3944_wr_packet);
	if(i2c_status != STATUS_OK) {
		// Uhoh
		printf("Failure in %s when writing to i2c (%d)", __func__, i2c_status);
	}
}

void set_led(uint8_t led, uint8_t value) {
	uint8_t reg = 0x00;
	if((led & LED_BANK_MASK) == 0x1) {
		// LED 4 through 7
		reg = LED_BANK_2_REG;
	}
	else {
		// LED 0 through 3
		reg = LED_BANK_1_REG;
	}
	
	_lp3944_i2c_write(reg, (value & 0x7));
}

void _set_duty_cycle(uint8_t percentage, uint8_t pwm_reg) {
	if(percentage > MAX_PERCENTAGE) {
		// Not allowed.
		return;
	}
	
	uint8_t reg_value = ((percentage * 256) / 100); // See datasheet
	_lp3944_i2c_write(pwm_reg, reg_value);
}

void set_duty_cycle_0(uint8_t percentage) {
	_set_duty_cycle(percentage, DUTY0_REG);
}

void set_duty_cycle_1(uint8_t percentage) {
	_set_duty_cycle(percentage, DUTY1_REG);
}

void _set_dim_rate(uint16_t dim_period_ms, uint8_t dim_reg) {
	if(dim_period_ms > MAX_DIM_PERIOD) {
		// Not allowed
		return;
	}
	
	// We cast down to a uin8_t from uin16_t.
	// This is safe because our max input is 1600, which equates to a reg_value of 255
	uint8_t reg_value = (((dim_period_ms * 160) / 1000) - 1); // See datasheet
	_lp3944_i2c_write(dim_reg, reg_value);
}

void set_dim_period_0(uint16_t dim_period_ms) {
	_set_dim_rate(dim_period_ms, DIM0_REG);
}

void set_dim_period_1(uint16_t dim_period_ms) {
	_set_dim_rate(dim_period_ms, DIM1_REG);
}