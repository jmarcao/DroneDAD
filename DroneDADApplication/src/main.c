/*
Application code
*/

#include <asf.h>
#include <at25dfx_hal.h>
#include <crc32.h>

// Serial Setup
#define AT25DFX_BUFFER_SIZE  (10)
static uint8_t read_buffer[AT25DFX_BUFFER_SIZE] = { 0 };
static uint8_t write_buffer[AT25DFX_BUFFER_SIZE] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
struct spi_module at25dfx_spi;
struct at25dfx_chip_module at25dfx_chip;

static void at25dfx_init(void)
{
	enum status_code status;
	struct at25dfx_chip_config at25dfx_chip_config;
	struct spi_config at25dfx_spi_config;
	spi_get_config_defaults(&at25dfx_spi_config);
	at25dfx_spi_config.mode_specific.master.baudrate = 120000; // 120kHz - AT25DFX_CLOCK_SPEED;
	at25dfx_spi_config.mux_setting = SPI_SIGNAL_MUX_SETTING_E; // AT25DFX_SPI_PINMUX_SETTING;
	at25dfx_spi_config.pinmux_pad0 = PINMUX_PA16C_SERCOM1_PAD0; // MISO - AT25DFX_SPI_PINMUX_PAD0;
	at25dfx_spi_config.pinmux_pad1 = PINMUX_UNUSED; // CS - AT25DFX_SPI_PINMUX_PAD1;
	at25dfx_spi_config.pinmux_pad2 = PINMUX_PA18C_SERCOM1_PAD2; // MOSI - AT25DFX_SPI_PINMUX_PAD2;
	at25dfx_spi_config.pinmux_pad3 = PINMUX_PA19C_SERCOM1_PAD3; // SCK - AT25DFX_SPI_PINMUX_PAD3;
	status = spi_init(&at25dfx_spi, SERCOM1 /*AT25DFX_SPI*/, &at25dfx_spi_config);
	spi_enable(&at25dfx_spi);
	
	at25dfx_chip_config.type = AT25DFX_081A; // AT25DFX_MEM_TYPE;
	at25dfx_chip_config.cs_pin = PIN_PA07; // AT25DFX_CS;
	status = at25dfx_chip_init(&at25dfx_chip, &at25dfx_spi, &at25dfx_chip_config);
}

int main (void)
{
	system_init();
	at25dfx_init();

	/* Insert application code here, after the board has been initialized. */
	
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

	/* Insert application code here, after the board has been initialized. */
	//while(1) {
		at25dfx_chip_wake(&at25dfx_chip);
		    
		if (at25dfx_chip_check_presence(&at25dfx_chip) != STATUS_OK) {
			// Handle missing or non-responsive device
			while(1) { }
		}
		at25dfx_chip_read_buffer(&at25dfx_chip, 0x0000, read_buffer, AT25DFX_BUFFER_SIZE);
		at25dfx_chip_set_sector_protect(&at25dfx_chip, 0x10000, false);
		at25dfx_chip_erase_block(&at25dfx_chip, 0x10000, AT25DFX_BLOCK_SIZE_4KB);
		at25dfx_chip_write_buffer(&at25dfx_chip, 0x10000, write_buffer, AT25DFX_BUFFER_SIZE);
		at25dfx_chip_set_global_sector_protect(&at25dfx_chip, true);
		at25dfx_chip_read_buffer(&at25dfx_chip, 0x10000, read_buffer, AT25DFX_BUFFER_SIZE);
		at25dfx_chip_sleep(&at25dfx_chip);
			
		// LED light up with Button Press.
		bool level = port_pin_get_input_level(PIN_PB23);
		if(level == false) {
			port_pin_set_output_level(PIN_PA23, false);
		}
		else {
			port_pin_set_output_level(PIN_PA23,  true);
		}
	//}
}
