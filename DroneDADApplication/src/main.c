/*
Application code
*/

#include <asf.h>
#include <at25dfx_hal.h>
#include <crc32.h>

/** UART module for debug. */
static struct usart_module cdc_uart_module;

// Serial Setup
#define AT25DFX_BUFFER_SIZE  (512)
static uint8_t read_buffer[AT25DFX_BUFFER_SIZE] = { 0 };
static uint8_t write_buffer[AT25DFX_BUFFER_SIZE] = { 0 };
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

void nvm_init(void)
{
	struct nvm_config config_nvm;
	nvm_get_config_defaults(&config_nvm);
	config_nvm.manual_page_write = false;
	nvm_set_config(&config_nvm);
}

/**
 * \brief Configure UART console.
 */
#define EDBG_CDC_MODULE              SERCOM4
#define EDBG_CDC_SERCOM_MUX_SETTING  USART_RX_3_TX_2_XCK_3
#define EDBG_CDC_SERCOM_PINMUX_PAD0  PINMUX_UNUSED
#define EDBG_CDC_SERCOM_PINMUX_PAD1  PINMUX_UNUSED
#define EDBG_CDC_SERCOM_PINMUX_PAD2  PINMUX_PB10D_SERCOM4_PAD2
#define EDBG_CDC_SERCOM_PINMUX_PAD3  PINMUX_PB11D_SERCOM4_PAD3
static void serial_init(void)
{
	struct usart_config usart_conf;

	usart_get_config_defaults(&usart_conf);
	usart_conf.mux_setting = EDBG_CDC_SERCOM_MUX_SETTING;
	usart_conf.pinmux_pad0 = EDBG_CDC_SERCOM_PINMUX_PAD0;
	usart_conf.pinmux_pad1 = EDBG_CDC_SERCOM_PINMUX_PAD1;
	usart_conf.pinmux_pad2 = EDBG_CDC_SERCOM_PINMUX_PAD2;
	usart_conf.pinmux_pad3 = EDBG_CDC_SERCOM_PINMUX_PAD3;
	usart_conf.baudrate    = 115200;

	stdio_serial_init(&cdc_uart_module, EDBG_CDC_MODULE, &usart_conf);
	usart_enable(&cdc_uart_module);
}

int main (void)
{
	
	system_init();
	at25dfx_init();
	dsu_crc32_init();
	nvm_init();
	serial_init();
	
	printf("In Application!\n\r");
	
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
	crcstat = dsu_crc32_cal(&read_buffer[0], AT25DFX_BUFFER_SIZE, &read_crc_res);
	crcstat = dsu_crc32_cal(&write_buffer[0], AT25DFX_BUFFER_SIZE, &write_crc_res);
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
	crcstat = dsu_crc32_cal(&read_page_buffer[0], NVMCTRL_PAGE_SIZE, &read_crc_res);
	crcstat = dsu_crc32_cal(&write_page_buffer[0], NVMCTRL_PAGE_SIZE, &write_crc_res);
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
