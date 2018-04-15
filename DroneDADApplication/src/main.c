/*
Application code
*/

#include <asf.h>
#include <at25dfx_hal.h>
#include <crc32.h>

#include <errno.h>
#include "asf.h"
#include "main.h"
#include "stdio_serial.h"
#include "driver/include/m2m_wifi.h"
#include "socket/include/socket.h"
#include "iot/http/http_client.h"

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

// Flash Information
#define MAX_APPLICATION_COUNT 3
#define FLASH_HEADER_ADDR 0x0

struct application_metadata {
	uint32_t index;
	uint32_t crc;
	uint32_t start_addr;
	uint32_t data_len;
};

struct flash_header {
	uint32_t crc; // Maybe not necessary. Would be a CRC of address.
	uint32_t metadata_addr[MAX_APPLICATION_COUNT];
};

enum status_code dd_flash_read_data(uint32_t addr, uint8_t* buffer, uint32_t buffer_len) {
	enum status_code status;
	
	status = at25dfx_chip_wake(&at25dfx_chip);
	if(status != STATUS_OK) {
		return status;
	}
	
	status = at25dfx_chip_read_buffer(&at25dfx_chip, addr, buffer, buffer_len);
	if(status != STATUS_OK) {
		return status;
	}
	
	status = at25dfx_chip_sleep(&at25dfx_chip);
	if(status != STATUS_OK) {
		return status;
	}
	
	return status;
};

enum status_code dd_flash_write_data(uint32_t addr, uint8_t* buffer, uint32_t buffer_len) {
	enum status_code status;
	
	status = at25dfx_chip_wake(&at25dfx_chip);
	if(status != STATUS_OK) {
		return status;
	}
	
	status = at25dfx_chip_set_sector_protect(&at25dfx_chip, addr, false);
	if(status != STATUS_OK) {
		return status;
	}
	
	// Determine block size to erase.
	enum at25dfx_block_size block_size;
	if(buffer_len <= 4096) {
		block_size = AT25DFX_BLOCK_SIZE_4KB;
	}
	else if(buffer_len <= 32768) {
		block_size = AT25DFX_BLOCK_SIZE_32KB;
	}
	else {
		block_size = AT25DFX_BLOCK_SIZE_64KB;
	}
	
	status = at25dfx_chip_erase_block(&at25dfx_chip, addr, block_size);
	if(status != STATUS_OK) {
		return status;
	}
	
	status = at25dfx_chip_write_buffer(&at25dfx_chip, addr, buffer, buffer_len);
	if(status != STATUS_OK) {
		return status;
	}
	
	status = at25dfx_chip_set_global_sector_protect(&at25dfx_chip, true);
	if(status != STATUS_OK) {
		return status;
	}
	
	status = at25dfx_chip_sleep(&at25dfx_chip);
	if(status != STATUS_OK) {
		return status;
	}
	
	return status;
};

enum status_code get_application_metadata_addr(uint8_t index, uint32_t* addr) {
	enum status_code status;
	struct flash_header fh;
	
	status = dd_flash_read_data(FLASH_HEADER_ADDR, &fh, sizeof (struct flash_header));
	if(status != STATUS_OK) {
		return status;
	}
	
	*addr = fh.metadata_addr[index];
	
	return status;
}

enum status_code get_application_metadata(uint8_t index, struct application_metadata* am) {
	enum status_code status;
	uint32_t metadata_addr;
	
	status = get_application_metadata_addr(index, &metadata_addr);
	if(status != STATUS_OK) {
		return status;
	}
	
	status = dd_flash_read_data(metadata_addr, am, sizeof(struct application_metadata));
	if(status != STATUS_OK) {
		return status;
	}
	
	return status;
};

/**
 * \brief Configure UART console.
 */
#define EDBG_CDC_MODULE              SERCOM4
#define EDBG_CDC_SERCOM_MUX_SETTING  USART_RX_3_TX_2_XCK_3
#define EDBG_CDC_SERCOM_PINMUX_PAD0  PINMUX_UNUSED
#define EDBG_CDC_SERCOM_PINMUX_PAD1  PINMUX_UNUSED
#define EDBG_CDC_SERCOM_PINMUX_PAD2  PINMUX_PB10D_SERCOM4_PAD2
#define EDBG_CDC_SERCOM_PINMUX_PAD3  PINMUX_PB11D_SERCOM4_PAD3

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



#include "LEDDriver.h"
#include "GyroscopeDriver.h"

#define DATA_LENGTH 8
static uint8_t wr_buffer[DATA_LENGTH] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07
};
static struct i2c_master_packet wr_packet = {
	.address          = 0,
	.data_length      = DATA_LENGTH,
	.data             = wr_buffer,
	.ten_bit_address  = false,
	.high_speed       = false,
	.hs_master_code   = 0x00,
};

static void uartConsole_init(void)
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

void initDroneDAD() {
	// Basic System Components
	system_init();
	uartConsole_init();
	i2c_init();
	delay_init();
	
	// Storage
	at25dfx_init();
	nvm_init();
	dsu_crc32_init();
	
	// Sensor/Actuator
	lp3944_init();
	lsm6ds3_init();
	
	// Wifi
	wifiState_init();
	configure_timer();
	nm_bsp_init();
}

void setupWifiSubsystem() {
	
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

	printf("=== DroneDAD ===");
	
	/* Initialize the HTTP client service. */
	configure_http_client();
	
	/* Initialize Wi-Fi parameters structure. */
	memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));

	/* Initialize Wi-Fi driver with data and status callbacks. */
	param.pfAppWifiCb = wifi_cb;
	ret = m2m_wifi_init(&param);
	if (M2M_SUCCESS != ret) {
		printf("main: m2m_wifi_init call error! (res %d)\r\n", ret);
		while (1) {
		}
	}

	/* Initialize socket module. */
	socketInit();
	/* Register socket callback function. */
	registerSocketCallback(socket_cb, resolve_cb);

	/* Connect to router. */
	printf("main: connecting to WiFi AP %s...\r\n", (char *)MAIN_WLAN_SSID);
	m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID), MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);

	while (!(is_state_set(COMPLETED) || is_state_set(CANCELED))) {
		/* Handle pending events from network controller. */
		m2m_wifi_handle_events(NULL);
		/* Checks the timer timeout. */
		sw_timer_task(&swt_module_inst);
	}
	printf("main: please unplug the SD/MMC card.\r\n");
	printf("main: done.\r\n");

	while (1) {
	} /* Loop forever. */

	return 0;
}
