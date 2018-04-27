/*
 * Bootloader
 */

#include <asf.h>
#include <at25dfx_hal.h>
#include <crc32.h>
#include <stdio.h>
#include <delay.h>

// NVM Information
#define NVM_NUMBER_OF_PAGES 0x1000
#define NVM_NUMBER_OF_ROWS (NVM_NUMBER_OF_PAGES / 0x4) // 1024 rows

// Flash Information
#define MAX_APPLICATION_COUNT 3
#define FLASH_HEADER_ADDR 0x0

// NVM Locations
#define APPLICATION_ROW 64
#define APP_START_ADDR (APPLICATION_ROW * NVMCTRL_ROW_PAGES * NVMCTRL_PAGE_SIZE) // 0x=6400 for Row 64
#define BOOT_STATUS_ROW (NVM_NUMBER_OF_ROWS - 1) // Store boot status in second to last row. (1023)
#define APPLICATION_METADATA_ROW (NVM_NUMBER_OF_ROWS - 2) // Store metadata inn third to last row. (1022)

// Install Flags
#define INSTALL_FLAG_TRUE 0xFF
#define INSTALL_FLAG_FALSE 0x00

// Signature and Versioning.
#define DRONEDAD_BOOT_SIGNATURE_LENGTH 8
#define DRONEDAD_MAJOR_VERSION 0x00 // Just to make us feel more like a legit product
#define DRONEDAD_MINOR_VERSION 0x00
#define DRONEDAD_REVISION 0x01 // Revision can be used just to debug, forcing a bad signature.
const uint8_t DRONEDAD_BOOT_SIGNATURE[] = {0xDD, 0xAD, 0x20, 0x18, \
	0x00, DRONEDAD_MAJOR_VERSION, DRONEDAD_MINOR_VERSION, DRONEDAD_REVISION}; // DD AD 20 18 0x00 <MajorVersion> <MinorVersion> <Revesion>
	
struct boot_status {
	uint8_t signature[DRONEDAD_BOOT_SIGNATURE_LENGTH];
	uint8_t install_flag; // 
	uint8_t install_idx;
	uint32_t application_star_addr; // Either set to APP_START_ADDR or leave unused. We'll see the usefulness.
};

struct application_metadata {
	uint32_t index;
	uint32_t crc;
	uint32_t start_addr;
	uint32_t data_len;
};

struct flash_header {
	uint32_t metadata_addr[MAX_APPLICATION_COUNT];
};

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
	if(STATUS_OK != status) {
		printf("Failed to intialize Flash memory chip.\r\n");
	}
	spi_enable(&at25dfx_spi);
	
	at25dfx_chip_config.type = AT25DFX_081A; // AT25DFX_MEM_TYPE;
	at25dfx_chip_config.cs_pin = PIN_PA07; // AT25DFX_CS;
	status = at25dfx_chip_init(&at25dfx_chip, &at25dfx_spi, &at25dfx_chip_config);
	if(STATUS_OK != status) {
		printf("Failed to intialize Flash memory chip.\r\n");
	}
	
	at25dfx_chip_wake(&at25dfx_chip);
	at25dfx_chip_sleep(&at25dfx_chip);
}

void watchdog_early_warning_callback(void)
{
    port_pin_set_output_level(PIN_PA23, false);
}
void configure_wdt(void)
{
    /* Create a new configuration structure for the Watchdog settings and fill
     * with the default module settings. */
    struct wdt_conf config_wdt;
    wdt_get_config_defaults(&config_wdt);
	
    /* Set the Watchdog configuration settings */
    config_wdt.always_on            = false;
    config_wdt.clock_source         = GCLK_GENERATOR_4;
    config_wdt.timeout_period       = WDT_PERIOD_4096CLK;
    config_wdt.early_warning_period = WDT_PERIOD_2048CLK;
	
    /* Initialize and enable the Watchdog with the user settings */
    wdt_set_config(&config_wdt);
}
void configure_wdt_callbacks(void)
{
    wdt_register_callback(watchdog_early_warning_callback,
        WDT_CALLBACK_EARLY_WARNING);
    wdt_enable_callback(WDT_CALLBACK_EARLY_WARNING);
}

void nvm_init(void)
{
	struct nvm_config config_nvm;
	nvm_get_config_defaults(&config_nvm);
	config_nvm.manual_page_write = false;
	nvm_set_config(&config_nvm);
}

void application_jump(void) {
	// Declare the application entry function pointer
	void (*application_code_entry)(void);

	// Set Main Stack Pointer to the proper address (set in Linker for Application)
	__set_MSP(*(uint32_t*)APP_START_ADDR);

	// Update Vector Table Offset Register
	SCB->VTOR = ((uint32_t)APP_START_ADDR & SCB_VTOR_TBLOFF_Msk);

	// Set the function pointer.
	application_code_entry = (void (*)(void))(unsigned*)(*(unsigned*)(APP_START_ADDR + 4));

	// Last thing we do before jumping is disable the USART. Otherwise the App cannot regain control of it.
	usart_disable(&cdc_uart_module);
	spi_disable(&at25dfx_spi);

	application_code_entry();
}

enum status_code dd_nvm_row_read(int row, uint8_t* data, int data_len) {
	enum status_code status;
	
	int row_addr = row * NVMCTRL_ROW_PAGES * NVMCTRL_PAGE_SIZE;
	do {
		status = nvm_read_buffer(row_addr, data, data_len);
	} while (status == STATUS_BUSY);
	
	return status;
};

enum status_code dd_nvm_page_read(int page, uint8_t* data, int data_len) {
	enum status_code status;
	
	//printf("Reading NVM Page %d\r\n", page);
	
	int page_addr = page * NVMCTRL_PAGE_SIZE;
	do {
		status = nvm_read_buffer(page_addr, data, data_len);
	} while (status == STATUS_BUSY);
	
	return status;
};

enum status_code dd_nvm_page_write(int page, uint8_t* data, int data_len) {
	enum status_code status;
	int page_addr = page * NVMCTRL_PAGE_SIZE;
	
	//printf("Writing to NVM Page %d\r\n", page);
	
	do {
		status = nvm_write_buffer(page_addr, data, data_len);
	} while(status == STATUS_BUSY);
	
	return status;
};

enum status_code dd_nvm_row_write(int row, uint8_t* data, int data_len) {
	enum status_code status;
	int row_addr = row * NVMCTRL_ROW_PAGES * NVMCTRL_PAGE_SIZE;
	
	do {
		status = nvm_erase_row(row_addr);	
	} while(status == STATUS_BUSY);
	if(status != STATUS_OK) {
		return status;
	}
	
	int i = 0;
	uint32_t page = row * NVMCTRL_ROW_PAGES;
	uint32_t dataRemaining = data_len;
	while(dataRemaining != 0) {
		if(dataRemaining > NVMCTRL_PAGE_SIZE) {
			status = dd_nvm_page_write(page + i, data + (i*NVMCTRL_PAGE_SIZE), NVMCTRL_PAGE_SIZE);
			if(status != STATUS_OK) {
				return status;
			}
			i++;
			dataRemaining -= NVMCTRL_PAGE_SIZE;
		}
		else {
			status = dd_nvm_page_write(page + i, data + (i*NVMCTRL_PAGE_SIZE), dataRemaining);
			if(status != STATUS_OK) {
				return status;
			}
			dataRemaining = 0;
		}
	}
	return status;
}

enum status_code get_boot_status(struct boot_status* bs) {
	enum status_code status;
	status = dd_nvm_row_read(BOOT_STATUS_ROW, (uint8_t*)bs, sizeof (struct boot_status));
	return status;
}

enum status_code set_boot_status(struct boot_status* bs) {
	enum status_code status;
	status = dd_nvm_row_write(BOOT_STATUS_ROW, (uint8_t*)bs, sizeof (struct boot_status));
	return status;	
}

void get_boot_status_default(struct boot_status* bs) {
	for(int i = 0; i < DRONEDAD_BOOT_SIGNATURE_LENGTH; i++) {
		bs->signature[i] = DRONEDAD_BOOT_SIGNATURE[i];
	}
	bs->install_flag = INSTALL_FLAG_FALSE;
	bs->install_idx = 0;
	bs->application_star_addr = APP_START_ADDR;
}


/*
 * An error has occurred. This loop will blink the LED in a distinct way to let the user know
 * a Boot loader error has occurred.
 */
void error_loop() {
	while(1) {}
}

bool check_signature(uint8_t* signature) {
	bool valid = true;
	for(int i = 0; valid && (i < DRONEDAD_BOOT_SIGNATURE_LENGTH); i++) {
		valid &= (signature[i] == DRONEDAD_BOOT_SIGNATURE[i]);
	}
	return valid;
}

enum status_code load_golden_image() {
	return STATUS_OK; // TODO
}

void restart() {
	printf("Restarting...");
	while(1) {}; // TODO: Implement watchdog functionality to reboot.
}

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

bool check_flash_application_crc(uint32_t crc_to_check, uint32_t flash_addr, uint32_t data_len) {
	enum status_code status;
	const uint32_t CHUNK_LEN = 4096; // We'll read 4kb chunks.
	uint8_t data[CHUNK_LEN];
	uint32_t calculated_crc = 0;
	
	// Loop through the application in 4KB chunks to calculate the CRC.
	int i = 0;
	bool dataReady = true;
	while(dataReady) {
		status = dd_flash_read_data(flash_addr + i, &data[0], CHUNK_LEN);
		if(STATUS_OK != status) {
			return false;
		}

		// Compute the CRC on the chunk if we have a full chunk in our buffer.
		if(i + CHUNK_LEN < data_len) { // If we have more than a chunk left to read, crc a full chunk
			crc32_recalculate(&data[0], CHUNK_LEN, &calculated_crc);
		}
		else { // If we have less than a full chunk to read, do not! We will do our last crc after this loop.
			crc32_recalculate(&data[0], data_len - i, &calculated_crc);
			dataReady = false;
		}

		// Keep Going.
		i += CHUNK_LEN;
		printf("Checked %d of %d bytes\r", i, data_len);
	}
	printf("\r\n");
	
	return (calculated_crc == crc_to_check);
}

bool check_nvm_application_crc(uint32_t crc_to_check, uint32_t nvm_row, uint32_t data_len) {
	enum status_code status;
	const uint32_t CHUNK_LEN = NVMCTRL_PAGE_SIZE; // We'll read in pages
	uint8_t data[CHUNK_LEN];
	uint32_t calculated_crc;
	
	uint32_t nvm_page = nvm_row * NVMCTRL_ROW_PAGES;
		
	// Loop through the application in 256B chunks to calculate the CRC.
	int i = 0;
	uint32_t dataRemaining = data_len;
	while(dataRemaining > 0) {
		if((dataRemaining / CHUNK_LEN) > 0) {
			// Read a chunk of data
			status = dd_nvm_page_read(nvm_page + i, &data[0], CHUNK_LEN);
			if(STATUS_OK != status) {
				printf("NVM Row Read failed! (%d)\r\n", status);
				return false;	
			}
			crc32_recalculate(&data[0], CHUNK_LEN, &calculated_crc);
			dataRemaining -= CHUNK_LEN;
		}
		else {
			// Read the last bit
			status = dd_nvm_page_read(nvm_page + i, &data[0], dataRemaining);
			if(STATUS_OK != status) {
				printf("NVM Row Read failed! (%d)\r\n", status);
				return false;
			}
			crc32_recalculate(&data[0], dataRemaining, &calculated_crc);
			dataRemaining = 0;
		}
				
		// Keep Going.
		i++;
		printf("Checked %d of %d bytes\r", (data_len - dataRemaining), data_len);
	}
	
	printf("%s: Expected %x\r\n", __func__, crc_to_check);
	printf("%s: Actual %x\r\n", __func__, calculated_crc);
	return (calculated_crc == crc_to_check);
}

enum status_code write_application_to_nvm(uint32_t flash_addr, uint32_t data_len) {
	enum status_code status = STATUS_OK;
	const int CHUNK_LEN = NVMCTRL_ROW_SIZE;
	uint8_t data[CHUNK_LEN];
	uint32_t calculated_crc = 0;
	
	int i = 0;
	uint32_t dataRemaining = data_len;
	while(dataRemaining > 0) {
		if(dataRemaining > CHUNK_LEN) {
			status = dd_flash_read_data(flash_addr + i, &data[0], CHUNK_LEN);
			if(status != STATUS_OK) {
				printf("Failed Flash Read (%d)\r\n", status);
				break;
			}
			
			crc32_recalculate(&data[0], CHUNK_LEN, &calculated_crc);
			
			status = dd_nvm_row_write(APPLICATION_ROW + (i / CHUNK_LEN), &data[0], CHUNK_LEN);
			if(status != STATUS_OK) {
				printf("Failed NVM Write (%d)\r\n", status);
				break;
			}
			
			dataRemaining -= CHUNK_LEN;
		}
		else {
			status = dd_flash_read_data(flash_addr + i, &data[0], dataRemaining);
			if(status != STATUS_OK) {
				printf("Failed Flash Read (%d)\r\n", status);
				break;
			}
			
			crc32_recalculate(&data[0], dataRemaining, &calculated_crc);
			
			status = dd_nvm_row_write(APPLICATION_ROW + (i / CHUNK_LEN), &data[0], dataRemaining);
			if(status != STATUS_OK) {
				printf("Failed NVM Write (%d)\r\n", status);
				break;
			}
			
			dataRemaining = 0;
		}
		printf("Wrote %d of %d bytes.\r", (data_len - dataRemaining), data_len);
		i += CHUNK_LEN;
	}
	/*
	for(unsigned int i = 0; i < data_len; i += CHUNK_LEN) {
		status = dd_flash_read_data(flash_addr + i, &data[0], CHUNK_LEN);
		if(status != STATUS_OK) {
			printf("Failed Flash Read (%d)\r\n", status);
			break;
		}
		crc32_recalculate(&data[0], CHUNK_LEN, &calculated_crc);
		status = dd_nvm_row_write(APPLICATION_ROW + (i / CHUNK_LEN), &data[0], CHUNK_LEN);
		if(status != STATUS_OK) {
			printf("Failed NVM Write (%d)\r\n", status);
			break;
		}
		printf("Wrote %d of %d bytes.\r", i, data_len);
	}
	*/
	printf("\r\n");
	
	printf("%s: Flash Actual %x\r\n", __func__, calculated_crc);
	return status;
}

enum status_code do_fw_update(struct boot_status* bs) {
	enum status_code status;
	
	// Get the metadata from the Flash.
	struct application_metadata am;
	status = get_application_metadata(bs->install_idx, &am);
	if(STATUS_OK != status) {
		return status;
	}
	printf("New FW Located at Index %d (0x%x)\r\n", am.index, am.start_addr);
	printf("New FW CRC: 0x%x\r\n", am.crc);
			
	// Check CRC on flash.
	bool flash_crc_valid = check_flash_application_crc(am.crc, am.start_addr, am.data_len);
	if(flash_crc_valid == false) {
		printf("CRC of data on flash failed!\r\n");
		return STATUS_BUSY;
	}
			
	// Update.
	status = write_application_to_nvm(am.start_addr, am.data_len);
	if(STATUS_OK != status) {
		printf("Failed writing application to NVM!\r\n");
		return status;
	}
			
	// Check CRC on nvm.
	bool nvm_crc_valid = check_nvm_application_crc(am.crc, APPLICATION_ROW, am.data_len);
	if(nvm_crc_valid == false) {
		printf("Failed NVM CRC Check!\r\n");
		return STATUS_BUSY;
	}
	
	// Write the metadata to NVM
	status = dd_nvm_row_write(APPLICATION_METADATA_ROW, &am, sizeof(struct application_metadata));
	if(STATUS_OK != status) {
		return status;
	}
			
	// Update the status flag.
	bs->install_flag = INSTALL_FLAG_FALSE;
	status = set_boot_status(bs);
	if(STATUS_OK != status) {
		return status;
	}
	
	return status;
};

bool loaded_fw_valid() {
	enum status_code status;
	bool ret = false;
	struct application_metadata am;
	status = dd_nvm_row_read(APPLICATION_METADATA_ROW, &am, sizeof(struct application_metadata));
	if(STATUS_OK != status) {
		return status;
	}
	
	ret = check_nvm_application_crc(am.crc, APPLICATION_ROW, am.data_len);
	
	return ret;
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

void nvm_test() {
	 uint8_t page_buffer[NVMCTRL_PAGE_SIZE];
	 uint8_t read_buffer[NVMCTRL_PAGE_SIZE] = { '\0' };
	 for (uint32_t i = 0; i < NVMCTRL_PAGE_SIZE; i++) {
		 page_buffer[i] = i;
	 }
	 enum status_code error_code;
	 do
	 {
		 error_code = nvm_erase_row(
		 100 * NVMCTRL_ROW_PAGES * NVMCTRL_PAGE_SIZE);
	 } while (error_code == STATUS_BUSY);
	 do
	 {
		 error_code = nvm_write_buffer(
		 100 * NVMCTRL_ROW_PAGES * NVMCTRL_PAGE_SIZE,
		 page_buffer, NVMCTRL_PAGE_SIZE);
	 } while (error_code == STATUS_BUSY);
	 do
	 {
		 error_code = nvm_read_buffer(
		 100 * NVMCTRL_ROW_PAGES * NVMCTRL_PAGE_SIZE,
		 read_buffer, NVMCTRL_PAGE_SIZE);
	 } while (error_code == STATUS_BUSY);
}

int main (void) {	
	enum status_code status;
	
	// Initialize
	system_init();
	nvm_init();
	at25dfx_init();
	delay_init();
	// Enable watchdogs, kick by calling wdt_reset_count() -- Only when ready!
	//configure_wdt();
	//configure_wdt_callbacks();
	//system_interrupt_enable_global();
	
	// Get our boot status
	struct boot_status bs;
	get_boot_status(&bs);
	uartConsole_init();
	
	printf("\r\n\r\n====DRONEDAD Bootloader====\r\n");
	printf("Compiled %s %s\r\n", __TIME__, __DATE__);

	printf("Checking Boot Status...\r\n");
	if(check_signature(bs.signature) == false) {
		printf("Invalid boot signature. Boot Status cannot be trusted.\r\n");
		printf("Attempting CRC check on FW.\r\n");
		if(loaded_fw_valid() == true) {
			// Our FW  is valid, we have corrupted the boot flag. Reset the flag and reboot.
			printf("FW valid. Reseting Boot signature and restarting...\r\n");
			get_boot_status_default(&bs);
			set_boot_status(&bs);
			restart();
		}
		else {
			// Our boot status was invalid and the CRC did not match.
			// We should load the golden image from flash to NVM.
			printf("FW invalid. Attempting to load Golden Image\r\n");
			load_golden_image();
			get_boot_status_default(&bs);
			set_boot_status(&bs);
			restart();
		}
	}
	else {
		printf("Boot Signature Valid.\r\n");
	}
	
	if(bs.install_flag == INSTALL_FLAG_TRUE) {
		printf("Update Scheduled. Now Updating...\r\n");
		
		status = do_fw_update(&bs);
		if(STATUS_OK != status) {
			printf("Failed FW update!");
			error_loop();
		}
		else {
			printf("Firmware Update Successful.\r\n");
		}
		
		// All good, reboot.
		restart();
	}
	
	// Application time!
	printf("Jumping to application\r\n");
	application_jump();
	
	// Should not get here.
	error_loop();
}	
