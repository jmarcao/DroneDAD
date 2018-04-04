#include <asf.h>
#include <at25dfx_hal.h>
#include <crc32.h>
#include <stdio.h>

// NVM Information
#define NVM_NUMBER_OF_PAGES 0x1000
#define NVM_NUMBER_OF_ROWS (NVM_NUMBER_OF_PAGES / 0x4) // 1024 rows

// Flash Information
#define MAX_APPLICATION_COUNT 3
#define FLASH_HEADER_ADDR 0x0

// NVM Locations
#define APPLICATION_ROW 40 
#define APP_START_ADDR (APPLICATION_ROW * NVMCTRL_ROW_PAGES * NVMCTRL_PAGE_SIZE) // 0x2800 for Row 40
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
	uint32_t crc; // Maybe not necessary. Would be a CRC of address.
	uint32_t metadata_addr[MAX_APPLICATION_COUNT];
};

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

enum status_code dd_nvm_row_write(int row, uint8_t* data, int data_len) {
	enum status_code status;
	int row_addr = row * NVMCTRL_ROW_PAGES * NVMCTRL_PAGE_SIZE;
	
	do {
		status = nvm_erase_row(row_addr);	
	} while(status == STATUS_BUSY);
	
	if(status != STATUS_OK) {
		return status;
	}
	
	do {
		nvm_write_buffer(row_addr, data, data_len);
	} while (status == STATUS_BUSY);
	
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
	uint32_t calculated_crc;
	
	// Loop through the application in 4KB chunks to calculate the CRC.
	for(int i = 0; i < data_len; i += CHUNK_LEN) {
		status = dd_flash_read_data(flash_addr + i, &data[0], CHUNK_LEN);
		if(STATUS_OK != status) {
			return false;
		}
		
		status = dsu_crc32_cal(&data[0], CHUNK_LEN, &calculated_crc);
		if(STATUS_OK != status) {
			return false;
		}
	}
	
	return (calculated_crc == crc_to_check);
}

bool check_nvm_application_crc(uint32_t crc_to_check, uint32_t nvm_row, uint32_t data_len) {
	enum status_code status;
	const uint32_t CHUNK_LEN = NVMCTRL_ROW_SIZE; // We'll read in rows.
	uint8_t data[CHUNK_LEN];
	uint32_t calculated_crc;
		
	// Loop through the application in 4KB chunks to calculate the CRC.
	for(int i = 0; i < data_len; i += CHUNK_LEN) {
		status = dd_nvm_row_read(nvm_row + i, &data[0], CHUNK_LEN);
		if(STATUS_OK != status) {
			return false;
		}
				
		status = dsu_crc32_cal(&data[0], CHUNK_LEN, &calculated_crc);
		if(STATUS_OK != status) {
			return false;
		}
	}
		
	return (calculated_crc == crc_to_check);
}

enum status_code write_application_to_nvm(uint32_t flash_addr, uint32_t data_len) {
	enum status_code status;
	const int CHUNK_LEN = NVMCTRL_ROW_SIZE;
	uint8_t data[CHUNK_LEN];
	
	for(unsigned int i = 0; i < data_len; i += CHUNK_LEN) {
		status = dd_flash_read_data(flash_addr + i, &data[0], CHUNK_LEN);
		status = dd_nvm_row_write(APPLICATION_ROW + i, &data[0], CHUNK_LEN);
	}
	
}

enum status_code do_fw_update(struct boot_status* bs) {
	enum status_code status;
	
	// Get the metadata from the Flash.
	struct application_metadata am;
	status = get_application_metadata(bs->install_idx, &am);
	if(STATUS_OK != status) {
		return status;
	}
			
	// Check CRC on flash.
	bool flash_crc_valid = check_flash_application_crc(am.crc, am.start_addr, am.data_len);
	if(flash_crc_valid == false) {
		error_loop();
	}
			
	// Update.
	status = write_application_to_nvm(am.start_addr, am.data_len);
	if(STATUS_OK != status) {
		return status;
	}
			
	// Check CRC on nvm.
	bool nvm_crc_valid = check_nvm_application_crc(am.crc, APPLICATION_ROW, am.data_len);
	if(nvm_crc_valid == false) {
		error_loop();
	}
	
	// Write the metadata to NVM
	status = dd_nvm_row_write(APPLICATION_METADATA_ROW, &am, sizeof(struct application_metadata));
	if(STATUS_OK != status) {
		return status;
	}
			
	// Update the status flag.
	bs->install_flag = INSTALL_FLAG_FALSE;
	status = set_boot_status(&bs);
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

int main (void) {	
	enum status_code status;
	
	// Initialize
	system_init();
	nvm_init();
	at25dfx_init();
	dsu_crc32_init();
	// Enable watchdogs, kick by calling wdt_reset_count() -- Only when ready! Already tested.
	configure_wdt();
	configure_wdt_callbacks();
	system_interrupt_enable_global();
	
	while(true) { }
	
	// Get our boot status
	struct boot_status bs;
	get_boot_status(&bs);
	
	if(check_signature(bs.signature) == false) {
		printf("Invalid boot signature. Boot Status cannot be trusted.");
		printf("Attempting CRC check on FW.");
		if(loaded_fw_valid() == true) {
			// Our FW  is valid, we have corrupted the boot flag. Reset the flag and reboot.
			printf("FW valid. Reseting Boot signature and restarting...");
			get_boot_status_default(&bs);
			set_boot_status(&bs);
			restart();
		}
		else {
			// Our boot status was invalid and the CRC did not match.
			// We should load the golden image from flash to NVM.
			printf("FW invalid. Attempting to load Golden Image");
			load_golden_image();
			restart();
		}
	}
	
	if(bs.install_flag == INSTALL_FLAG_TRUE) {
		printf("We need to install!");
		
		status = do_fw_update(&bs);
		if(STATUS_OK != status) {
			printf("Failed FW update!");
			error_loop();
		}
		
		// All good, reboot.
		restart();
	}
	
	// Application time!
	application_jump();
	
	// Should not get here.
	error_loop();
}	
