/*
 * FlashStorage.h
 *
 * Created: 4/16/2018 10:22:19 PM
 *  Author: John
 */ 


#ifndef FLASHSTORAGE_H_
#define FLASHSTORAGE_H_

// Flash Information
#define MAX_APPLICATION_COUNT 3
#define FLASH_HEADER_ADDR 0x0
#define AT25DFX_BUFFER_SIZE  (512)

// Static members
static uint8_t read_buffer[AT25DFX_BUFFER_SIZE] = { 0 };
static uint8_t write_buffer[AT25DFX_BUFFER_SIZE] = { 0 };
struct spi_module at25dfx_spi;
struct at25dfx_chip_module at25dfx_chip;

// Data Structures
struct application_metadata {
	uint32_t index;
	uint32_t crc;
	uint32_t start_addr;
	uint32_t data_len;
};

struct flash_header {
	uint32_t metadata_addr[MAX_APPLICATION_COUNT];
	uint8_t last_index;
};


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

static enum status_code dd_flash_read_data(uint32_t addr, uint8_t* buffer, uint32_t buffer_len) {
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

static enum status_code dd_flash_write_data(uint32_t addr, uint8_t* buffer, uint32_t buffer_len) {
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

static enum status_code get_application_metadata_addr(uint8_t index, uint32_t* addr) {
	enum status_code status;
	struct flash_header fh;
	
	status = dd_flash_read_data(FLASH_HEADER_ADDR, &fh, sizeof (struct flash_header));
	if(status != STATUS_OK) {
		return status;
	}
	
	*addr = fh.metadata_addr[index];
	
	return status;
}

static enum status_code get_application_metadata(uint8_t index, struct application_metadata* am) {
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

static enum status_code get_flash_header(struct flash_header* fh) {
	enum status_code status;
	
	status = dd_flash_read_data(FLASH_HEADER_ADDR, &fh, sizeof (struct flash_header));
	if(status != STATUS_OK) {
		return status;
	}
	
	return status;	
}

#endif /* FLASHSTORAGE_H_ */