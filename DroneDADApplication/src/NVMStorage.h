/*
 * NVMStorage.h
 *
 * Created: 4/16/2018 10:56:26 PM
 *  Author: John
 */ 


#ifndef NVMSTORAGE_H_
#define NVMSTORAGE_H_

// NVM Information
#define NVM_NUMBER_OF_PAGES 0x1000
#define NVM_NUMBER_OF_ROWS (NVM_NUMBER_OF_PAGES / 0x4) // 1024 rows

// NVM Locations
#define APPLICATION_ROW 40
#define APP_START_ADDR (APPLICATION_ROW * NVMCTRL_ROW_PAGES * NVMCTRL_PAGE_SIZE) // 0x2800 for Row 40
#define BOOT_STATUS_ROW (NVM_NUMBER_OF_ROWS - 1) // Store boot status in second to last row. (1023)
#define APPLICATION_METADATA_ROW (NVM_NUMBER_OF_ROWS - 2) // Store metadata inn third to last row. (1022)

// Install Flags
#define INSTALL_FLAG_TRUE 0xFF
#define INSTALL_FLAG_FALSE 0x00

// Signature and BL Version.
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

void nvm_init(void)
{
	struct nvm_config config_nvm;
	nvm_get_config_defaults(&config_nvm);
	config_nvm.manual_page_write = false;
	nvm_set_config(&config_nvm);
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
	struct boot_status bss;
	dd_nvm_row_read(BOOT_STATUS_ROW, (uint8_t*)&bss, sizeof (struct boot_status));
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


#endif /* NVMSTORAGE_H_ */