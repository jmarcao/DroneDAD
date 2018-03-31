#include <asf.h>
#define APP_START_ADDR 0x2000
#define BOOT_STATUS_ADDR 0x1F00

#define INSTALL_FLAG_TRUE = 0xFF;
#define INSTALL_FLAG_FALSE = 0x00;
const uint8_t DRONEDAD_BOOT_SIGNATURE = {0xDD, 0xAD, 0x20 0x18 };
	
struct boot_status {
	uint8_t signature[8];
	uint8_t install_flag; // 
	uint8_t install_idx;
	uint8_t application_star_addr; // Either set to APP_START_ADDR or leave unused. We'll see the usefulness.
}; // Total = 88 bytes.

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

enum status_code dd_nvm_row_read(int row, uint8_t* data, int* data_len) {
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
		status = nvm_erase_row(row);	
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
	const int boot_status_row = (BOOT_STATUS_ADDR / NVMCTRL_PAGE_SIZE) / NVMCTRL_ROW_PAGES;
	enum status_code status;
	
	status = dd_nvm_row_read(boot_status_row, (uint8_t*)bs, sizeof struct boot_status);
	
	return status;
}

/*
 * An error has occurred. This loop will blink the LED in a distinct way to let the user know
 * a Boot loader error has occurred.
 */
void error_loop() {
	
}

int main (void)
{		
	// Initialize
	system_init();
	nvm_init();
	
	struct boot_status bs;
	get_boot_status(&bs);
	
	if(bs.install_flag == INSTALL_FLAG_TRUE) {
		printf("We need to install!");
		
		// Load data into RAM.
		
		// Check CRC one more time.
		
		// Write to NVM.
		
		// Check CRC again!
		
		// Update the status flag.
	}
	else {
		// Normal operation, continue.
		
		// Possibly, for safety, compute CRC of current application code. (If it doesnt take too long).
		
		// All good, jump.
		application_jump();
		// OR
		error_loop();
	}
}	
