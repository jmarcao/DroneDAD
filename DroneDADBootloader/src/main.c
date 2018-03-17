/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# Minimal main function that starts with a call to system_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>
#define APP_START_ADDR 0x2000

int main (void)
{		
	system_init();

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
