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
