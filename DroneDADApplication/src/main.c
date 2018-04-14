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

#define STRING_EOL                      "\r\n"
#define STRING_HEADER                   "-- HTTP file downloader example --"STRING_EOL \
"-- "BOARD_NAME " --"STRING_EOL	\
"-- Compiled: "__DATE__ " "__TIME__ " --"STRING_EOL

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

/** File download processing state. */
static download_state down_state = NOT_READY;

/** UART module for debug. */
static struct usart_module cdc_uart_module;

/** Instance of Timer module. */
struct sw_timer_module swt_module_inst;

/** Instance of HTTP client module. */
struct http_client_module http_client_module_inst;

/**
	* \brief Initialize download state to not ready.
	*/
static void init_state(void)
{
	down_state = NOT_READY;
}

/**
	* \brief Clear state parameter at download processing state.
	* \param[in] mask Check download_state.
	*/
static void clear_state(download_state mask)
{
	down_state &= ~mask;
}

/**
	* \brief Add state parameter at download processing state.
	* \param[in] mask Check download_state.
	*/
static void add_state(download_state mask)
{
	down_state |= mask;
}

/**
	* \brief File download processing state check.
	* \param[in] mask Check download_state.
	* \return true if this state is set, false otherwise.
	*/

static inline bool is_state_set(download_state mask)
{
	return ((down_state & mask) != 0);
}

/**
	* \brief Start file download via HTTP connection.
	*/
static void start_download(void)
{
	if (!is_state_set(WIFI_CONNECTED)) {
		printf("start_download: Wi-Fi is not connected.\r\n");
		return;
	}

	if (is_state_set(GET_REQUESTED)) {
		printf("start_download: request is sent already.\r\n");
		return;
	}

	if (is_state_set(DOWNLOADING)) {
		printf("start_download: running download already.\r\n");
		return;
	}

	/* Send the HTTP request. */
	printf("start_download: sending HTTP request...\r\n");
	http_client_send_request(&http_client_module_inst, MAIN_HTTP_FILE_URL, HTTP_METHOD_GET, NULL, NULL);
}

/**
	* \brief Store received packet to file.
	* \param[in] data Packet data.
	* \param[in] length Packet data length.
	*/
static void store_file_packet(char *data, uint32_t length)
{
	printf("Data");
}

/**
	* \brief Callback of the HTTP client.
	*
	* \param[in]  module_inst     Module instance of HTTP client module.
	* \param[in]  type            Type of event.
	* \param[in]  data            Data structure of the event. \refer http_client_data
	*/
static void http_client_callback(struct http_client_module *module_inst, int type, union http_client_data *data)
{
	switch (type) {
	case HTTP_CLIENT_CALLBACK_SOCK_CONNECTED:
		printf("http_client_callback: HTTP client socket connected.\r\n");
		break;

	case HTTP_CLIENT_CALLBACK_REQUESTED:
		printf("http_client_callback: request completed.\r\n");
		add_state(GET_REQUESTED);
		break;

	case HTTP_CLIENT_CALLBACK_RECV_RESPONSE:
		printf("http_client_callback: received response %u data size %u\r\n",
				(unsigned int)data->recv_response.response_code,
				(unsigned int)data->recv_response.content_length);
		if ((unsigned int)data->recv_response.response_code == 200) {
			//http_file_size = data->recv_response.content_length;
			//received_file_size = 0;
		} 
		else {
			add_state(CANCELED);
			return;
		}
		
		if (data->recv_response.content_length <= MAIN_BUFFER_MAX_SIZE) {
			store_file_packet(data->recv_response.content, data->recv_response.content_length);
			add_state(COMPLETED);
		}
		
		break;

	case HTTP_CLIENT_CALLBACK_RECV_CHUNKED_DATA:
		store_file_packet(data->recv_chunked_data.data, data->recv_chunked_data.length);
		if (data->recv_chunked_data.is_complete) {
			add_state(COMPLETED);
		}

		break;

	case HTTP_CLIENT_CALLBACK_DISCONNECTED:
		printf("http_client_callback: disconnection reason:%d\r\n", data->disconnected.reason);

		/* If disconnect reason is equal to -ECONNRESET(-104),
			* It means the server has closed the connection (timeout).
			* This is normal operation.
			*/
		if (data->disconnected.reason == -EAGAIN) {
			/* Server has not responded. Retry immediately. */
			if (is_state_set(DOWNLOADING)) {
				//f_close(&file_object);
				clear_state(DOWNLOADING);
			}

			if (is_state_set(GET_REQUESTED)) {
				clear_state(GET_REQUESTED);
			}

			start_download();
		}

		break;
	}
}

/**
	* \brief Callback to get the data from socket.
	*
	* \param[in] sock socket handler.
	* \param[in] u8Msg socket event type. Possible values are:
	*  - SOCKET_MSG_BIND
	*  - SOCKET_MSG_LISTEN
	*  - SOCKET_MSG_ACCEPT
	*  - SOCKET_MSG_CONNECT
	*  - SOCKET_MSG_RECV
	*  - SOCKET_MSG_SEND
	*  - SOCKET_MSG_SENDTO
	*  - SOCKET_MSG_RECVFROM
	* \param[in] pvMsg is a pointer to message structure. Existing types are:
	*  - tstrSocketBindMsg
	*  - tstrSocketListenMsg
	*  - tstrSocketAcceptMsg
	*  - tstrSocketConnectMsg
	*  - tstrSocketRecvMsg
	*/
static void socket_cb(SOCKET sock, uint8_t u8Msg, void *pvMsg)
{
	http_client_socket_event_handler(sock, u8Msg, pvMsg);
}

/**
	* \brief Callback for the gethostbyname function (DNS Resolution callback).
	* \param[in] pu8DomainName Domain name of the host.
	* \param[in] u32ServerIP Server IPv4 address encoded in NW byte order format. If it is Zero, then the DNS resolution failed.
	*/
static void resolve_cb(uint8_t *pu8DomainName, uint32_t u32ServerIP)
{
	printf("resolve_cb: %s IP address is %d.%d.%d.%d\r\n\r\n", pu8DomainName,
			(int)IPV4_BYTE(u32ServerIP, 0), (int)IPV4_BYTE(u32ServerIP, 1),
			(int)IPV4_BYTE(u32ServerIP, 2), (int)IPV4_BYTE(u32ServerIP, 3));
	http_client_socket_resolve_handler(pu8DomainName, u32ServerIP);
}

/**
	* \brief Callback to get the Wi-Fi status update.
	*
	* \param[in] u8MsgType type of Wi-Fi notification. Possible types are:
	*  - [M2M_WIFI_RESP_CURRENT_RSSI](@ref M2M_WIFI_RESP_CURRENT_RSSI)
	*  - [M2M_WIFI_RESP_CON_STATE_CHANGED](@ref M2M_WIFI_RESP_CON_STATE_CHANGED)
	*  - [M2M_WIFI_RESP_CONNTION_STATE](@ref M2M_WIFI_RESP_CONNTION_STATE)
	*  - [M2M_WIFI_RESP_SCAN_DONE](@ref M2M_WIFI_RESP_SCAN_DONE)
	*  - [M2M_WIFI_RESP_SCAN_RESULT](@ref M2M_WIFI_RESP_SCAN_RESULT)
	*  - [M2M_WIFI_REQ_WPS](@ref M2M_WIFI_REQ_WPS)
	*  - [M2M_WIFI_RESP_IP_CONFIGURED](@ref M2M_WIFI_RESP_IP_CONFIGURED)
	*  - [M2M_WIFI_RESP_IP_CONFLICT](@ref M2M_WIFI_RESP_IP_CONFLICT)
	*  - [M2M_WIFI_RESP_P2P](@ref M2M_WIFI_RESP_P2P)
	*  - [M2M_WIFI_RESP_AP](@ref M2M_WIFI_RESP_AP)
	*  - [M2M_WIFI_RESP_CLIENT_INFO](@ref M2M_WIFI_RESP_CLIENT_INFO)
	* \param[in] pvMsg A pointer to a buffer containing the notification parameters
	* (if any). It should be casted to the correct data type corresponding to the
	* notification type. Existing types are:
	*  - tstrM2mWifiStateChanged
	*  - tstrM2MWPSInfo
	*  - tstrM2MP2pResp
	*  - tstrM2MAPResp
	*  - tstrM2mScanDone
	*  - tstrM2mWifiscanResult
	*/
static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
	switch (u8MsgType) {
	case M2M_WIFI_RESP_CON_STATE_CHANGED:
	{
		tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
		if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
			printf("wifi_cb: M2M_WIFI_CONNECTED\r\n");
			m2m_wifi_request_dhcp_client();
		} else if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
			printf("wifi_cb: M2M_WIFI_DISCONNECTED\r\n");
			clear_state(WIFI_CONNECTED);
			if (is_state_set(DOWNLOADING)) {
				//f_close(&file_object);
				clear_state(DOWNLOADING);
			}

			if (is_state_set(GET_REQUESTED)) {
				clear_state(GET_REQUESTED);
			}

			m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID),
					MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
		}

		break;
	}

	case M2M_WIFI_REQ_DHCP_CONF:
	{
		uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
		printf("wifi_cb: IP address is %u.%u.%u.%u\r\n",
				pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
		add_state(WIFI_CONNECTED);
		start_download();
		break;
	}

	default:
		break;
	}
}

/**
	* \brief Configure Timer module.
	*/
static void configure_timer(void)
{
	struct sw_timer_config swt_conf;
	sw_timer_get_config_defaults(&swt_conf);

	sw_timer_init(&swt_module_inst, &swt_conf);
	sw_timer_enable(&swt_module_inst);
}

/**
	* \brief Configure HTTP client module.
	*/
static void configure_http_client(void)
{
	struct http_client_config httpc_conf;
	int ret;

	http_client_get_config_defaults(&httpc_conf);

	httpc_conf.recv_buffer_size = MAIN_BUFFER_MAX_SIZE;
	httpc_conf.timer_inst = &swt_module_inst;

	ret = http_client_init(&http_client_module_inst, &httpc_conf);
	if (ret < 0) {
		printf("configure_http_client: HTTP client initialization failed! (res %d)\r\n", ret);
		while (1) {
		} /* Loop forever. */
	}
		
	http_client_register_callback(&http_client_module_inst, http_client_callback);
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

int main (void) {
	/* Initialize the board. */
	system_init();
	at25dfx_init();
	dsu_crc32_init();
	nvm_init();
	configure_i2c();
	delay_init();
	lp3944_init();
	lsm6ds3_init();
	
	float temp;
	temp = readTempF();
	
	while(1) {}	
		
	tstrWifiInitParam param;
	int8_t ret;
	init_state();

	/* Initialize the UART console. */
	serial_init();
	printf(STRING_HEADER);
	printf("\r\nThis example requires the AP to have internet access.\r\n\r\n");

	/* Initialize the Timer. */
	configure_timer();

	/* Initialize the HTTP client service. */
	configure_http_client();

	/* Initialize the BSP. */
	nm_bsp_init();

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
