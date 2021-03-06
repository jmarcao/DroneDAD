/*
 * HttpDownloader.h
 *
 * Created: 4/15/2018 6:29:35 PM
 *  Author: John
 */ 


#ifndef HTTPDOWNLOADER_H_
#define HTTPDOWNLOADER_H_

#include "errno.h"

#include "FlashStorage.h"
#include "NVMStorage.h"
#include "LEDDriver.h"

/** File download processing state. */
static download_state dl_state = NOT_READY;

/** UART module for debug. */
static struct usart_module cdc_uart_module;

/** Instance of HTTP client module. */
struct http_client_module http_client_module_inst;

typedef struct {
	char data[MAX_METADATA_BUFFER_LEN];
	uint16_t begin;
	uint16_t end;
} metadata_buffer_t;

typedef struct {
	char data[MAX_FIRMWARE_BUFFER_LEN];
	uint16_t begin;
	uint16_t end;
} firmware_buffer_t;

static metadata_buffer_t metadata_buffer = {
	.begin = 0,
	.end = 0
};
static firmware_buffer_t firmware_buffer;
static uint32_t fw_writeStart = 0x2000;
static uint32_t fw_writeAddr = 0x2000;
static uint32_t fw_metadataAddr = 0x1000;
static uint32_t fw_rollingCRC;
static uint32_t fw_dataLen;
static uint32_t metadata_crc;
static char serverVersion[METADATA_VERSION_LENGTH+1];
static char metadata_crc_buffer[8 + 1];

/** Instance of Timer module. */
struct sw_timer_module http_swt_module_inst;

/**
* \brief Configure Timer module.
*/
void configure_timer(void)
{
	struct sw_timer_config swt_conf;
	sw_timer_get_config_defaults(&swt_conf);
	sw_timer_init(&http_swt_module_inst, &swt_conf);
	sw_timer_enable(&http_swt_module_inst);
}

static void wifiState_init(void)
{
	dl_state = NOT_READY;
}

static void clear_state(download_state mask)
{
	dl_state &= ~mask;
}

static void add_state(download_state mask)
{
	dl_state |= mask;
}

static inline bool is_state_set(download_state mask)
{
	return ((dl_state & mask) != 0);
}

/**
	* \brief Start file download via HTTP connection.
	*/
static void sendHttpReq_metadata(void)
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
	http_client_send_request(&http_client_module_inst, URL_METADATA, HTTP_METHOD_GET, NULL, NULL);
}

static void sendHttpReq_firmware(void)
{
	fw_rollingCRC = 0;
	
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
	http_client_send_request(&http_client_module_inst, URL_LATEST_FIRMWARE, HTTP_METHOD_GET, NULL, NULL);
}

/**
	* \brief Store received packet to file.
	* \param[in] data Packet data.
	* \param[in] length Packet data length.
	*/
static void store_metadata_file(char *data, uint32_t length)
{	
	for(int i = 0; i < length; i++) {
		metadata_buffer.data[metadata_buffer.end] = *(data + (i * sizeof(char)));
		metadata_buffer.end++;
	}
}

static void parse_metadata_buffer() {
	if(metadata_buffer.data[0] != 'V') {
		printf("Unexpected data in metadata_buffer!\r\n");
		return;
	}

	// Store version
	for(int i = 0; i < METADATA_VERSION_LENGTH; i++) {
		serverVersion[i] = metadata_buffer.data[i+1];
	}
	serverVersion[METADATA_VERSION_LENGTH] = '\0';
	
	// Store CRC
	for(int i = 0; i < 8; i++) {
		metadata_crc_buffer[i] = metadata_buffer.data[i+8];
	}
	metadata_crc_buffer[8] = '\0';
	
	printf("Server Version is %s\r\n", &serverVersion[0]);
	printf("Server's FW CRC is %s\r\n", &metadata_crc_buffer[0]);
	//metadata_crc = strtol(metadata_crc_buffer, NULL, 16);
}

static void store_firmware_file(char *data, uint32_t length)
{
	printf("Storing %d bytes in buffer\r\n", length);
	fw_dataLen += length;

	uint32_t dataBegin = 0;
	uint32_t dataEnd = length;
	
	// Also calculate CRC while we have the data.
	crc32_recalculate(&data[0], length, &fw_rollingCRC);

	for(int i = dataBegin; i < dataEnd; i++) {
		firmware_buffer.data[firmware_buffer.end] = data[i];
		firmware_buffer.end++;
		if(firmware_buffer.end == MAX_FIRMWARE_BUFFER_LEN) {
			// We filled our buffer, we should write it to flash and clear before continuing.
			dd_flash_write_data(fw_writeAddr, &firmware_buffer.data[0], firmware_buffer.end);
			fw_writeAddr += MAX_FIRMWARE_BUFFER_LEN;

			// Reset our trackers
			firmware_buffer.begin = 0;
			firmware_buffer.end = 0;
		}
	}
}

static void finalize_firmware_update() {	
	if(firmware_buffer.end != 0) {
		// We have some leftover data to write. Write it!
		dd_flash_write_data(fw_writeAddr, &firmware_buffer.data[0], firmware_buffer.end);
		firmware_buffer.begin = 0;
		firmware_buffer.end = 0;
	}

	char fw_rollingCRCBuffer[9];
	sprintf(fw_rollingCRCBuffer, "%x", fw_rollingCRC);
	if(!strcmp(&metadata_crc_buffer[0], &fw_rollingCRCBuffer[0])) {
		printf("CRC Verified.\r\n");
		// CRC matched. Our download was all good.
		struct application_metadata md;
		md.crc = fw_rollingCRC;
		md.data_len = fw_dataLen;
		md.index = 0;
		md.start_addr = fw_writeStart;
		dd_flash_write_data(fw_metadataAddr, &md, sizeof(struct application_metadata));
		
		struct flash_header fh;
		fh.metadata_addr[0] = fw_metadataAddr;
		dd_flash_write_data(0x0, &fh, sizeof(struct flash_header));

		// Set flag in boot status to alert we should update on next boot.
		struct boot_status bs;
		get_boot_status(&bs);
		bs.install_flag = INSTALL_FLAG_TRUE;
		bs.install_idx = 0;
		set_boot_status(&bs);
		
		printf("Wrote FW Image to flash. Will update on next boot.\r\n");
	}
	else {
		printf("Firmware download completed, but CRC was not correct!\r\n");
		printf("Expected %x but was %x\r\n", metadata_crc, fw_rollingCRC);
	}
}

/**
	* \brief Callback of the HTTP client.
	*
	* \param[in]  module_inst     Module instance of HTTP client module.
	* \param[in]  type            Type of event.
	* \param[in]  data            Data structure of the event. \refer http_client_data
	*/
static void http_client_metadata_req_callback(struct http_client_module *module_inst, int type, union http_client_data *data)
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
				
		if ((unsigned int)data->recv_response.response_code != 200) {
			printf("Download canceled.");
			add_state(CANCELED);
			return;
		}
		
		if (data->recv_response.content_length <= MAIN_BUFFER_MAX_SIZE) {
			store_metadata_file(data->recv_response.content, data->recv_response.content_length);
			add_state(COMPLETED);
		}
		
		break;

	case HTTP_CLIENT_CALLBACK_RECV_CHUNKED_DATA:
		store_metadata_file(data->recv_chunked_data.data, data->recv_chunked_data.length);
		if (data->recv_chunked_data.is_complete) {
			add_state(COMPLETED);
		}

		break;

	case HTTP_CLIENT_CALLBACK_DISCONNECTED:
		printf("http_client_callback: disconnection reason:%d\r\n", data->disconnected.reason);

		/* If disconnect reason is equal to -ECONNRESET(-104),
			* It means the server has closed the connection (timeout).
			* This is normal operation.
			* Sometimes we lose network too, just try again. Our wifi module is not very strong.
			*/
		if ((data->disconnected.reason == -EAGAIN) || (data->disconnected.reason == -ETIME)) {
			/* Server has not responded. Retry immediately. */
			if (is_state_set(DOWNLOADING)) {
				//f_close(&file_object);
				clear_state(DOWNLOADING);
			}

			if (is_state_set(GET_REQUESTED)) {
				clear_state(GET_REQUESTED);
			}

			sendHttpReq_metadata();
		}

		break;
	}
	
	if(is_state_set(COMPLETED)) {
		parse_metadata_buffer();
	}
}

static void http_client_firmware_download_req_callback(struct http_client_module *module_inst, int type, union http_client_data *data)
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
				
		if ((unsigned int)data->recv_response.response_code != 200) {
			printf("Download canceled.");
			add_state(CANCELED);
			return;
		}
		
		if (data->recv_response.content_length <= MAIN_BUFFER_MAX_SIZE) {
			store_firmware_file(data->recv_response.content, data->recv_response.content_length);
			add_state(COMPLETED);
		}
		
		break;

	case HTTP_CLIENT_CALLBACK_RECV_CHUNKED_DATA:
		store_firmware_file(data->recv_chunked_data.data, data->recv_chunked_data.length);
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

			sendHttpReq_firmware();
		}

		break;
	}

	if(is_state_set(COMPLETED)) {
		finalize_firmware_update();
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
		} else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
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
		break;
	}

	default:
		break;
	}
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
	httpc_conf.timer_inst = &http_swt_module_inst;

	ret = http_client_init(&http_client_module_inst, &httpc_conf);
	if (ret < 0) {
		printf("configure_http_client: HTTP client initialization failed! (res %d)\r\n", ret);
		while (1) {
		} /* Loop forever. */
	}
		
	http_client_register_callback(&http_client_module_inst, http_client_metadata_req_callback);
}

static uint32_t getServerFirmwareVersion() {
	// Set the Firmware handler callback
	http_client_register_callback(&http_client_module_inst, http_client_metadata_req_callback);
	sendHttpReq_metadata();
	
	while (!(is_state_set(COMPLETED) || is_state_set(CANCELED))) {
		/* Handle pending events from network controller. */
		m2m_wifi_handle_events(NULL);
		/* Checks the timer timeout. */
		sw_timer_task(&http_swt_module_inst);
	}
	clear_state(GET_REQUESTED);
	clear_state(DOWNLOADING);
	clear_state(COMPLETED);

	uint32_t versionNumber = atoi(serverVersion);
	return versionNumber;
}

static bool downloadFirmwareUpdate() {
	// Set the Firmware handler callback
	http_client_register_callback(&http_client_module_inst, http_client_firmware_download_req_callback);
	sendHttpReq_firmware();
	
	while (!(is_state_set(COMPLETED) || is_state_set(CANCELED))) {
		/* Handle pending events from network controller. */
		m2m_wifi_handle_events(NULL);
		/* Checks the timer timeout. */
		sw_timer_task(&http_swt_module_inst);
	}
	clear_state(GET_REQUESTED);
	clear_state(DOWNLOADING);
	clear_state(COMPLETED);
}

static void handleUpdateRequest() {
	tstrWifiInitParam param;
	int8_t ret;
	
	// Special LED Pattern to signify FW Update
	set_otafu_lighting();
		
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
	while (!(is_state_set(WIFI_CONNECTED) || is_state_set(CANCELED))) {
		/* Handle pending events from network controller. */
		m2m_wifi_handle_events(NULL);
		/* Checks the timer timeout. */
		sw_timer_task(&http_swt_module_inst);
	}
	
	// Download the latest firmware metadata from the server.
	printf("Downloading metadata....\r\n");
	uint32_t latestVersion = getServerFirmwareVersion();
	
	
	int currentVersion = 0;
	if(latestVersion > currentVersion) {
		printf("Update available. Downloading...\r\n");
		downloadFirmwareUpdate();
	}
	else {
		printf("Latest Version already installed.");
	}
}




#endif /* HTTPDOWNLOADER_H_ */