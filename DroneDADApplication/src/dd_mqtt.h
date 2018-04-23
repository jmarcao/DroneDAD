#ifndef DD_MQTT_H
#define DD_MQTT_H

#include "asf.h"
#include "driver/include/m2m_wifi.h"
#include "iot/mqtt/mqtt.h"
#include "iot/sw_timer.h"
#include "socket/include/socket.h"
#include "SerialConsole.h"
#include "MPU9150Driver.h"
#include "LEDDriver.h"

/* Max size of UART buffer. */
#define MAIN_CHAT_BUFFER_SIZE 64

/* Max size of MQTT buffer. */
#define MAIN_MQTT_BUFFER_SIZE 128

/* Limitation of user name. */
#define MAIN_CHAT_USER_NAME_SIZE 64

/* Chat MQTT topic. */
#define MAIN_CHAT_TOPIC "dronedad/"
#define SUBSCRIBE_TOPIC "server/"
#define PUBLISH_TOPIC "client/"
#define SUBTOPIC_AX "ax"
#define SUBTOPIC_AY "ay"
#define SUBTOPIC_AZ "az"
#define SUBTOPIC_PITCH "pitch"
#define SUBTOPIC_ROLL "roll"
#define SUBTOPIC_STALLANGLE "stallangle"
#define SUBTOPIC_OTAFU "otafu"

static bool mqtt_ready = false;
static bool mqtt_exit = false; // Exit flag for MQTT loop.

/*
 * A MQTT broker server which was connected.
 * m2m.eclipse.org is public MQTT broker.
 */
static const char main_mqtt_broker[] = "m14.cloudmqtt.com";

/** Wi-Fi Settings */
#define MAIN_WLAN_SSID        "AirPennNet-Device" /* < Destination SSID */
#define MAIN_WLAN_AUTH        M2M_WIFI_SEC_WPA_PSK /* < Security manner */
#define MAIN_WLAN_PSK         "penn1740wifi" /* < Password for Destination SSID */

#define MQTT_USERNAME				"tmcutlfi"  /* < MQTT username */
#define MQTT_PASSWORD				"qy3NS7Qli47X" /* < MQTT password */
#define MQTT_PORT				18768 /* < MQTT PORT */

// MQTT Instance Variables
char mqtt_user[MAIN_CHAT_USER_NAME_SIZE];
static struct mqtt_module mqtt_inst;

/* Receive buffer of the MQTT service. */
static char mqtt_buffer[MAIN_MQTT_BUFFER_SIZE];

/** UART buffer. */
static char uart_buffer[MAIN_CHAT_BUFFER_SIZE];

/** Written size of UART buffer. */
static int uart_buffer_written = 0;

/** A buffer of character from the serial. */
static uint16_t uart_ch_buffer;

/** Instance of Timer module. */
struct sw_timer_module mqtt_swt_module_inst;

/**
* \brief Configure Timer module.
*/
static void mqtt_configure_timer(void)
{
	struct sw_timer_config swt_conf;
	sw_timer_get_config_defaults(&swt_conf);
	sw_timer_init(&mqtt_swt_module_inst, &swt_conf);
	sw_timer_enable(&mqtt_swt_module_inst);
}

static int16_t stallAngle = 60; // Default value.
static float filtered_pitch = 0;
static float filtered_roll = 0;
static const float weight = 0.9;
static struct mpu9150_output_data mpuData;
static void handle_mpu_timeout() {
	get_mpu9150_reading(&mpuData);
	
	float pitch_bottom_eq = (float)sqrt((mpuData.ay * mpuData.ay) + (mpuData.az * mpuData.az));
	float pitch_top_eq = (float)mpuData.ax;
	float pitch = atan(pitch_top_eq / pitch_bottom_eq);
	filtered_pitch = (1-weight)*filtered_pitch + (weight)*pitch;
	
	float roll_bottom_eq = (float)sqrt((mpuData.az * mpuData.az) + (mpuData.ax * mpuData.ax));
	float roll_top_eq = (float)mpuData.ay;
	float roll = atan(roll_top_eq / roll_bottom_eq);
	filtered_roll = (1-weight)*filtered_roll + (weight)*roll;
	
	int16_t int_pitch = (int16_t)filtered_pitch;
	int16_t int_roll = (int16_t)filtered_roll;
	
	int16_t int_pitch_deg = (int16_t)(filtered_pitch*((float)90/1.5708));
	int16_t int_roll_deg = (int16_t)(filtered_roll*((float)90/1.5708));
	
	led_update_stall_warning(abs(int_pitch_deg), stallAngle);
	
	printf("Pitch: %d\r\nRoll: %d\r\n", int_pitch_deg, int_roll_deg);
	char sendBuf[33] = { '\0' };
	itoa(mpuData.ax, sendBuf, 10);
	mqtt_publish(&mqtt_inst, MAIN_CHAT_TOPIC PUBLISH_TOPIC SUBTOPIC_AX, &sendBuf[0], sizeof(char)*33, 0, 0);
	itoa(mpuData.ay, sendBuf, 10);
	mqtt_publish(&mqtt_inst, MAIN_CHAT_TOPIC PUBLISH_TOPIC SUBTOPIC_AY, &sendBuf[0], sizeof(char)*33, 0, 0);
	itoa(mpuData.az, sendBuf, 10);
	mqtt_publish(&mqtt_inst, MAIN_CHAT_TOPIC PUBLISH_TOPIC SUBTOPIC_AZ, &sendBuf[0], sizeof(char)*33, 0, 0);
	itoa(int_pitch_deg, sendBuf, 10);
	mqtt_publish(&mqtt_inst, MAIN_CHAT_TOPIC PUBLISH_TOPIC SUBTOPIC_PITCH, &sendBuf[0], sizeof(char)*33, 0, 0);
	itoa(int_roll_deg, sendBuf, 10);
	mqtt_publish(&mqtt_inst, MAIN_CHAT_TOPIC PUBLISH_TOPIC SUBTOPIC_ROLL, &sendBuf[0], sizeof(char)*33, 0, 0);
}

struct sw_timer_module mpuPoll_swt_module_inst;
static void mpuPoll_configure_timer(void)
{
	struct sw_timer_config swt_conf;
	sw_timer_get_config_defaults(&swt_conf);
	sw_timer_init(&mpuPoll_swt_module_inst, &swt_conf);
	sw_timer_enable(&mpuPoll_swt_module_inst);
	int id = sw_timer_register_callback(&mpuPoll_swt_module_inst, (sw_timer_callback_t)handle_mpu_timeout,
	NULL, 500);
	sw_timer_enable_callback(&mpuPoll_swt_module_inst, id, 0);
}

/************************************************************************/
/* WIFI STUFF, FIGURE OUT A WAY TO COMBINE WITH THE REST OF THE WIFI                                                                     */
/************************************************************************/

/**
 * \brief Callback of USART input.
 *
 * \param[in] module USART module structure.
 */
static void uart_callback(const struct usart_module *const module)
{
	/* If input string is bigger than buffer size limit, ignore the excess part. */
	if (uart_buffer_written < MAIN_CHAT_BUFFER_SIZE) {
		uart_buffer[uart_buffer_written++] = uart_ch_buffer & 0xFF;
	}
}

static void set_uart_callback() {
	usart_register_callback(&cdc_uart_module, (usart_callback_t)uart_callback, USART_CALLBACK_BUFFER_RECEIVED);
}

/**
 * \brief Callback to get the Wi-Fi status update.
 */
static void wifi_callback(uint8 msg_type, void *msg_data)
{
	tstrM2mWifiStateChanged *msg_wifi_state;
	uint8 *msg_ip_addr;

	switch (msg_type) {
	case M2M_WIFI_RESP_CON_STATE_CHANGED:
		msg_wifi_state = (tstrM2mWifiStateChanged *)msg_data;
		if (msg_wifi_state->u8CurrState == M2M_WIFI_CONNECTED) {
			/* If Wi-Fi is connected. */
			printf("Wi-Fi connected\r\n");
			m2m_wifi_request_dhcp_client();
		} else if (msg_wifi_state->u8CurrState == M2M_WIFI_DISCONNECTED) {
			/* If Wi-Fi is disconnected. */
			printf("Wi-Fi disconnected\r\n");
			m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID),
					MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
			/* Disconnect from MQTT broker. */
			/* Force close the MQTT connection, because cannot send a disconnect message to the broker when network is broken. */
			mqtt_disconnect(&mqtt_inst, 1);
		}

		break;

	case M2M_WIFI_REQ_DHCP_CONF:
		msg_ip_addr = (uint8 *)msg_data;
		printf("Wi-Fi IP is %u.%u.%u.%u\r\n",
				msg_ip_addr[0], msg_ip_addr[1], msg_ip_addr[2], msg_ip_addr[3]);
		/* Try to connect to MQTT broker when Wi-Fi was connected. */
		mqtt_connect(&mqtt_inst, main_mqtt_broker);
		break;

	default:
		break;
	}
}

/**
 * \brief Callback to get the Socket event.
 */
static void socket_event_handler(SOCKET sock, uint8_t msg_type, void *msg_data)
{
	mqtt_socket_event_handler(sock, msg_type, msg_data);
}

/**
 * \brief Callback of gethostbyname function.
 *
 * \param[in] doamin_name Domain name.
 * \param[in] server_ip IP of server.
 */
static void socket_resolve_handler(uint8_t *doamin_name, uint32_t server_ip)
{
	mqtt_socket_resolve_handler(doamin_name, server_ip);
}

/**
 * \brief Callback to get the MQTT status update.
 *
 * \param[in] conn_id instance id of connection which is being used.
 * \param[in] type type of MQTT notification. Possible types are:
 *  - [MQTT_CALLBACK_SOCK_CONNECTED](@ref MQTT_CALLBACK_SOCK_CONNECTED)
 *  - [MQTT_CALLBACK_CONNECTED](@ref MQTT_CALLBACK_CONNECTED)
 *  - [MQTT_CALLBACK_PUBLISHED](@ref MQTT_CALLBACK_PUBLISHED)
 *  - [MQTT_CALLBACK_SUBSCRIBED](@ref MQTT_CALLBACK_SUBSCRIBED)
 *  - [MQTT_CALLBACK_UNSUBSCRIBED](@ref MQTT_CALLBACK_UNSUBSCRIBED)
 *  - [MQTT_CALLBACK_DISCONNECTED](@ref MQTT_CALLBACK_DISCONNECTED)
 *  - [MQTT_CALLBACK_RECV_PUBLISH](@ref MQTT_CALLBACK_RECV_PUBLISH)
 * \param[in] data A structure contains notification informations. @ref mqtt_data
 */
static void mqtt_callback(struct mqtt_module *module_inst, int type, union mqtt_data *data)
{
	switch (type) {
	case MQTT_CALLBACK_SOCK_CONNECTED:
	{
		/*
		 * If connecting to broker server is complete successfully, Start sending CONNECT message of MQTT.
		 * Or else retry to connect to broker server.
		 */
		if (data->sock_connected.result >= 0) {
			mqtt_connect_broker(module_inst, 1, MQTT_USERNAME, MQTT_PASSWORD, mqtt_user, NULL, NULL, 0, 0, 0);
		} else {
			printf("Connect fail to server(%s)! retry it automatically.\r\n", main_mqtt_broker);
			mqtt_connect(module_inst, main_mqtt_broker); /* Retry that. */
		}
	}
	break;

	case MQTT_CALLBACK_CONNECTED:
		if (data->connected.result == MQTT_CONN_RESULT_ACCEPT) {
			/* Subscribe chat topic. */
			mqtt_subscribe(module_inst, MAIN_CHAT_TOPIC SUBSCRIBE_TOPIC "#", 0);
			/* Enable USART receiving callback. */
			usart_enable_callback(&cdc_uart_module, USART_CALLBACK_BUFFER_RECEIVED);
			printf("Preparation of the chat has been completed.\r\n");
			mqtt_ready = true;
		} else {
			/* Cannot connect for some reason. */
			printf("MQTT broker decline your access! error code %d\r\n", data->connected.result);
		}

		break;

	case MQTT_CALLBACK_RECV_PUBLISH:
		/* You received publish message which you had subscribed. */
		if (data->recv_publish.topic != NULL && data->recv_publish.msg != NULL) {
			if(!strncmp(data->recv_publish.topic, MAIN_CHAT_TOPIC SUBSCRIBE_TOPIC SUBTOPIC_STALLANGLE, 
					strlen(MAIN_CHAT_TOPIC SUBSCRIBE_TOPIC SUBTOPIC_STALLANGLE)))
			{
				int16_t incoming_angle = 0;
				char incoming_angle_buf[16] = { '\0' };
				for (int i = 0; i < data->recv_publish.msg_size; i++) {
					incoming_angle_buf[i] = data->recv_publish.msg[i];
				}
				incoming_angle = atoi(incoming_angle_buf);
				printf("Received Stall Angle Update: %d\r\n",  incoming_angle);
				if((incoming_angle < 90) && (incoming_angle > 0)) {
					stallAngle = incoming_angle;
				}
				else {
					fprintf(stderr, "Invalid Stall Angle value!\r\n");
				}
			}
			else if(!strncmp(data->recv_publish.topic, MAIN_CHAT_TOPIC SUBSCRIBE_TOPIC SUBTOPIC_OTAFU,
					strlen(MAIN_CHAT_TOPIC SUBSCRIBE_TOPIC SUBTOPIC_OTAFU)))
			{
				printf("OTAFU Requested\r\n");
				// Clean up WIFI as we will reconnect.
				m2m_wifi_disconnect();
				m2m_wifi_deinit(NULL);
				socketDeinit();
				// Hanlde request
				handleUpdateRequest();
				mqtt_exit = true;
			}
			else if (!strncmp(data->recv_publish.topic, MAIN_CHAT_TOPIC, strlen(MAIN_CHAT_TOPIC))) {
				/* Print user name and message */
				for (int i = strlen(MAIN_CHAT_TOPIC); i < data->recv_publish.topic_size; i++) {
					printf("%c", data->recv_publish.topic[i]);
				}
				printf(" >> ");
				for (int i = 0; i < data->recv_publish.msg_size; i++) {
					printf("%c", data->recv_publish.msg[i]);
				}
				printf("\r\n");
			}
		}

		break;

	case MQTT_CALLBACK_DISCONNECTED:
		/* Stop timer and USART callback. */
		printf("MQTT disconnected\r\n");
		usart_disable_callback(&cdc_uart_module, USART_CALLBACK_BUFFER_RECEIVED);
		break;
	}
}




/************************************************************************/
/* MQTT                                                                     */
/************************************************************************/

/**
 * \brief Configure MQTT service.
 */
static void configure_mqtt(void)
{
	struct mqtt_config mqtt_conf;
	int result;
	
	mqtt_configure_timer();

	mqtt_get_config_defaults(&mqtt_conf);
	/* To use the MQTT service, it is necessary to always set the buffer and the timer. */
	mqtt_conf.timer_inst = &mqtt_swt_module_inst;
	mqtt_conf.recv_buffer = mqtt_buffer;
	mqtt_conf.recv_buffer_size = MAIN_MQTT_BUFFER_SIZE;
	mqtt_conf.port = MQTT_PORT;
	
	result = mqtt_init(&mqtt_inst, &mqtt_conf);
	if (result < 0) {
		printf("MQTT initialization failed. Error code is (%d)\r\n", result);
		while (1) {
		}
	}

	result = mqtt_register_callback(&mqtt_inst, mqtt_callback);
	if (result < 0) {
		printf("MQTT register callback failed. Error code is (%d)\r\n", result);
		while (1) {
		}
	}
}

/**
 * \brief Checking the USART buffer.
 *
 * Finding the new line character(\n or \r\n) in the USART buffer.
 * If buffer was overflowed, Sending the buffer.
 */
static void check_usart_buffer(char *topic)
{
	int i;

	/* Publish the input string when newline was received or input string is bigger than buffer size limit. */
	if (uart_buffer_written >= MAIN_CHAT_BUFFER_SIZE) {
		mqtt_publish(&mqtt_inst, topic, uart_buffer, MAIN_CHAT_BUFFER_SIZE, 0, 0);
		uart_buffer_written = 0;
	} else {
		for (i = 0; i < uart_buffer_written; i++) {
			/* Find newline character ('\n' or '\r\n') and publish the previous string . */
			if (uart_buffer[i] == '\n' || uart_buffer[i] == '\r' ) {
				mqtt_publish(&mqtt_inst, topic, uart_buffer, (i > 0 && uart_buffer[i - 1] == '\r') ? i - 1 : i, 0, 0);
				/* Move remain data to start of the buffer. */
				if (uart_buffer_written > i + 1) {
					memmove(uart_buffer, uart_buffer + i + 1, uart_buffer_written - i - 1);
					uart_buffer_written = uart_buffer_written - i - 1;
				} else {
					uart_buffer_written = 0;
				}

				break;
			}
		}
	}
}

static void dd_mqtt_loop() {
	tstrWifiInitParam param;
	int8_t ret;
	char topic[strlen(MAIN_CHAT_TOPIC) + MAIN_CHAT_USER_NAME_SIZE + 1];
	
	set_uart_callback();
	mpuPoll_configure_timer();
	
	/* Setup user name first */
	mqtt_user[0] = 'c';
	sprintf(topic, "%s%s", MAIN_CHAT_TOPIC, PUBLISH_TOPIC);

	/* Initialize Wi-Fi parameters structure. */
	memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));

	/* Initialize Wi-Fi driver with data and status callbacks. */
	param.pfAppWifiCb = wifi_callback; /* Set Wi-Fi event callback. */
	ret = m2m_wifi_init(&param);
	if (M2M_SUCCESS != ret) {
		printf("main: m2m_wifi_init call error!(%d)\r\n", ret);
		while (1) { /* Loop forever. */
		}
	}

	/* Initialize socket interface. */
	socketInit();
	registerSocketCallback(socket_event_handler, socket_resolve_handler);

	/* Connect to router. */
	m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID),
	MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);

	while (!mqtt_exit) {
		/* Handle pending events from network controller. */
		m2m_wifi_handle_events(NULL);
		/* Try to read user input from USART. */
		usart_read_job(&cdc_uart_module, &uart_ch_buffer);
		/* Checks the timer timeout. */
		sw_timer_task(&mqtt_swt_module_inst);
		if(mqtt_ready) {
			sw_timer_task(&mpuPoll_swt_module_inst);
		}
		/* Checks the USART buffer. */
		check_usart_buffer(topic);
	}
}


#endif
