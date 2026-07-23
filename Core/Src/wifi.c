/*
 * wifi.c
 *
 *  Created on: Mar 11, 2026
 *      Author: buiqu
 */

#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "wifi.h"
#include "uart_logger.h"
#include "uart_driver.h"
#include "cmsis_os.h"
#include "can_storage.h"

#define WIFI_HEALTH_LED_PIN GPIO_PIN_1
#define WIFI_HEALTH_LED_PORT GPIOD

#define WIFI_ERR_LED_PIN GPIO_PIN_3
#define WIFI_ERR_LED_PORT GPIOD

/* List of Commands */
#define ESP_CMD_HEALTH       "AT\r\n"
#define ESP_CMD_ECHO_OFF     "ATE0\r\n"

#define ESP_CMD_SET_MODE     "AT+CWMODE=%d\r\n"
#define ESP_CMD_JOIN_AP      "AT+CWJAP=\"%s\",\"%s\"\r\n"
#define ESP_CMD_DHCP_MODE    "AT+CWDHCP=%d,%d\r\n"
#define ESP_CMD_QUIT_AP	 	 "AT+CWQAP\r\n"
#define ESP_CMD_UDP_START    "AT+CIPSTART=\"UDP\",\"%s\",%d\r\n"
#define ESP_CMD_SEND_BYTES   "AT+CIPSEND=%d\r\n"
#define ESP_CMD_AT_VERSION	 "AT+GMR\r\n"
#define ESP_CMD_RESET		 "AT+RST\r\n"
#define ESP_CMD_MAC_ADDR	 "AT+CIPSTAMAC?\r\n"
#define ESP_CMD_DHCP_STATUS	 "AT+CWDHCP?\r\n"
#define ESP_CMD_SET_STATIC_IP	 "AT+CIPSTA=\"%s\",\"%s\",\"%s\"\r\n"
#define ESP_CMD_IP_STATUS	 "AT+CIPSTA?\r\n"

#define ESP_READY_STR "ready"
#define ESP_OK "OK"
#define ESP_ERR "ERROR"
#define ESP_SEND_OK "SEND OK"

#define ESP_DHCP_ENABLED 1
#define ESP_DHCP_DISABLED 0

// WiFi mode: 1 = Station, 2 = SoftAP, 3 = SoftAP+Station
#define ESP_MODE_STA 1

#if WIFI_LOG_ENABLED
static void wifi_logln_impl(const char *msg) {
	uart_logger_add_msg_format("[Wifi] %s\r\n", msg);
}

static void wifi_log_fmt_ln_impl(const char *fmt, ...) {
	char buf[128];
	va_list args;

	va_start(args, fmt);
	int len = vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);

	if (len > 0 && len < (int) sizeof(buf)) {
		uart_logger_add_msg_format("[Wifi] %s\r\n", buf);
	}
}

#define wifi_log(msg) ((void) wifi_logln_impl((msg)))
#define wifi_log_fmt(fmt, ...) ((void) wifi_log_fmt_ln_impl((fmt), ##__VA_ARGS__))
#endif

typedef struct {
	uint8_t message_type;
	uint32_t id;
	uint8_t len;
	uint8_t payload[WIFI_MAX_DATA_LEN];
} wifi_payload_t;

static void WIFI_Task(void *argument);

static osThreadId_t wifiTaskHandler;

static osSemaphoreId_t wifi_rx_sem;

static osSemaphoreId_t wifi_disabled_sem;

static const osThreadAttr_t wifiTaskAttr = { .name = "wifiTask", .stack_size =
		512 * 4, .priority = (osPriority_t) osPriorityRealtime };

static uart_driver_state_t esp32_uart_driver_state = { 0 };

static WifiState_t wifi_state = WIFI_STATE_RESET;

static char esp32_last_response[RX_LINE_MAX_LEN];

static bool wifi_initialized = false;

static uint8_t current_retry_count = 0;

volatile wifi_health_state_t wifi_health_state = { .last_progress = 0,
		.wait_start = 0, .current_state = WIFI_STATE_RESET };

static bool wifi_handle_reset();
static bool wifi_handle_wait_ready();
static bool wifi_handle_join_ap();
static bool wifi_handle_connected();
static bool wifi_handle_disabled();
static bool wifi_handle_error();

static void wifi_clear_rx_semaphore(void);
static size_t wifi_terminal_line_length(const uint8_t *data, size_t len);
static bool wifi_is_terminal_response(const uint8_t *line, size_t len);
static void wifi_store_last_response(const uint8_t *data, size_t len);
static bool wifi_wait_for_response(const char *expected, uint16_t timeout);

//static bool wifi_send_cmd_and_wait_respond(const char *fmt,
//		const uint16_t timeout, ...);

static bool wifi_cmd_expect(const char *expected, const uint16_t timeout,
bool show_cmd, const char *fmt, ...);

static void uart_rx_callback_handler(const uint8_t *data, size_t len);

static void uart_rx_line_callback_handler(const uint8_t *data, size_t len);

static bool wifi_send_raw_expect(const char *expected, const uint16_t timeout,
		bool show, const char *data, size_t len);

static bool wifi_send_all_can();

void WIFI_Task_Init(UART_HandleTypeDef *esp32_uart_interface) {
	wifi_initialized = false;

	esp32_uart_driver_state.huart = esp32_uart_interface;
	esp32_uart_driver_state.rx_line_callback = uart_rx_line_callback_handler;
	esp32_uart_driver_state.rx_callback = uart_rx_callback_handler;

	wifi_rx_sem = osSemaphoreNew(1, 0, NULL);
	if (wifi_rx_sem == NULL) {
		wifi_log("Failed to create RX semaphore\r\n");
		return;
	}

	wifi_disabled_sem = osSemaphoreNew(1, 0, NULL);
	if (wifi_disabled_sem == NULL) {
		wifi_log("Failed to create disabled semaphore\r\n");
		return;
	}

	wifiTaskHandler = osThreadNew(WIFI_Task, NULL, &wifiTaskAttr);
	if (wifiTaskHandler == NULL) {
		wifi_log("Failed to create WiFi task\r\n");
		return;
	}

	wifi_initialized = true;
}

void WIFI_esp32_uart_rx_callback(size_t len) {
	if (esp32_uart_driver_state.initialized) {
		on_uart_rx_callback(&esp32_uart_driver_state, len);
	}
}

void WIFI_esp32_uart_tx_callback() {
	if (esp32_uart_driver_state.initialized) {
		on_uart_tx_callback(&esp32_uart_driver_state);
	}
}

void WIFI_esp32_uart_error_callback() {
	if (esp32_uart_driver_state.initialized) {
		on_uart_err_callback(&esp32_uart_driver_state);
	}
}

bool WIFI_transmit_data(const char *data, size_t bytes, const uint16_t timeout) {
	if (!wifi_initialized || wifi_state != WIFI_STATE_CONNECTED) {
		return false;
	}

	if (bytes == 0 || bytes > WIFI_MAX_DATA_LEN) {
		wifi_log("Payload length exceeds maximum\r\n");
		return false;
	}

	if (!wifi_cmd_expect(ESP_OK, 200, true, ESP_CMD_SEND_BYTES, bytes))
		return false;

	if (!wifi_send_raw_expect(ESP_SEND_OK, timeout, false, data, bytes))
		return false;

	return true;
}

static void WIFI_Task(void *argument) {
	HAL_GPIO_WritePin(WIFI_HEALTH_LED_PORT, WIFI_HEALTH_LED_PIN, GPIO_PIN_SET);

	HAL_GPIO_WritePin(WIFI_ERR_LED_PORT, WIFI_ERR_LED_PIN, GPIO_PIN_SET);

	osDelay(1000);

	HAL_GPIO_WritePin(WIFI_HEALTH_LED_PORT, WIFI_HEALTH_LED_PIN, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(WIFI_ERR_LED_PORT, WIFI_ERR_LED_PIN, GPIO_PIN_RESET);

	wifi_log("[ESP32] Initializing esp32 uart driver...\r\n");

	esp32_uart_driver_state.controller_rx_sem = wifi_rx_sem;

	UART_Task_Init(&esp32_uart_driver_state);

	wifi_log("[ESP32] Initialized esp32 uart driver\r\n");

	wifi_health_state.last_progress = osKernelGetTickCount();

	while (1) {
		wifi_health_state.wait_start = osKernelGetTickCount();

		switch (wifi_state) {
		case WIFI_STATE_RESET:
			if (wifi_handle_reset()) {
				wifi_state = WIFI_STATE_WAIT_READY;
			} else {
				wifi_state = WIFI_STATE_ERROR;
			}
			break;
		case WIFI_STATE_WAIT_READY:
			// Wait until the Wifi module is ready to transmit
			if (wifi_handle_wait_ready()) {
				wifi_state = WIFI_STATE_JOIN_AP;
			} else {
				wifi_state = WIFI_STATE_ERROR;
			}
			break;
		case WIFI_STATE_JOIN_AP:
			if (wifi_handle_join_ap()) {
				current_retry_count = 0;

				wifi_state = WIFI_STATE_CONNECTED;
			} else {
				wifi_state = WIFI_STATE_ERROR;
			}
			break;
		case WIFI_STATE_CONNECTED:
			if (!wifi_handle_connected()) {
				wifi_state = WIFI_STATE_ERROR;
			}
			break;
		case WIFI_STATE_DISABLED:
			wifi_handle_disabled();

			break;
		case WIFI_STATE_ERROR:
			wifi_handle_error();

			if (current_retry_count < WIFI_NUM_RETRIES) {
				// Attempt resetting the Wifi module
				wifi_state = WIFI_STATE_RESET;

				current_retry_count++;
			} else {
				wifi_log(
						"Number of retries reached. Disable Wifi module\r\n");

				wifi_state = WIFI_STATE_DISABLED;
			}
			break;
		}

		// Update wifi health info
		wifi_health_state.last_progress = osKernelGetTickCount();
		wifi_health_state.current_state = wifi_state;

		osDelay(100);
	}
}

static void uart_rx_line_callback_handler(const uint8_t *data, size_t len) {
	if (wifi_is_terminal_response(data, len)) {
		wifi_store_last_response(data, len);
		osSemaphoreRelease(wifi_rx_sem);
	}
}

static void uart_rx_callback_handler(const uint8_t *data, size_t len) {
	// ESP32 responds
	if (wifi_state == WIFI_STATE_DISABLED) {
		wifi_log(
				"Received response from ESP32, re-enable ESP32\r\n");

		// Re-enable ESP32 com if we receives something from esp32
		osSemaphoreRelease(wifi_disabled_sem);
	}
}

static bool wifi_handle_reset() {
	wifi_log("Resetting, wait 3 seconds...\r\n");

	osDelay(3000);

	// Reset ESP32
	wifi_log("Resetting ESP32...\r\n");

	if (!wifi_cmd_expect(ESP_OK, 1000, true, ESP_CMD_RESET))
		return false;

	osDelay(3000);

	wifi_log("Reset ESP32 firmware...\r\n");

	return true;
}

static bool wifi_handle_wait_ready() {
	wifi_log("Wait for ESP32 to be ready...\r\n");

	if (!wifi_cmd_expect(ESP_OK, 1000, true, ESP_CMD_HEALTH))
		return false;

	return true;
}

static bool wifi_handle_join_ap() {
	wifi_log("Turning off echo\r\n");

	if (!wifi_cmd_expect(ESP_OK, 1000, true, ESP_CMD_ECHO_OFF))
		return false;

	osDelay(200);

	wifi_log("Displaying AT firmware version\r\n");

	// Display AT firmware version
	if (!wifi_cmd_expect(ESP_OK, 1000, true, ESP_CMD_AT_VERSION))
		return false;

	osDelay(200);

	wifi_log("Displaying MAC address\r\n");

	// Display MAC address
	if (!wifi_cmd_expect(ESP_OK, 1000, true, ESP_CMD_MAC_ADDR))
		return false;

	osDelay(200);

	wifi_log("Joining AP...\r\n");

	wifi_log("Setting mode...\r\n");

	if (!wifi_cmd_expect(ESP_OK, 1000, true, ESP_CMD_SET_MODE, 1))
		return false;

	osDelay(200);

	wifi_log("Quitting any existing AP connection...\r\n");

	// Quit any existing AP connection
	if (!wifi_cmd_expect(ESP_OK, 3000, true, ESP_CMD_QUIT_AP))
		return false;

	wifi_log("Configuring AP settings...\r\n");

#if !WIFI_ENABLE_STATIC_IP
	wifi_log("Enabling DHCP for station mode...\r\n");

	// Enable DHCP for station mode
	if (!wifi_cmd_expect(ESP_OK, 1000, true, ESP_CMD_DHCP_MODE, ESP_DHCP_ENABLED, ESP_MODE_STA))
		return false;
#else
	wifi_log("Disabling DHCP for station mode...\r\n");

	// Disable DHCP for station mode
	if (!wifi_cmd_expect(ESP_OK, 1000, true, ESP_CMD_DHCP_MODE, ESP_DHCP_DISABLED, ESP_MODE_STA))
		return false;

	osDelay(200);

	// Set static IP for station mode
	if (!wifi_cmd_expect(ESP_OK, 1000, true, ESP_CMD_SET_STATIC_IP, WIFI_STATIC_IP, WIFI_STATIC_GATEWAY, WIFI_STATIC_NETMASK))
		return false;

	osDelay(200);

	// Verify static IP settings
	if (!wifi_cmd_expect(ESP_OK, 1000, true, ESP_CMD_IP_STATUS))
		return false;
#endif

	osDelay(200);

	wifi_log_fmt("Joining AP with SSID: %s", WIFI_SSID);

	// Join the specified WiFi access point
	if (!wifi_cmd_expect(ESP_OK, 30000, true, ESP_CMD_JOIN_AP, WIFI_SSID,
	WIFI_PASSWORD)) {
		wifi_log("Failed to join AP. Credentials not correct or IP address not given (DHCP enabled?)\r\n");

		return false;
	}

	osDelay(3000);

	wifi_log("Starting UDP connection...\r\n");

	if (!wifi_cmd_expect(ESP_OK, 3000, true, ESP_CMD_UDP_START, HOST_IP,
	HOST_PORT))
		return false;

	osDelay(3000);

	HAL_GPIO_WritePin(WIFI_HEALTH_LED_PORT, WIFI_HEALTH_LED_PIN, GPIO_PIN_SET);

	HAL_GPIO_WritePin(WIFI_ERR_LED_PORT, WIFI_ERR_LED_PIN, GPIO_PIN_RESET);

	wifi_log("Ready to transmit data!\r\n");

	return true;
}

static bool wifi_handle_connected() {
	if (!wifi_send_all_can()) {
		return false;
	}

	return true;
}

static bool wifi_handle_disabled() {
	if (osSemaphoreAcquire(wifi_disabled_sem, osWaitForever) == osOK) {
		wifi_state = WIFI_STATE_RESET;
	} else {
		wifi_log("Failed to acquire disabled semaphore\r\n");

		return false;
	}

	return true;
}

static bool wifi_handle_error() {
	wifi_log("ERROR HAS OCCURED!\r\n");

	HAL_GPIO_WritePin(WIFI_HEALTH_LED_PORT, WIFI_HEALTH_LED_PIN, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(WIFI_ERR_LED_PORT, WIFI_ERR_LED_PIN, GPIO_PIN_SET);

	return true;
}

//static bool wifi_send_cmd_and_wait_respond(const char *fmt,
//		const uint16_t timeout, ...) {
//	/* drain stale signal */
//	while (osSemaphoreAcquire(wifi_rx_sem, 0) == osOK) {
//	}
//
//	char buf[128];
//	char out_msg[512];
//
//	va_list args;
//	va_start(args, timeout);
//	vsnprintf(buf, sizeof(buf), fmt, args);
//	va_end(args);
//
//	wifi_log("Sent CMD\r\n", 0);
//
//	// Send data into the uart interface that connects to esp32
//	bool ret = uart_send_data(&esp32_uart_driver_state, buf);
//
//	// wait for response
//	if (osSemaphoreAcquire(wifi_rx_sem, timeout) == osOK) {
//		sprintf(out_msg, "[Wifi] CMD: %s | UART says: %s\r\n", buf,
//				esp32_uart_driver_state.rx_line_buf);
//
//		wifi_log(out_msg, 256);
//
//		return ret;
//	} else {
//		// Timeout
//		wifi_log("[ESP32] Timeout.\r\n", 0);
//
//		return false;
//	}
//
//	return ret;
//}

static bool wifi_cmd_expect(const char *expected, const uint16_t timeout,
bool show_cmd, const char *fmt, ...) {
	if (fmt == NULL || expected == NULL) {
		wifi_log("Invalid arguments\r\n");
		return false;
	}

	wifi_clear_rx_semaphore();

	char tx_buf[128];
	char log_buf[256];

	va_list args;
	va_start(args, fmt);
	int tx_len = vsnprintf(tx_buf, sizeof(tx_buf), fmt, args);
	va_end(args);

	if (tx_len < 0 || tx_len >= (int) sizeof(tx_buf)) {
		wifi_log("Command formatting failed or truncated\r\n");
		return false;
	}

	if (show_cmd) {
		snprintf(log_buf, sizeof(log_buf), "Sent CMD: %s\r\n", tx_buf);
		wifi_log(log_buf);
	}

	if (!uart_send_data_w_len(&esp32_uart_driver_state, tx_buf, tx_len)) {
		wifi_log("UART send failed\r\n");
		return false;
	}

	return wifi_wait_for_response(expected, timeout);
}

static bool wifi_send_raw_expect(const char *expected, const uint16_t timeout,
		bool show, const char *data, size_t len) {
	if (expected == NULL || data == NULL || len == 0) {
		wifi_log("Invalid raw send arguments\r\n");
		return false;
	}

	wifi_clear_rx_semaphore();

	if (show) {
		char log_buf[256];
		int off = snprintf(log_buf, sizeof(log_buf), "Sent RAW (%u): ",
				(unsigned) len);

		for (size_t i = 0; i < len && off < (int) sizeof(log_buf) - 4;
				i++) {
			off += snprintf(&log_buf[off], sizeof(log_buf) - off, "%02X ",
					data[i]);
		}

		snprintf(&log_buf[off], sizeof(log_buf) - off, "\r\n");
		wifi_log(log_buf);
	}

	if (!uart_send_data_w_len(&esp32_uart_driver_state, (const char*) data,
			len)) {
		wifi_log("UART send failed\r\n");
		return false;
	}

	return wifi_wait_for_response(expected, timeout);
}

static void wifi_clear_rx_semaphore(void) {
	while (osSemaphoreAcquire(wifi_rx_sem, 0) == osOK) {
	}
}

static size_t wifi_terminal_line_length(const uint8_t *data, size_t len) {
	if (data == NULL) {
		return 0;
	}

	size_t line_len = 0;

	while (line_len < len && data[line_len] != '\r' && data[line_len] != '\n') {
		line_len++;
	}

	return line_len;
}

static bool wifi_is_terminal_response(const uint8_t *line, size_t len) {
	size_t line_len = wifi_terminal_line_length(line, len);

	if (line_len == 0) {
		return false;
	}

	return ((line_len == 2 && memcmp(line, "OK", 2) == 0)
			|| (line_len == 5 && memcmp(line, "ERROR", 5) == 0)
			|| (line_len == 4 && memcmp(line, "FAIL", 4) == 0)
			|| (line_len == 7 && memcmp(line, "SEND OK", 7) == 0)
			|| (line_len == 9 && memcmp(line, "SEND FAIL", 9) == 0));
}

static void wifi_store_last_response(const uint8_t *data, size_t len) {
	size_t copy_len = wifi_terminal_line_length(data, len);

	if (copy_len >= sizeof(esp32_last_response)) {
		copy_len = sizeof(esp32_last_response) - 1;
	}

	memcpy(esp32_last_response, data, copy_len);
	esp32_last_response[copy_len] = '\0';
}

static bool wifi_wait_for_response(const char *expected, uint16_t timeout) {
	if (osSemaphoreAcquire(wifi_rx_sem, timeout) != osOK) {
		wifi_log("Timeout waiting for response\r\n");
		return false;
	}

	const char *rx = (const char*) esp32_last_response;
	char log_buf[256];

	snprintf(log_buf, sizeof(log_buf), "UART says: %s -- \r\n", rx);
	wifi_log(log_buf);

	size_t rx_len = strlen(rx);
	size_t expected_len = strlen(expected);

	return (rx_len == expected_len) && (memcmp(rx, expected, expected_len) == 0);
}

static bool wifi_send_all_can() {
	can_storage_t *can_storage = get_can_storage();
	uint8_t buf[sizeof(wifi_payload_t)];

	// Acquire the mutex to ensure exclusive access to the CAN storage
	if (osMutexAcquire(can_storage->mutex, osWaitForever) != osOK) {
		wifi_log("Failed to acquire CAN storage mutex\r\n");
		return false;
	}

	for (uint16_t i = 0; i < TABLE_SIZE; i++) {
		can_msg_node_t *node = &can_storage->nodes[i];

		if (node->valid) {
			// Prepare the payload
			buf[0] = 0; // Wifi is 0

			buf[4] = (node->key >> 24) & 0xFF;
			buf[3] = (node->key >> 16) & 0xFF;
			buf[2] = (node->key >> 8) & 0xFF;
			buf[1] = node->key & 0xFF;

			buf[5] = node->value.len;

			// Clear the rest of the buffer to ensure no leftover data is sent
			memset(&buf[6], 0, sizeof(buf) - 6);

			// Copy the payload data into the buffer
			memcpy(&buf[6], node->value.payload, node->value.len);

			// Send the payload over WiFi
			if (!WIFI_transmit_data((const char*) buf, node->value.len + 6, 3000)) {
				wifi_log("Failed to send CAN message over WiFi. Ignore...\r\n");
				osMutexRelease(can_storage->mutex);

				return true;
			}

			// Mark the node as invalid after sending
			node->valid = false;
		}
	}

	osMutexRelease(can_storage->mutex);

	return true;
}
