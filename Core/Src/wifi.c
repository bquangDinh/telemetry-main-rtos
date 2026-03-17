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
#include "utilities.h"
#include "cmsis_os.h"

typedef enum {
	WIFI_STATE_RESET,
	WIFI_STATE_WAIT_READY,
	WIFI_STATE_JOIN_AP,
	WIFI_STATE_CONNECTED,
	WIFI_STATE_ERROR,
} WifiState_t;

typedef struct {
	uint8_t payload[WIFI_MAX_DATA_LEN];
	uint8_t len;
} wifi_payload_t;

typedef struct {
	wifi_payload_t buffer[WIFI_PAYLOAD_QUEUE_MAX_CAPACITY];
	uint16_t tail;
	uint16_t head;
	uint16_t count;
	osMutexId_t mutex;
	osSemaphoreId_t items;
} wifi_queue_t;

/* List of Commands */
#define ESP_CMD_HEALTH       "AT\r\n"
#define ESP_CMD_ECHO_OFF     "ATE0\r\n"

#define ESP_CMD_SET_MODE     "AT+CWMODE=%d\r\n"
#define ESP_CMD_JOIN_AP      "AT+CWJAP=\"%s\",\"%s\"\r\n"
#define ESP_CMD_UDP_START    "AT+CIPSTART=\"UDP\",\"%s\",%d\r\n"
#define ESP_CMD_SEND_BYTES   "AT+CIPSEND=%d\r\n"
#define ESP_CMD_AT_VERSION	 "AT+GMR\r\n"

#define ESP_READY_STR "ready"
#define ESP_OK "OK"
#define ESP_ERR "ERROR"

static void WIFI_Task(void *argument);

static osThreadId_t wifiTaskHandler;

static osSemaphoreId_t wifi_rx_sem;

static const osThreadAttr_t wifiTaskAttr = { .name = "wifiTask", .stack_size =
		512 * 4, .priority = (osPriority_t) osPriorityRealtime };

static uart_driver_state_t esp32_uart_driver_state = { 0 };

static WifiState_t wifi_state = WIFI_STATE_RESET;

static wifi_queue_t wifi_payload_queue;

static char esp32_last_response[RX_LINE_MAX_LEN];

static bool wifi_handle_reset();
static bool wifi_handle_wait_ready();
static bool wifi_handle_join_ap();
static bool wifi_handle_connected();
static bool wifi_handle_error();

static bool wifi_send_cmd_and_wait_respond(const char *fmt,
		const uint16_t timeout, ...);

static bool wifi_cmd_expect(const char *expected, const uint16_t timeout,
		const char *fmt, ...);

static bool wifi_payload_pop(wifi_payload_t *out);

static void uart_rx_line_callback_handler(const uint8_t *data, size_t len);

static bool wifi_is_terminal_response(const char *line);

void WIFI_Task_Init(UART_HandleTypeDef *esp32_uart_interface) {
	esp32_uart_driver_state.huart = esp32_uart_interface;
	esp32_uart_driver_state.rx_line_callback = uart_rx_line_callback_handler;

	wifi_payload_queue.mutex = osMutexNew(NULL);
	wifi_payload_queue.items = osSemaphoreNew(WIFI_PAYLOAD_QUEUE_MAX_CAPACITY,
			0, NULL);

	wifi_rx_sem = osSemaphoreNew(1, 0, NULL);

	wifiTaskHandler = osThreadNew(WIFI_Task, NULL, &wifiTaskAttr);
}

void WIFI_esp32_uart_rx_callback(size_t len) {
	if (esp32_uart_driver_state.initialized) {
		on_uart_rx_callback(&esp32_uart_driver_state, len);
	}
}

bool WIFI_add_payload_to_queue(const uint8_t *data, uint16_t len) {
	if (data == NULL || len == 0)
		return false;

	osMutexAcquire(wifi_payload_queue.mutex, osWaitForever);

	if (wifi_payload_queue.count >= WIFI_PAYLOAD_QUEUE_MAX_CAPACITY) {
		osMutexRelease(wifi_payload_queue.mutex);

		return false;
	}

	wifi_payload_t *slot = &wifi_payload_queue.buffer[wifi_payload_queue.head];

	if (len >= WIFI_MAX_DATA_LEN) {
		len = WIFI_MAX_DATA_LEN - 1;
	}

	slot->len = len;

	memset(slot->payload, 0, WIFI_MAX_DATA_LEN);

	memcpy(slot->payload, data, len);

	wifi_payload_queue.head = (wifi_payload_queue.head + 1)
			% WIFI_PAYLOAD_QUEUE_MAX_CAPACITY;

	wifi_payload_queue.count++;

	osMutexRelease(wifi_payload_queue.mutex);

	// Signal that there is item to consume
	osSemaphoreRelease(wifi_payload_queue.items);

	return true;
}

bool WIFI_transmit_data(const char *data, size_t bytes, const uint16_t timeout) {
	if (wifi_state != WIFI_STATE_CONNECTED) {
		return false;
	}

	if (!wifi_send_cmd_and_wait_respond(ESP_CMD_SEND_BYTES, 200, bytes))
		return false;

	if (!wifi_send_cmd_and_wait_respond(data, timeout))
		return false;

	return true;
}

bool WIIF_transmit_data_format(const char *fmt, const uint16_t timeout, ...) {
	char buf[256];

	va_list args;
	va_start(args, timeout);
	vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);

	return WIFI_transmit_data(buf, 256, timeout);
}

static void WIFI_Task(void *argument) {
	uart_logger_add_msg("[ESP32] Initializing esp32 uart driver...\r\n", 0);

	esp32_uart_driver_state.controller_rx_sem = wifi_rx_sem;

	UART_Task_Init(&esp32_uart_driver_state);

	uart_logger_add_msg("[ESP32] Initialized esp32 uart driver\r\n", 0);

	while (1) {
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
		case WIFI_STATE_ERROR:
			wifi_handle_error();

			// Attempt resetting the Wifi module
			wifi_state = WIFI_STATE_RESET;
			break;
		}

		osDelay(500);
	}
}

static void uart_rx_line_callback_handler(const uint8_t *data, size_t len) {
	// Only release if it receives valid response from ESP32
	/* Ignore empty lines */
	size_t no_newline_str_len = strcspn((char*) data, "\r\n");

	if (no_newline_str_len > 0) {
		if (wifi_is_terminal_response((char*) data)) {
			if (no_newline_str_len >= sizeof(esp32_last_response))
				no_newline_str_len = sizeof(esp32_last_response) - 1;

			memcpy(esp32_last_response, data, no_newline_str_len);

			esp32_last_response[no_newline_str_len] = '\0';

			osSemaphoreRelease(wifi_rx_sem);
		}
	}
}

static bool wifi_handle_reset() {
	uart_logger_add_msg("[Wifi] Resetting, wait 1 second...\r\n", 0);

	osDelay(1000);

	return true;
}

static bool wifi_handle_wait_ready() {
	uart_logger_add_msg("[ESP32] Wait for ESP32 to be ready...\r\n", 0);

	if (!wifi_cmd_expect(ESP_OK, 1000, ESP_CMD_HEALTH))
			return false;

	return false;
}

static bool wifi_handle_join_ap() {
	uart_logger_add_msg("[ESP32] Turning off echo\r\n", 0);

	if (!wifi_cmd_expect(ESP_OK, 1000, ESP_CMD_ECHO_OFF))
		return false;

	osDelay(200);

	// Display AT firmware version
	if (!wifi_cmd_expect(ESP_OK, 1000, ESP_CMD_AT_VERSION))
		return false;

	osDelay(200);

	if (!wifi_cmd_expect(ESP_OK, 1000, ESP_CMD_SET_MODE, 1))
		return false;

	osDelay(200);

	if (!wifi_cmd_expect(ESP_OK, 3000, ESP_CMD_JOIN_AP, WIFI_SSID,
	WIFI_PASSWORD))
		return false;

	osDelay(3000);

	if (!wifi_cmd_expect(ESP_OK, 3000, ESP_CMD_UDP_START, HOST_IP,
	HOST_PORT))
		return false;

	osDelay(3000);

	uart_logger_add_msg("[Wifi] Ready to transmit data\r\n", 0);

	return true;
}

static bool wifi_handle_connected() {
	wifi_payload_t payload;

	// Wait for items
	osSemaphoreAcquire(wifi_payload_queue.items, osWaitForever);

	if (wifi_payload_pop(&payload)) {
		uart_logger_add_msg("[Wifi] Data is available!\r\n", 0);

		bool ret = WIFI_transmit_data((char*) payload.payload, payload.len,
				3000);

		osDelay(3000);

		return ret;
	}

	return true;
}

static bool wifi_handle_error() {
	uart_logger_add_msg("[Wifi] ERROR HAS OCCURED!\r\n", 0);

	return true;
}

static bool wifi_send_cmd_and_wait_respond(const char *fmt,
		const uint16_t timeout, ...) {
	/* drain stale signal */
	while (osSemaphoreAcquire(wifi_rx_sem, 0) == osOK) {
	}

	char buf[128];
	char out_msg[512];

	va_list args;
	va_start(args, timeout);
	vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);

	uart_logger_add_msg("[Wifi] Sent CMD\r\n", 0);

	// Send data into the uart interface that connects to esp32
	bool ret = uart_send_data(&esp32_uart_driver_state, buf);

	// wait for response
	if (osSemaphoreAcquire(wifi_rx_sem, timeout) == osOK) {
		sprintf(out_msg, "[Wifi] CMD: %s | UART says: %s\r\n", buf,
				esp32_uart_driver_state.rx_line_buf);

		uart_logger_add_msg(out_msg, 256);

		return ret;
	} else {
		// Timeout
		uart_logger_add_msg("[ESP32] Timeout.\r\n", 0);

		return false;
	}

	return ret;
}

static bool wifi_cmd_expect(const char *expected, const uint16_t timeout,
		const char *fmt, ...) {
	if (fmt == NULL || expected == NULL) {
		uart_logger_add_msg("[Wifi] Invalid arguments\r\n", 0);
		return false;
	}

	while (osSemaphoreAcquire(wifi_rx_sem, 0) == osOK) {
	}

	char tx_buf[128];
	char log_buf[256];

	va_list args;
	va_start(args, fmt);
	int tx_len = vsnprintf(tx_buf, sizeof(tx_buf), fmt, args);
	va_end(args);

	if (tx_len < 0 || tx_len >= (int) sizeof(tx_buf)) {
		uart_logger_add_msg("[Wifi] Command formatting failed or truncated\r\n",
				0);
		return false;
	}

	snprintf(log_buf, sizeof(log_buf), "[Wifi] Sent CMD: %s\r\n", tx_buf);
	uart_logger_add_msg(log_buf, 0);

	if (!uart_send_data(&esp32_uart_driver_state, tx_buf)) {
		uart_logger_add_msg("[Wifi] UART send failed\r\n", 0);
		return false;
	}

	if (osSemaphoreAcquire(wifi_rx_sem, timeout) != osOK) {
		uart_logger_add_msg("[Wifi] Timeout waiting for response\r\n", 0);
		return false;
	}

	const char *rx = (const char*) esp32_last_response;

	if (rx == NULL) {
		uart_logger_add_msg("[Wifi] last response buffer is NULL\r\n", 0);
		return false;
	}

	snprintf(log_buf, sizeof(log_buf), "[Wifi] UART says: %s -- \r\n", rx);
	uart_logger_add_msg(log_buf, 0);

	size_t rx_len = strcspn(rx, "\r\n");
	size_t expected_len = strlen(expected);

	return (rx_len == expected_len) && (memcmp(rx, expected, expected_len) == 0);
}

static bool wifi_payload_pop(wifi_payload_t *out) {
	if (out == NULL)
		return false;

	osMutexAcquire(wifi_payload_queue.mutex, osWaitForever);

	if (wifi_payload_queue.count == 0) {
		osMutexRelease(wifi_payload_queue.mutex);

		return false;
	}

	*out = wifi_payload_queue.buffer[wifi_payload_queue.tail];
	wifi_payload_queue.tail = (wifi_payload_queue.tail + 1)
			% WIFI_PAYLOAD_QUEUE_MAX_CAPACITY;
	wifi_payload_queue.count--;

	osMutexRelease(wifi_payload_queue.mutex);

	return true;
}

static bool wifi_is_terminal_response(const char *line) {
	if (line == NULL) {
		return false;
	}

	size_t len = strcspn(line, "\r\n");

	if (len == 0) {
		return false;
	}

	return ((len == 2 && memcmp(line, "OK", 2) == 0)
			|| (len == 5 && memcmp(line, "ERROR", 5) == 0)
			|| (len == 4 && memcmp(line, "FAIL", 4) == 0)
			|| (len == 7 && memcmp(line, "SEND OK", 7) == 0)
			|| (len == 9 && memcmp(line, "SEND FAIL", 9) == 0));
}
