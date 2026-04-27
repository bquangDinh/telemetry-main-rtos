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

typedef struct {
	uint8_t message_type;
	uint32_t id;
	uint8_t len;
	uint8_t payload[WIFI_MAX_DATA_LEN];
} wifi_payload_t;

typedef struct {
	wifi_payload_t buffer[WIFI_PAYLOAD_QUEUE_MAX_CAPACITY];
	wifi_payload_t pending_payload;
	bool payload_processing;
	uint8_t retries_count;
	uint16_t tail;
	uint16_t head;
	uint16_t count;
	osMutexId_t mutex;
	osSemaphoreId_t items;
} wifi_queue_t;

#define WIFI_HEALTH_LED_PIN GPIO_PIN_1
#define WIFI_HEALTH_LED_PORT GPIOD

#define WIFI_ERR_LED_PIN GPIO_PIN_3
#define WIFI_ERR_LED_PORT GPIOD

/* List of Commands */
#define ESP_CMD_HEALTH       "AT\r\n"
#define ESP_CMD_ECHO_OFF     "ATE0\r\n"

#define ESP_CMD_SET_MODE     "AT+CWMODE=%d\r\n"
#define ESP_CMD_JOIN_AP      "AT+CWJAP=\"%s\",\"%s\"\r\n"
#define ESP_CMD_UDP_START    "AT+CIPSTART=\"UDP\",\"%s\",%d\r\n"
#define ESP_CMD_SEND_BYTES   "AT+CIPSEND=%d\r\n"
#define ESP_CMD_AT_VERSION	 "AT+GMR\r\n"
#define ESP_CMD_RESET		 "AT+RST\r\n"

#define ESP_READY_STR "ready"
#define ESP_OK "OK"
#define ESP_ERR "ERROR"
#define ESP_SEND_OK "SEND OK"

static void WIFI_Task(void *argument);

static osThreadId_t wifiTaskHandler;

static osSemaphoreId_t wifi_rx_sem;

static osSemaphoreId_t wifi_disabled_sem;

static const osThreadAttr_t wifiTaskAttr = { .name = "wifiTask", .stack_size =
		512 * 4, .priority = (osPriority_t) osPriorityRealtime };

static uart_driver_state_t esp32_uart_driver_state = { 0 };

static WifiState_t wifi_state = WIFI_STATE_RESET;

static wifi_queue_t wifi_payload_queue;

static char esp32_last_response[RX_LINE_MAX_LEN];

static uint8_t current_retry_count = 0;

volatile wifi_health_state_t wifi_health_state = { .last_progress = 0,
		.wait_start = 0, .current_state = WIFI_STATE_RESET };

static bool wifi_handle_reset();
static bool wifi_handle_wait_ready();
static bool wifi_handle_join_ap();
static bool wifi_handle_connected();
static bool wifi_handle_disabled();
static bool wifi_handle_error();
static bool wifi_process_pending_payload(void);
static void wifi_clear_rx_semaphore(void);
static size_t wifi_terminal_line_length(const uint8_t *data, size_t len);
static bool wifi_is_terminal_response(const uint8_t *line, size_t len);
static void wifi_store_last_response(const uint8_t *data, size_t len);
static bool wifi_wait_for_response(const char *expected, uint16_t timeout);

//static bool wifi_send_cmd_and_wait_respond(const char *fmt,
//		const uint16_t timeout, ...);

static bool wifi_cmd_expect(const char *expected, const uint16_t timeout,
bool show_cmd, const char *fmt, ...);

static bool wifi_payload_pop(wifi_payload_t *out);

static void uart_rx_callback_handler(const uint8_t *data, size_t len);

static void uart_rx_line_callback_handler(const uint8_t *data, size_t len);

static void reset_queue();

static bool wifi_send_raw_expect(const char *expected, const uint16_t timeout,
		bool show, const char *data, size_t len);

void WIFI_Task_Init(UART_HandleTypeDef *esp32_uart_interface) {
	esp32_uart_driver_state.huart = esp32_uart_interface;
	esp32_uart_driver_state.rx_line_callback = uart_rx_line_callback_handler;
	esp32_uart_driver_state.rx_callback = uart_rx_callback_handler;

	wifi_payload_queue.mutex = osMutexNew(NULL);
	wifi_payload_queue.items = osSemaphoreNew(WIFI_PAYLOAD_QUEUE_MAX_CAPACITY,
			0, NULL);

	wifi_rx_sem = osSemaphoreNew(1, 0, NULL);
	wifi_disabled_sem = osSemaphoreNew(1, 0, NULL);

	wifiTaskHandler = osThreadNew(WIFI_Task, NULL, &wifiTaskAttr);
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

bool WIFI_add_payload_to_queue(const wifi_message_type_t message_type,
		const uint32_t id, const uint8_t *data, uint16_t len) {
	if (data == NULL || len == 0)
		return false;

	// Lock to make sure no other process trying to read the queue while it's adding item to the queue
	osMutexAcquire(wifi_payload_queue.mutex, osWaitForever);

	if (wifi_payload_queue.count >= WIFI_PAYLOAD_QUEUE_MAX_CAPACITY) {
		osMutexRelease(wifi_payload_queue.mutex);

		return false;
	}

	wifi_payload_t *slot = &wifi_payload_queue.buffer[wifi_payload_queue.head];

	if (len > sizeof(slot->payload)) {
		len = sizeof(slot->payload);
	}

	slot->id = id;

	slot->len = len;

	slot->message_type = (uint8_t) message_type;

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

	if (!wifi_cmd_expect(ESP_OK, 200, true, ESP_CMD_SEND_BYTES, bytes))
		return false;

	if (!wifi_send_raw_expect(ESP_SEND_OK, timeout, false, data, bytes))
		return false;

	return true;
}

static void WIFI_Task(void *argument) {
	uart_logger_add_msg("[ESP32] Initializing esp32 uart driver...\r\n", 0);

	esp32_uart_driver_state.controller_rx_sem = wifi_rx_sem;

	UART_Task_Init(&esp32_uart_driver_state);

	uart_logger_add_msg("[ESP32] Initialized esp32 uart driver\r\n", 0);

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
				uart_logger_add_msg(
						"[Wifi] Number of retries reached. Disable Wifi module\r\n",
						0);

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
		uart_logger_add_msg(
				"[Wifi] Received response from ESP32, re-enable ESP32\r\n", 0);

		// Re-enable ESP32 com if we receives something from esp32
		osSemaphoreRelease(wifi_disabled_sem);
	}
}

static bool wifi_handle_reset() {
	uart_logger_add_msg("[Wifi] Resetting, wait 3 seconds...\r\n", 0);

	reset_queue();

	osDelay(3000);

	// Reset ESP32
	uart_logger_add_msg("[ESP32] Resetting ESP32...\r\n", 0);

	if (!wifi_cmd_expect(ESP_OK, 1000, true, ESP_CMD_RESET))
		return false;

	osDelay(3000);

	uart_logger_add_msg("[ESP32] Reset ESP32 firmware...\r\n", 0);

	return true;
}

static bool wifi_handle_wait_ready() {
	uart_logger_add_msg("[ESP32] Wait for ESP32 to be ready...\r\n", 0);

	if (!wifi_cmd_expect(ESP_OK, 1000, true, ESP_CMD_HEALTH))
		return false;

	return true;
}

static bool wifi_handle_join_ap() {
	uart_logger_add_msg("[ESP32] Turning off echo\r\n", 0);

	if (!wifi_cmd_expect(ESP_OK, 1000, true, ESP_CMD_ECHO_OFF))
		return false;

	osDelay(200);

	// Display AT firmware version
	if (!wifi_cmd_expect(ESP_OK, 1000, true, ESP_CMD_AT_VERSION))
		return false;

	osDelay(200);

	if (!wifi_cmd_expect(ESP_OK, 1000, true, ESP_CMD_SET_MODE, 1))
		return false;

	osDelay(200);

	if (!wifi_cmd_expect(ESP_OK, 3000, false, ESP_CMD_JOIN_AP, WIFI_SSID,
	WIFI_PASSWORD))
		return false;

	osDelay(3000);

	if (!wifi_cmd_expect(ESP_OK, 3000, true, ESP_CMD_UDP_START, HOST_IP,
	HOST_PORT))
		return false;

	osDelay(3000);

	uart_logger_add_msg("[Wifi] Ready to transmit data\r\n", 0);

	return true;
}

static bool wifi_handle_connected() {
	// Ping the Wifi module to check if it's still ON
	if (!wifi_cmd_expect(ESP_OK, 1000, true, ESP_CMD_HEALTH))
		return false;

	if (!wifi_process_pending_payload()) {
		return false;
	}

	HAL_GPIO_WritePin(WIFI_HEALTH_LED_PORT, WIFI_HEALTH_LED_PIN, GPIO_PIN_SET);

	HAL_GPIO_WritePin(WIFI_ERR_LED_PORT, WIFI_ERR_LED_PIN, GPIO_PIN_RESET);

	return true;
}

static bool wifi_handle_disabled() {
	if (osSemaphoreAcquire(wifi_disabled_sem, 1000) == osOK) {
		wifi_state = WIFI_STATE_RESET;

		current_retry_count = 0;
	}

	return true;
}

static bool wifi_handle_error() {
	uart_logger_add_msg("[Wifi] ERROR HAS OCCURED!\r\n", 0);

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
//	uart_logger_add_msg("[Wifi] Sent CMD\r\n", 0);
//
//	// Send data into the uart interface that connects to esp32
//	bool ret = uart_send_data(&esp32_uart_driver_state, buf);
//
//	// wait for response
//	if (osSemaphoreAcquire(wifi_rx_sem, timeout) == osOK) {
//		sprintf(out_msg, "[Wifi] CMD: %s | UART says: %s\r\n", buf,
//				esp32_uart_driver_state.rx_line_buf);
//
//		uart_logger_add_msg(out_msg, 256);
//
//		return ret;
//	} else {
//		// Timeout
//		uart_logger_add_msg("[ESP32] Timeout.\r\n", 0);
//
//		return false;
//	}
//
//	return ret;
//}

static bool wifi_cmd_expect(const char *expected, const uint16_t timeout,
bool show_cmd, const char *fmt, ...) {
	if (fmt == NULL || expected == NULL) {
		uart_logger_add_msg("[Wifi] Invalid arguments\r\n", 0);
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
		uart_logger_add_msg("[Wifi] Command formatting failed or truncated\r\n",
				0);
		return false;
	}

	if (show_cmd) {
		snprintf(log_buf, sizeof(log_buf), "[Wifi] Sent CMD: %s\r\n", tx_buf);
		uart_logger_add_msg(log_buf, 0);
	}

	if (!uart_send_data_w_len(&esp32_uart_driver_state, tx_buf, tx_len)) {
		uart_logger_add_msg("[Wifi] UART send failed\r\n", 0);
		return false;
	}

	return wifi_wait_for_response(expected, timeout);
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

static void reset_queue() {
	osMutexAcquire(wifi_payload_queue.mutex, osWaitForever);

	// Reset the queue
	wifi_payload_queue.head = 0;
	wifi_payload_queue.tail = 0;
	wifi_payload_queue.retries_count = 0;
	wifi_payload_queue.payload_processing = false;

	// drain semaphore to 0
	while (osSemaphoreAcquire(wifi_payload_queue.items, 0) == osOK) {
	}

	osMutexRelease(wifi_payload_queue.mutex);
}

static bool wifi_send_raw_expect(const char *expected, const uint16_t timeout,
		bool show, const char *data, size_t len) {
	if (expected == NULL || data == NULL || len == 0) {
		uart_logger_add_msg("[Wifi] Invalid raw send arguments\r\n", 0);
		return false;
	}

	wifi_clear_rx_semaphore();

	if (show) {
		char log_buf[256];
		int off = snprintf(log_buf, sizeof(log_buf), "[Wifi] Sent RAW (%u): ",
				(unsigned) len);

		for (size_t i = 0; i < len && off < (int) sizeof(log_buf) - 4;
				i++) {
			off += snprintf(&log_buf[off], sizeof(log_buf) - off, "%02X ",
					data[i]);
		}

		snprintf(&log_buf[off], sizeof(log_buf) - off, "\r\n");
		uart_logger_add_msg(log_buf, 0);
	}

	if (!uart_send_data_w_len(&esp32_uart_driver_state, (const char*) data,
			len)) {
		uart_logger_add_msg("[Wifi] UART send failed\r\n", 0);
		return false;
	}

	return wifi_wait_for_response(expected, timeout);
}

static bool wifi_process_pending_payload(void) {
	bool payload_available = false;

	if (!wifi_payload_queue.payload_processing) {
		if (osSemaphoreAcquire(wifi_payload_queue.items, 1000) == osOK) {
			payload_available = wifi_payload_pop(
					&wifi_payload_queue.pending_payload);
		}
	}

	if (!(payload_available || wifi_payload_queue.payload_processing)) {
		return true;
	}

	if (wifi_payload_queue.retries_count >= WIFI_PAYLOAD_TRANSMIT_RETRIES) {
		wifi_payload_queue.payload_processing = false;
		uart_logger_add_msg("[WIFI] Failed to send payload. Max retries reached\r\n", 0);
		return false;
	}

	if (WIFI_transmit_data((const char*) &wifi_payload_queue.pending_payload,
			sizeof(wifi_payload_queue.pending_payload), 3000)) {
		uart_logger_add_msg("[WIFI] Send data ok\r\n", 0);
		wifi_payload_queue.payload_processing = false;
		wifi_payload_queue.retries_count = 0;
	} else {
		uart_logger_add_msg("[WIFI] Failed to send. Try again...\r\n", 0);
		wifi_payload_queue.retries_count++;
		wifi_payload_queue.payload_processing = true;
	}

	return true;
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
		uart_logger_add_msg("[Wifi] Timeout waiting for response\r\n", 0);
		return false;
	}

	const char *rx = (const char*) esp32_last_response;
	char log_buf[256];

	snprintf(log_buf, sizeof(log_buf), "[Wifi] UART says: %s -- \r\n", rx);
	uart_logger_add_msg(log_buf, 0);

	size_t rx_len = strlen(rx);
	size_t expected_len = strlen(expected);

	return (rx_len == expected_len) && (memcmp(rx, expected, expected_len) == 0);
}
