/*
 * wifi.c
 *
 *  Created on: Mar 11, 2026
 *      Author: buiqu
 */

#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>

#include "wifi.h"
#include "uart_logger.h"
#include "uart_driver.h"
#include "cmsis_os.h"

typedef enum {
	WIFI_STATE_RESET,
	WIFI_STATE_WAIT_READY,
	WIFI_STATE_JOIN_AP,
	WIFI_STATE_CONNECTED,
	WIFI_STATE_ERROR,
} WifiState_t;

/* List of Commands */
#define ESP_CMD_HEALTH       "AT\r\n"
#define ESP_CMD_ECHO_OFF     "ATE0\r\n"

#define ESP_CMD_SET_MODE     "AT+CWMODE=%d\r\n"
#define ESP_CMD_JOIN_AP      "AT+CWJAP=\"%s\",\"%s\"\r\n"
#define ESP_CMD_UDP_START    "AT+CIPSTART=\"UDP\",\"%s\",%d\r\n"
#define ESP_CMD_SEND_BYTES   "AT+CIPSEND=%d\r\n"

static void WIFI_Task(void *argument);

static osThreadId_t wifiTaskHandler;

static osSemaphoreId_t wifi_rx_sem;

static const osThreadAttr_t wifiTaskAttr = { .name = "wifiTask", .stack_size =
		256 * 4, .priority = (osPriority_t) osPriorityRealtime };

static uart_driver_state_t esp32_uart_driver_state = { 0 };

static WifiState_t wifi_state = WIFI_STATE_RESET;

static bool wifi_handle_reset();
static bool wifi_handle_wait_ready();
static bool wifi_handle_join_ap();
static bool wifi_handle_connected();
static bool wifi_handle_error();

static bool wifi_send_cmd_and_wait_respond(const char *fmt, const uint16_t timeout, ...);

static bool wifi_transmit_data(const void* data, size_t bytes);

void WIFI_Task_Init(UART_HandleTypeDef *esp32_uart_interface) {
	esp32_uart_driver_state.huart = esp32_uart_interface;

	wifiTaskHandler = osThreadNew(WIFI_Task, NULL, &wifiTaskAttr);
}

void WIFI_esp32_uart_rx_callback(size_t len) {
	if (esp32_uart_driver_state.initialized) {
		on_uart_rx_callback(&esp32_uart_driver_state, len);
	}
}

bool wifi_transmit_data(const void* data, size_t bytes) {
	if (wifi_state != WIFI_STATE_CONNECTED) {
		uart_logger_add_msg("[Wifi] Wifi is not connected to any AP, reset or configure in firmware\r\n", 0);
		return false;
	}

	if (!wifi_send_cmd_and_wait_respond(ESP_CMD_SEND_BYTES, 200, bytes)) return false;

	if (!wifi_send_cmd_and_wait_respond(data, 200)) return false;

	return true;
}

static void WIFI_Task(void *argument) {
	uart_logger_add_msg("[ESP32] Initializing esp32 uart driver...\r\n", 0);

	wifi_rx_sem = osSemaphoreNew(1, 0, NULL);

	esp32_uart_driver_state.controller_rx_sem = wifi_rx_sem;

	//  Init DMA transfer from memory to peripheral
	// So I can see the respond from ESP32
	// DMA is better than interrupt-based approach as it does not need to wake up CPU for every bytes
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

static bool wifi_handle_reset() {
	uart_logger_add_msg("[Wifi] Resetting...\r\n", 0);

	return true;
}

static bool wifi_handle_wait_ready() {
	uart_logger_add_msg("[ESP32] Wait for ESP32 to be ready for 1 second...\r\n", 0);

	osDelay(1000);

	return true;
}

static bool wifi_handle_join_ap() {
	if (!wifi_send_cmd_and_wait_respond(ESP_CMD_SET_MODE, 500, 1)) return false;

	osDelay(200);

	if (!wifi_send_cmd_and_wait_respond(ESP_CMD_JOIN_AP, 3000, WIFI_SSID, WIFI_PASSWORD)) return false;

	osDelay(3000);

	if (!wifi_send_cmd_and_wait_respond(ESP_CMD_UDP_START, 3000, HOST_IP, HOST_PORT)) return false;

	osDelay(3000);

	return true;
}

static bool wifi_handle_connected() {
	return true;
}

static bool wifi_handle_error() {
	uart_logger_add_msg("[Wifi] ERROR HAS OCCURED!\r\n", 0);

	return true;
}

static bool wifi_send_cmd_and_wait_respond(const char *fmt, const uint16_t timeout, ...) {
	/* drain stale signal */
	while (osSemaphoreAcquire(wifi_rx_sem, 0) == osOK) {
	}

	char buf[128];

	va_list args;
	va_start(args, timeout);
	vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);

	// Send data into the uart interface that connects to esp32
	bool ret = uart_send_data(&esp32_uart_driver_state, buf);

	// wait for response
	if (osSemaphoreAcquire(wifi_rx_sem, timeout) == osOK) {
		return ret;
	} else {
		// Timeout
		uart_logger_add_msg("[ESP32] Timeout.\r\n", 0);
	}

	return ret;
}
