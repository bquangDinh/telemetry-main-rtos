/*
 * cellular.c
 *
 *  Created on: Mar 11, 2026
 *      Author: buiqu
 */

#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>

#include "cellular.h"
#include "cmsis_os.h"
#include "uart_logger.h"
#include "uart_driver.h"

typedef enum {
	CELLULAR_STATE_RESET,
	CELLULAR_STATE_WAIT_READY,
	CELLULAR_STATE_READY,
	CELLULAR_STATE_CONFIGURED,
	CELLULAR_STATE_HUB_CONNECTED,
	CELLULAR_STATE_ERROR
} CellularState_t;

/* List of Commands */
#define CELLULAR_SET_HUB_CONNECTED_INTERRUPT "{\"req\":\"card.attn\",\"mode\":\"arm,connected\"}\r\n"
#define CELLULAR_ACQUIRE_BLUES_STATUS "{\"req\":\"card.status\"}\r\n"
#define CELLULAR_ACQUIRE_BLUES_VERSION "{\"req\":\"card.version\"}\r\n"
#define CELLULAR_ACQUIRE_BLUES_USAGE_TOTAL "{\"req\":\"card.usage.get\"}\r\n"
#define CELLULAR_ACQUIRE_BLUES_USAGE_HOURLY "{\"req\":\"card.usage.get\",\"mode\":\"1hour\"}\r\n"
#define CELLULAR_ACQUIRE_BLUES_USAGE_30DAYS "{\"req\":\"card.usage.get\",\"mode\":\"30day\"}\r\n"
#define CELLULAR_ACQUIRE_BLUES_USAGE_WITH_OFFSET "{\"req\":\"card.usage.get\",\"mode\":\"1day\",\"offset\":%d}\r\n"
#define CELLULAR_SET_BLUES_PRODUCT_ID_AND_SN "{\"req\":\"hub.set\",\"product\":\"%s\",\"sn\":\"%s\"}\r\n"
#define CELLULAR_SET_BLUES_HUB_MODE "{\"req\":\"hub.set\",\"mode\":\"%s\"}\r\n"
#define CELLULAR_SET_BLUES_OUTBOUND_INBOUND "{\"req\":\"hub.set\",\"outbound\":%d,\"inbound\":%d}\r\n"
#define CELLULAR_ACQUIRE_HUB_STATUS "{\"req\":\"hub.status\"}\r\n"
#define CELLULAR_ACQUIRE_HUB_INFO "{\"req\":\"hub.get\"}\r\n"
#define CELLULAR_HUB_FORCE_SYNC "{\"req\":\"hub.sync\"}\r\n"
#define CELLULAR_HUB_SYNC_STATUS "{\"req\":\"hub.sync.status\"}\r\n"

/* NoteCard Hub Modes */
// Periodically connect to the Notehub. This is the default value set on each Notecard after a factory reset.
#define CELLULAR_BLUES_HUB_MODE_PERIODIC "periodic"

// Enables an always-on network connection, for high power devices. Outbound data still syncs periodically, unless specified in a Note or File request.
#define CELLULAR_BLUES_HUB_MODE_CONTINUOUS "continuous"

// Disables periodic connection. The Notecard will not sync until it receives an explicit hub.sync request. OTA DFU updates are not available when using this mode.
#define CELLULAR_BLUES_HUB_MODE_MINIMUM "minimum"

// Disables automatic and manual syncs. hub.sync requests will be ignored in this mode. OTA DFU updates are not available when using this mode.
#define CELLULAR_BLUES_HUB_MODE_OFF "off"

// Puts the Notecard in DFU mode for IAP host MCU firmware updates. This mode is effectively the same as off in terms of the Notecard's network and Notehub connections.
#define CELLULAR_BLUES_HUB_MODE_DFU "dfu"

static void CELLULAR_Task(void *argument);

static osThreadId_t cellularTaskHandler;

static osSemaphoreId_t cellular_rx_sem;

static const osThreadAttr_t cellularTaskAttr = { .name = "cellularTask",
		.stack_size = 256 * 4, .priority = (osPriority_t) osPriorityRealtime1 };

static uart_driver_state_t blues_uart_driver_state = { 0 };

static CellularState_t cellular_state = CELLULAR_STATE_WAIT_READY;

static bool cellular_hub_connected = false;

static bool acquired_hub_connection_status = false;

static bool cellular_handle_reset();
static bool cellular_handle_wait_ready();
static bool cellular_handle_ready();
static bool cellular_handle_hub_connected();
static bool cellular_handle_error();

static bool cellular_send_cmd_and_wait_respond(const char *fmt,
		const uint16_t timeout, ...);

static bool cellular_transmit_data(const void *data, size_t bytes);

void CELLULAR_Task_Init(UART_HandleTypeDef *blues_uart_interface) {
	blues_uart_driver_state.huart = blues_uart_interface;

	cellularTaskHandler = osThreadNew(CELLULAR_Task, NULL, &cellularTaskAttr);
}

void CELLULAR_blues_uart_rx_callback(size_t len) {
	if (blues_uart_driver_state.initialized) {
		on_uart_rx_callback(&blues_uart_driver_state, len);
	}
}

void CELLULAR_ATTN_callback() {
	// Since ATTN should be set up to fire up when NoteCard's hub is connected successfully
	// worth to mention that ATTN could fire up if other events occurred if configured
	cellular_hub_connected = true;

	// Just to make sure the Blue Note Card has passed the configuration stage as this event could be fired sooner due to prior mis-configuration
	if (cellular_state == CELLULAR_STATE_CONFIGURED) {
		cellular_state = CELLULAR_STATE_HUB_CONNECTED;
	}
}

static void CELLULAR_Task(void *argument) {
	uart_logger_add_msg(
			"[Blues] Initializing blues note card uart driver...\r\n", 0);

	cellular_rx_sem = osSemaphoreNew(1, 0, NULL);

	blues_uart_driver_state.controller_rx_sem = cellular_rx_sem;

	//  Init DMA transfer from memory to peripheral
	// So I can see the respond from ESP32
	// DMA is better than interrupt-based approach as it does not need to wake up CPU for every bytes
	UART_Task_Init(&blues_uart_driver_state);

	uart_logger_add_msg("[ESP32] Initialized blues note card uart driver\r\n",
			0);

	while (1) {
		switch (cellular_state) {
		case CELLULAR_STATE_WAIT_READY:
			// Wait until the Wifi module is ready to transmit
			if (cellular_handle_wait_ready()) {
				cellular_state = CELLULAR_STATE_RESET;
			} else {
				cellular_state = CELLULAR_STATE_ERROR;
			}
		case CELLULAR_STATE_RESET:
			if (cellular_handle_reset()) {
				cellular_state = CELLULAR_STATE_READY;
			} else {
				cellular_state = CELLULAR_STATE_ERROR;
			}
			break;
		case CELLULAR_STATE_READY:
			if (cellular_handle_ready()) {
				// The hub connected event could be caught earlier due to mis-configuration prior to this step
				// So this variable is used to remember that the hub is connected already
				if (cellular_hub_connected) {
					cellular_state = CELLULAR_STATE_HUB_CONNECTED;
				} else {
					cellular_state = CELLULAR_STATE_CONFIGURED;
				}
			} else {
				cellular_state = CELLULAR_STATE_ERROR;
			}
			break;
		case CELLULAR_STATE_CONFIGURED:
			// Nothing to do here as the hub connection event will be caught by interrupt
			break;
		case CELLULAR_STATE_HUB_CONNECTED:
			if (!cellular_handle_hub_connected()) {
				cellular_state = CELLULAR_STATE_ERROR;
			}
			break;
		case CELLULAR_STATE_ERROR:
			cellular_handle_error();

			// Attempt resetting the NoteCard module
			cellular_state = CELLULAR_STATE_RESET;
			break;
		}

		osDelay(500);
	}
}

static bool cellular_handle_reset() {
	return true;
}

static bool cellular_handle_wait_ready() {
	uart_logger_add_msg(
			"[Blues] Wait for Blues Note Card to be ready for 30 seconds...\r\n",
			0);

	osDelay(30000);

	uart_logger_add_msg("[Blues] Ready\r\n", 0);

	return true;
}

static bool cellular_handle_ready() {
	char io_msg[256];

	uart_logger_add_msg("[Blues] Setting up NoteCard parameters...\r\n", 0);

	uart_logger_add_msg("\r\n[Blues] Set NoteCard Hub mode to off...\r\n", 0);

	if (!cellular_send_cmd_and_wait_respond(
	CELLULAR_SET_BLUES_HUB_MODE, 1000,
	CELLULAR_BLUES_HUB_MODE_OFF)) {
		return false;
	}

	osDelay(500);

	uart_logger_add_msg(
			"\r\n[Blues] Setting ATTN interrupt for hub connection...\r\n", 0);

	if (!cellular_send_cmd_and_wait_respond(
	CELLULAR_SET_HUB_CONNECTED_INTERRUPT, 1000)) {
		return false;
	}

	osDelay(500);

	uart_logger_add_msg("\r\n[Blues] Retrieving NoteCard status...\r\n", 0);

	if (!cellular_send_cmd_and_wait_respond(CELLULAR_ACQUIRE_BLUES_STATUS,
			1000)) {
		return false;
	}

	osDelay(500);

	uart_logger_add_msg("\r\n[Blues] Retrieving NoteCard version...\r\n", 0);

	if (!cellular_send_cmd_and_wait_respond(CELLULAR_ACQUIRE_BLUES_VERSION,
			1000)) {
		return false;
	}

	osDelay(500);

	uart_logger_add_msg("\r\n[Blues] Retrieving NoteCard usage total...\r\n",
			0);

	if (!cellular_send_cmd_and_wait_respond(CELLULAR_ACQUIRE_BLUES_USAGE_TOTAL,
			1000)) {
		return false;
	}

	osDelay(500);

	sprintf(io_msg,
			"\r\n[Blues] Set NoteCard Hub production ID (%s) and serial number (%s)...\r\n",
			CELLULAR_BLUES_PRODUCT_ID, CELLULAR_BLUES_SERIAL_NUMBER);

	uart_logger_add_msg(io_msg, 0);

	if (!cellular_send_cmd_and_wait_respond(
	CELLULAR_SET_BLUES_PRODUCT_ID_AND_SN, 1000, CELLULAR_BLUES_PRODUCT_ID,
	CELLULAR_BLUES_SERIAL_NUMBER)) {
		return false;
	}

	osDelay(500);

	sprintf(io_msg,
			"\r\n[Blues] Set NoteCard Hub inbound (%d minutes) and outbound (%d minutes) ...\r\n",
			CELLULAR_BLUES_INBOUND, CELLULAR_BLUES_OUTBOUND);

	uart_logger_add_msg(io_msg, 0);

	if (!cellular_send_cmd_and_wait_respond(
	CELLULAR_SET_BLUES_OUTBOUND_INBOUND, 1000,
	CELLULAR_BLUES_OUTBOUND, CELLULAR_BLUES_INBOUND)) {
		return false;
	}

	osDelay(500);

	uart_logger_add_msg(
			"\r\n[Blues] Set NoteCard Hub mode to periodic mode...\r\n", 0);

	if (!cellular_send_cmd_and_wait_respond(
	CELLULAR_SET_BLUES_HUB_MODE, 1000,
	CELLULAR_BLUES_HUB_MODE_PERIODIC)) {
		return false;
	}

	osDelay(500);

	return true;
}

static bool cellular_handle_hub_connected() {
	if (!acquired_hub_connection_status) {
		uart_logger_add_msg(
				"\r\n[Blues] Hub is connected. Acquiring hub connection status from NoteCard...\r\n",
				0);

		if (!cellular_send_cmd_and_wait_respond(
		CELLULAR_ACQUIRE_HUB_STATUS, 1000)) {
			return false;
		}

		osDelay(500);

		uart_logger_add_msg(
				"\r\n[Blues] Acquiring hub info from NoteCard...\r\n", 0);

		if (!cellular_send_cmd_and_wait_respond(
		CELLULAR_ACQUIRE_HUB_INFO, 1000)) {
			return false;
		}

		uart_logger_add_msg("\r\n[Cellular] Ready to transmit data...\r\n", 0);

		acquired_hub_connection_status = true;
	}

	return true;
}

static bool cellular_handle_error() {
	uart_logger_add_msg("\r\n[Cellular] AN ERROR HAS OCCURED!\r\n", 0);

	acquired_hub_connection_status = false;

	cellular_hub_connected = false;

	return true;
}

static bool cellular_send_cmd_and_wait_respond(const char *fmt,
		const uint16_t timeout, ...) {
	/* drain stale signal */
	while (osSemaphoreAcquire(cellular_rx_sem, 0) == osOK) {
	}

	char buf[128];

	va_list args;
	va_start(args, timeout);
	vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);

	// Send data into the uart interface that connects to esp32
	bool ret = uart_send_data(&blues_uart_driver_state, buf);

	// wait for response
	if (osSemaphoreAcquire(cellular_rx_sem, timeout) == osOK) {
		return ret;
	} else {
		// Timeout
		uart_logger_add_msg("[Blues] Timeout.\r\n", 0);
	}

	return ret;
}

static bool cellular_transmit_data(const void *data, size_t bytes) {
	if (cellular_state != CELLULAR_STATE_HUB_CONNECTED) {
		uart_logger_add_msg(
				"[Cellular] Blues NoteCard hub is not connected or mis-configured, reset or set correct parameters in firmware\r\n",
				0);
		return false;
	}

	return true;
}
