/*
 * cellular.c
 *
 *  Created on: Mar 11, 2026
 *      Author: buiqu
 */

#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "cellular.h"
#include "cmsis_os.h"
#include "uart_logger.h"
#include "uart_driver.h"

typedef struct {
	uint32_t id;
	uint8_t payload[CELLULAR_MAX_DATA_LEN];
	uint8_t len;
} cellular_payload_t;

typedef struct {
	cellular_payload_t buffer[CELLULAR_PAYLOAD_QUEUE_MAX_CAPACITY];
	uint16_t tail;
	uint16_t head;
	uint16_t count;
	osMutexId_t mutex;
	osSemaphoreId_t items;
} cellular_queue_t;

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

static osSemaphoreId_t cellular_disabled_sem;

static const osThreadAttr_t cellularTaskAttr = { .name = "cellularTask",
		.stack_size = 256 * 4, .priority = (osPriority_t) osPriorityRealtime1 };

static uart_driver_state_t blues_uart_driver_state = { 0 };

static CellularState_t cellular_state = CELLULAR_STATE_WAIT_READY;

static bool cellular_hub_connected = false;

static bool acquired_hub_connection_status = false;

static cellular_queue_t cellular_payload_queue;

static volatile uint8_t current_retry_count = 0;

volatile cellular_health_state_t cellular_health_state = { .last_progress = 0,
		.wait_start = 0, .current_state = CELLULAR_STATE_WAIT_READY };

static void uart_rx_line_callback_handler(const uint8_t *data, size_t len);

static bool cellular_handle_reset();
static bool cellular_handle_wait_ready();
static bool cellular_handle_ready();
static bool cellular_handle_hub_connected();
static bool cellular_handle_transmit();
static bool cellular_handle_disable();
static bool cellular_handle_error();

static bool cellular_send_cmd_and_wait_respond(const char *fmt,
		const uint16_t timeout, ...);

static bool cellular_payload_pop(cellular_payload_t *out);

void CELLULAR_Task_Init(UART_HandleTypeDef *blues_uart_interface) {
	blues_uart_driver_state.huart = blues_uart_interface;
	blues_uart_driver_state.rx_line_callback = uart_rx_line_callback_handler;

	cellular_payload_queue.mutex = osMutexNew(NULL);
	cellular_payload_queue.items = osSemaphoreNew(
	CELLULAR_PAYLOAD_QUEUE_MAX_CAPACITY, 0, NULL);

	cellular_rx_sem = osSemaphoreNew(1, 0, NULL);
	cellular_disabled_sem = osSemaphoreNew(1, 0, NULL);

	cellularTaskHandler = osThreadNew(CELLULAR_Task, NULL, &cellularTaskAttr);
}

void CELLULAR_blues_uart_rx_callback(size_t len) {
	if (blues_uart_driver_state.initialized) {
		on_uart_rx_callback(&blues_uart_driver_state, len);
	}
}

void CELLULAR_blues_uart_tx_callback() {
	if (blues_uart_driver_state.initialized) {
		on_uart_tx_callback(&blues_uart_driver_state);
	}
}

void CELLULAR_blues_uart_error_callback() {
	if (blues_uart_driver_state.initialized) {
		on_uart_err_callback(&blues_uart_driver_state);
	}
}

void CELLULAR_ATTN_callback() {
	// Since ATTN should be set up to fire up when NoteCard's hub is connected successfully
	// worth to mention that ATTN could fire up if other events occurred if configured
	cellular_hub_connected = true;

	// Just to make sure the Blue Note Card has passed the configuration stage as this event could be fired sooner due to prior mis-configuration
	if (cellular_state == CELLULAR_STATE_CONFIGURED) {
		cellular_state = CELLULAR_STATE_HUB_CONNECTED;
	} else if (cellular_state == CELLULAR_STATE_DISABLED) {
		// In case the module has been disabled due to number of retries reached
		// This one signal could wake the module back up and init again
		osSemaphoreRelease(cellular_disabled_sem);
	}
}

bool CELLULAR_add_payload_to_queue(const uint32_t id, const uint8_t *data,
		uint16_t len) {
	if (id == 0 || data == NULL || len == 0)
		return false;

	osMutexAcquire(cellular_payload_queue.mutex, osWaitForever);

	if (cellular_payload_queue.count >= CELLULAR_PAYLOAD_QUEUE_MAX_CAPACITY) {
		osMutexRelease(cellular_payload_queue.mutex);

		return false;
	}

	cellular_payload_t *slot =
			&cellular_payload_queue.buffer[cellular_payload_queue.head];

	if (len >= CELLULAR_MAX_DATA_LEN) {
		len = CELLULAR_MAX_DATA_LEN - 1;
	}

	slot->len = len;

	memset(slot->payload, 0, CELLULAR_MAX_DATA_LEN);

	memcpy(slot->payload, data, len);

	cellular_payload_queue.head = (cellular_payload_queue.head + 1)
			% CELLULAR_PAYLOAD_QUEUE_MAX_CAPACITY;

	cellular_payload_queue.count++;

	osMutexRelease(cellular_payload_queue.mutex);

	// Signal that there is item to consume
	osSemaphoreRelease(cellular_payload_queue.items);

	return true;
}

bool CELLULAR_transmit_data(const uint32_t id, const uint8_t *data, size_t len,
		uint16_t timeout) {
	if (cellular_state != CELLULAR_STATE_READY_TO_TRANSMIT) {
		uart_logger_add_msg("[Cellular] Not ready to transmit!\r\n", 0);
		return false;
	}

	if (len > CELLULAR_MAX_DATA_LEN) {
		uart_logger_add_msg(
				"[Cellular] Failed to send data, maximum data length is 64 bytes\r\n",
				0);
		return false;
	}

	static char buffer[1024];
	int offset = 0;

	memset(buffer, 0, 1024);

	// Start JSON
	offset +=
			snprintf(&buffer[offset], sizeof(buffer) - offset,
					"{\"req\":\"note.add\",\"file\":\"data.qo\",\"sync\":true,\"body\":{");

	if (offset < 0 || offset >= (int) sizeof(buffer)) {
		uart_logger_add_msg(
				"[Cellular] Failed to build Note JSON: Buffer overflow at JSON start\r\n",
				0);
		return false;
	}

	// Build JSON body
	// Add id
	offset += snprintf(&buffer[offset], sizeof(buffer) - offset, "\"id\":%lu,",
			(unsigned long) id);

	if (offset < 0 || offset >= (int) sizeof(buffer)) {
		uart_logger_add_msg(
				"[Cellular] Failed to build Note JSON: Buffer overflow at JSON start\r\n",
				0);
		return false;
	}

	// Add payload as hex string
	offset += snprintf(&buffer[offset], sizeof(buffer) - offset,
			"\"payload\":\"");

	if (offset < 0 || offset >= (int) sizeof(buffer)) {
		uart_logger_add_msg(
				"[Cellular] Failed to build Note JSON: Buffer overflow at payload start\r\n",
				0);
		return false;
	}

	for (size_t i = 0; i < len; i++) {
		offset += snprintf(&buffer[offset], sizeof(buffer) - offset, "%02X ",
				data[i]);

		if (offset < 0 || offset >= (int) sizeof(buffer)) {
			uart_logger_add_msg(
					"[Cellular] Failed to build Note JSON: Buffer overflow while writing payload\r\n",
					0);
			return false;
		}
	}

	offset += snprintf(&buffer[offset], sizeof(buffer) - offset,
			"\",\"len\":%lu}}\n", (unsigned long) len);

	if (offset < 0 || offset >= (int) sizeof(buffer)) {
		uart_logger_add_msg(
				"[Cellular] Failed to build Note JSON: Buffer overflow while writing payload\r\n",
				0);
		return false;
	}

	uart_logger_add_msg(buffer, 0);

	return cellular_send_cmd_and_wait_respond(buffer, timeout);
}

static void CELLULAR_Task(void *argument) {
	uart_logger_add_msg(
			"[Blues] Initializing blues note card uart driver...\r\n", 0);

	blues_uart_driver_state.controller_rx_sem = cellular_rx_sem;

	UART_Task_Init(&blues_uart_driver_state);

	uart_logger_add_msg("[ESP32] Initialized blues note card uart driver\r\n",
			0);

	cellular_health_state.last_progress = osKernelGetTickCount();

	while (1) {
		cellular_health_state.wait_start = osKernelGetTickCount();

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
			if (cellular_handle_hub_connected()) {
				cellular_state = CELLULAR_STATE_READY_TO_TRANSMIT;
			} else {
				cellular_state = CELLULAR_STATE_ERROR;
			}
			break;
		case CELLULAR_STATE_READY_TO_TRANSMIT:
			if (!cellular_handle_transmit()) {
				cellular_state = CELLULAR_STATE_ERROR;
			}
			break;
		case CELLULAR_STATE_DISABLED:
			cellular_handle_disable();

			break;
		case CELLULAR_STATE_ERROR:
			cellular_handle_error();

			if (current_retry_count < CELLULAR_NUM_RETRIES) {
				// Attempt resetting the NoteCard module
				cellular_state = CELLULAR_STATE_WAIT_READY;

				current_retry_count++;
			} else {
				uart_logger_add_msg(
						"[Cellular] Number of retries reached. Disable Cellular module\r\n",
						0);
				cellular_state = CELLULAR_STATE_DISABLED;
			}

			break;
		}

		// Update wifi health info
		cellular_health_state.last_progress = osKernelGetTickCount();
		cellular_health_state.current_state = cellular_state;

		osDelay(100);
	}
}

static void uart_rx_line_callback_handler(const uint8_t *data, size_t len) {
	osSemaphoreRelease(cellular_rx_sem);
}

static bool cellular_handle_reset() {
	return true;
}

static bool cellular_handle_wait_ready() {
	uart_logger_add_msg(
			"[Blues] Wait for Blues Note Card to be ready for 3 seconds...\r\n",
			0);

	osDelay(3000);

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

	if (!cellular_send_cmd_and_wait_respond(
	CELLULAR_ACQUIRE_BLUES_USAGE_TOTAL, 1000)) {
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

static bool cellular_handle_transmit() {
	cellular_payload_t payload;

	// Wait for items
	osSemaphoreAcquire(cellular_payload_queue.items, osWaitForever);

	if (cellular_payload_pop(&payload)) {
		uart_logger_add_msg("[Cellular] Data is available!\r\n", 0);

		bool ret = CELLULAR_transmit_data(payload.id, payload.payload,
				payload.len, 3000);

		return ret;
	}

	return true;
}

static bool cellular_handle_disable() {
	if (osSemaphoreAcquire(cellular_disabled_sem, 1000) == osOK) {
		cellular_state = CELLULAR_STATE_WAIT_READY;

		current_retry_count = 0;
	}

	return true;
}

static bool cellular_handle_error() {
	uart_logger_add_msg("\r\n[Cellular] AN ERROR HAS OCCURED!\r\n", 0);

	acquired_hub_connection_status = false;

	cellular_hub_connected = false;

	uart_logger_add_msg(
			"\r\n[Cellular] Waiting 3 seconds before trying again...!\r\n", 0);

	// Wait 3 seconds before try again
	osDelay(3000);

	return true;
}

static bool cellular_send_cmd_and_wait_respond(const char *fmt,
		const uint16_t timeout, ...) {
	if (fmt == NULL) {
		uart_logger_add_msg("[Cellular] Invalid arguments\r\n", 0);
		return false;
	}

	/* drain stale signal */
	while (osSemaphoreAcquire(cellular_rx_sem, 0) == osOK) {
	}

	char buf[128];

	va_list args;
	va_start(args, timeout);
	vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);

	// Send data into the uart interface that connects to blues
	bool ret = uart_send_data(&blues_uart_driver_state, buf);

	if (!ret) {
		uart_logger_add_msg("[Cellular] Failed to send data over UART\r\n", 0);

		return false;
	}

	// wait for response
	if (osSemaphoreAcquire(cellular_rx_sem, timeout) == osOK) {
		return true;
	} else {
		// Timeout
		uart_logger_add_msg("[Blues] Timeout.\r\n", 0);

		return false;
	}

	return true;
}

static bool cellular_payload_pop(cellular_payload_t *out) {
	if (out == NULL)
		return false;

	osMutexAcquire(cellular_payload_queue.mutex, osWaitForever);

	if (cellular_payload_queue.count == 0) {
		osMutexRelease(cellular_payload_queue.mutex);

		return false;
	}

	*out = cellular_payload_queue.buffer[cellular_payload_queue.tail];
	cellular_payload_queue.tail = (cellular_payload_queue.tail + 1)
			% CELLULAR_PAYLOAD_QUEUE_MAX_CAPACITY;
	cellular_payload_queue.count--;

	osMutexRelease(cellular_payload_queue.mutex);

	return true;
}
