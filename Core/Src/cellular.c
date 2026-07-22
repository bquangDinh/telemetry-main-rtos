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
#include "can_storage.h"

#define BULK_BUFFER_SIZE 4096

#define CELL_HEALTH_LED_PIN GPIO_PIN_4
#define CELL_HEALTH_LED_PORT GPIOD

#define CELL_ERR_LED_PIN GPIO_PIN_5
#define CELL_ERR_LED_PORT GPIOD

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

// Send data every 10 seconds
#define CELLULAR_SYNC_INTERVAL_MS 10000U

#define CELLULAR_WAIT_READY_TIMEOUT_MS 10000U

typedef enum {
	ERR_NONE,
	ERR_WAIT_READY,
	ERR_RESET,
	ERR_CONFIGURED,
	ERR_READY,
	ERR_HUB_CONNECTED,
	ERR_TRANSMIT,
	ERR_DISABLE,
} cellular_error_t;

/**
 * @brief Payload structure for cellular messages, which includes the CAN ID, payload data, and length of the payload. This structure will be used to represent the messages received from the CAN interface that are intended for processing by the cellular module, allowing for organized storage and management of the incoming data.
 */
typedef struct {
	uint32_t id;
	uint8_t payload[CELLULAR_MAX_DATA_LEN];
	uint8_t len;
} cellular_payload_t;

static void CELLULAR_Task(void *argument);

static osThreadId_t cellularTaskHandler;

static osSemaphoreId_t cellular_rx_sem;

static osSemaphoreId_t cellular_attn_sem;

static osSemaphoreId_t cellular_disabled_sem;

static const osThreadAttr_t cellularTaskAttr = { .name = "cellularTask",
		.stack_size = 256 * 4, .priority = (osPriority_t) osPriorityRealtime1 };

static uart_driver_state_t blues_uart_driver_state = { 0 };

static CellularState_t cellular_state = CELLULAR_STATE_WAIT_READY;

static bool acquired_hub_connection_status = false;

static volatile uint8_t current_retry_count = 0;

static uint32_t last_sync_time_ms = 0;

static cellular_error_t last_error_code = ERR_NONE;

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

static bool cellular_send_bulk_data();

/**
 * @brief Helper function to send a formatted command to the NoteCard and wait for a response. This function will format the command string using the provided format and arguments, send it to the NoteCard via the cellular UART driver, and then wait for a response within the specified timeout period. The function will return true if a response is received within the timeout, or false if there is an error or if the timeout is reached without receiving a response.
 * @param fmt The format string for the command to be sent, which can include format specifiers for additional arguments.
 * @param timeout The maximum time to wait for a response from the NoteCard, in milliseconds.
 * @param ... Additional arguments to be formatted into the command string, corresponding to the format specifiers in the fmt parameter.
 * @return true if a response is received within the timeout, false if there is an error or if the timeout is reached without receiving a response.
 */
static bool cellular_send_cmd_and_wait_respond(const char *fmt,
		const uint16_t timeout, ...);

/**
 * @brief Signal the NoteCard to send all pending data immediately
 */
static bool cellular_force_sync_with_note_card();

/**
 * Logging functions
 */
#if CELLULAR_LOG_ENABLED
static void cellular_logln_impl(const char *msg);
static void cellular_log_fmt_ln_impl(const char *fmt, ...);

#define cellular_log(msg) cellular_logln_impl(msg)
#define cellular_log_fmt(fmt, ...) cellular_log_fmt_ln_impl((fmt), ##__VA_ARGS__)
#else
#define cellular_log(msg) ((void) 0)
#define cellular_log_fmt(fmt, ...) ((void) 0)
#endif

#if CELLULAR_LOG_ENABLED
static void cellular_logln_impl(const char *msg) {
	uart_logger_add_msg_format("[Cellular] %s\r\n", msg);
}

static void cellular_log_fmt_ln_impl(const char *fmt, ...) {
	char buf[128];
	va_list args;

	va_start(args, fmt);
	int len = vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);

	if (len > 0 && len < (int) sizeof(buf)) {
		uart_logger_add_msg_format("[Cellular] %s\r\n", buf);
	}
}
#endif

/**
 * @brief Initialize the cellular task, UART driver state, and queue primitives.
 */
void CELLULAR_Task_Init(UART_HandleTypeDef *blues_uart_interface) {
	if (blues_uart_interface == NULL) {
		return;
	}

	blues_uart_driver_state.huart = blues_uart_interface;
	blues_uart_driver_state.rx_line_callback = uart_rx_line_callback_handler;

	cellular_rx_sem = osSemaphoreNew(1, 0, NULL);
	cellular_attn_sem = osSemaphoreNew(1, 0, NULL);
	cellular_disabled_sem = osSemaphoreNew(1, 0, NULL);

	if (cellular_rx_sem == NULL || cellular_attn_sem == NULL
			|| cellular_disabled_sem == NULL) {
		cellular_log("Failed to allocate memory for semaphore");
		return;
	}

	cellularTaskHandler = osThreadNew(CELLULAR_Task, NULL, &cellularTaskAttr);

	if (cellularTaskHandler == NULL) {
		cellular_log("Failed to create cellular task");
		return;
	}
}

/**
 * @brief Forward received UART bytes from the Blues interface into the cellular UART driver.
 */
void CELLULAR_blues_uart_rx_callback(size_t len) {
	if (blues_uart_driver_state.initialized) {
		on_uart_rx_callback(&blues_uart_driver_state, len);
	}
}

/**
 * @brief Forward UART transmit-complete events into the cellular UART driver.
 */
void CELLULAR_blues_uart_tx_callback() {
	if (blues_uart_driver_state.initialized) {
		on_uart_tx_callback(&blues_uart_driver_state);
	}
}

/**
 * @brief Forward UART error events into the cellular UART driver.
 */
void CELLULAR_blues_uart_error_callback() {
	if (blues_uart_driver_state.initialized) {
		on_uart_err_callback(&blues_uart_driver_state);
	}
}

/**
 * @brief Handle the ATTN signal from the NoteCard and advance the cellular state.
 */
void CELLULAR_ATTN_callback() {
	if (cellular_state == CELLULAR_STATE_CONFIGURED) {
		osSemaphoreRelease(cellular_attn_sem);
	} else if (cellular_state == CELLULAR_STATE_DISABLED) {
		osSemaphoreRelease(cellular_disabled_sem);
	}
}

/**
 * @brief Format a payload as a NoteCard note.add request and send it immediately.
 */
bool CELLULAR_transmit_data(const uint32_t id, const uint8_t *data, size_t len,
		uint16_t timeout) {
	if (cellular_state != CELLULAR_STATE_READY_TO_TRANSMIT) {
		cellular_log("Not ready to transmit!");
		return false;
	}

	if (len > CELLULAR_MAX_DATA_LEN) {
		cellular_log("Failed to send data, maximum data length is 64 bytes");
		return false;
	}

	static char buffer[1024];
	int offset = 0;

	memset(buffer, 0, 1024);

	// Start JSON
	offset +=
			snprintf(&buffer[offset], sizeof(buffer) - offset,
					"{\"req\":\"note.add\",\"file\":\"data.qo\",\"body\":{");

	if (offset < 0 || offset >= (int) sizeof(buffer)) {
		cellular_log("Failed to build Note JSON: Buffer overflow at JSON start");
		return false;
	}

	// Build JSON body
	// Add id
	offset += snprintf(&buffer[offset], sizeof(buffer) - offset, "\"id\":%lu,",
			(unsigned long) id);

	if (offset < 0 || offset >= (int) sizeof(buffer)) {
		cellular_log("Failed to build Note JSON: Buffer overflow at JSON start");
		return false;
	}

	// Add payload as hex string
	offset += snprintf(&buffer[offset], sizeof(buffer) - offset,
			"\"payload\":\"");

	if (offset < 0 || offset >= (int) sizeof(buffer)) {
		cellular_log("Failed to build Note JSON: Buffer overflow at payload start");
		return false;
	}

	for (size_t i = 0; i < len; i++) {
		offset += snprintf(&buffer[offset], sizeof(buffer) - offset, "%02X ",
				data[i]);

		if (offset < 0 || offset >= (int) sizeof(buffer)) {
			cellular_log("Failed to build Note JSON: Buffer overflow while writing payload");
			return false;
		}
	}

	offset += snprintf(&buffer[offset], sizeof(buffer) - offset,
			"\",\"len\":%lu}}\r\n", (unsigned long) len);

	if (offset < 0 || offset >= (int) sizeof(buffer)) {
		cellular_log("Failed to build Note JSON: Buffer overflow while writing payload");
		return false;
	}

	cellular_log(buffer);

	return cellular_send_cmd_and_wait_respond(buffer, timeout);
}

/**
 * @brief Main cellular task that drives the NoteCard state machine.
 */
static void CELLULAR_Task(void *argument) {
	HAL_GPIO_WritePin(CELL_HEALTH_LED_PORT, CELL_HEALTH_LED_PIN, GPIO_PIN_SET);

	HAL_GPIO_WritePin(CELL_ERR_LED_PORT, CELL_ERR_LED_PIN, GPIO_PIN_SET);

	osDelay(1000);

	HAL_GPIO_WritePin(CELL_HEALTH_LED_PORT, CELL_HEALTH_LED_PIN, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(CELL_ERR_LED_PORT, CELL_ERR_LED_PIN, GPIO_PIN_RESET);

	cellular_log("Initializing blues note card uart driver...");

	blues_uart_driver_state.controller_rx_sem = cellular_rx_sem;

	UART_Task_Init(&blues_uart_driver_state);

	cellular_log("Initialized blues note card uart driver");

	cellular_health_state.last_progress = osKernelGetTickCount();

	while (1) {
		cellular_health_state.wait_start = osKernelGetTickCount();

		switch (cellular_state) {
		case CELLULAR_STATE_WAIT_READY:
			// Wait until the Wifi module is ready to transmit
			if (cellular_handle_wait_ready()) {
				cellular_state = CELLULAR_STATE_RESET;
			} else {
				last_error_code = ERR_WAIT_READY;

				cellular_state = CELLULAR_STATE_ERROR;
			}
			break;
		case CELLULAR_STATE_RESET:
			if (cellular_handle_reset()) {
				cellular_state = CELLULAR_STATE_READY;
			} else {
				last_error_code = ERR_RESET;

				cellular_state = CELLULAR_STATE_ERROR;
			}
			break;
		case CELLULAR_STATE_READY:
			if (cellular_handle_ready()) {
				cellular_state = CELLULAR_STATE_CONFIGURED;
			} else {
				last_error_code = ERR_READY;

				cellular_state = CELLULAR_STATE_ERROR;
			}
			break;
		case CELLULAR_STATE_CONFIGURED:
			// Wait for ATTN event
			if (osSemaphoreAcquire(cellular_attn_sem, osWaitForever) == osOK) {
				cellular_state = CELLULAR_STATE_HUB_CONNECTED;
			} else {
				last_error_code = ERR_CONFIGURED;

				cellular_state = CELLULAR_STATE_ERROR;
			}
			break;
		case CELLULAR_STATE_HUB_CONNECTED:
			if (cellular_handle_hub_connected()) {
				// The module is fully recovered, so reset it here
				current_retry_count = 0;

				cellular_state = CELLULAR_STATE_READY_TO_TRANSMIT;
			} else {
				last_error_code = ERR_HUB_CONNECTED;

				cellular_state = CELLULAR_STATE_ERROR;
			}
			break;
		case CELLULAR_STATE_READY_TO_TRANSMIT:
			if (!cellular_handle_transmit()) {
				last_error_code = ERR_TRANSMIT;

				cellular_state = CELLULAR_STATE_ERROR;
			}
			break;
		case CELLULAR_STATE_DISABLED:
			if (!cellular_handle_disable()) {
				last_error_code = ERR_DISABLE;

				cellular_state = CELLULAR_STATE_ERROR;
			}

			break;
		case CELLULAR_STATE_ERROR:
			cellular_handle_error();

			if (current_retry_count < CELLULAR_NUM_RETRIES) {
				// Attempt resetting the NoteCard module
				cellular_state = CELLULAR_STATE_WAIT_READY;

				current_retry_count++;
			} else {
				cellular_log("Number of retries reached. Disable Cellular module");
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

/**
 * @brief Wake the cellular task when a complete UART line is received.
 */
static void uart_rx_line_callback_handler(const uint8_t *data, size_t len) {
	(void) data;
	(void) len;

	osSemaphoreRelease(cellular_rx_sem);
}

/**
 * @brief Handle the reset state of the cellular state machine.
 */
static bool cellular_handle_reset() {
	return true;
}

/**
 * @brief Wait for the NoteCard to become ready after startup or reset.
 */
static bool cellular_handle_wait_ready() {
	last_error_code = 0;

	cellular_log_fmt("Waiting for NoteCard to become ready (timeout: %d ms)...",
			CELLULAR_WAIT_READY_TIMEOUT_MS);

	osDelay(CELLULAR_WAIT_READY_TIMEOUT_MS);

	cellular_log("Ready");

	return true;
}

/**
 * @brief Configure the NoteCard for steady-state operation.
 */
static bool cellular_handle_ready() {
	cellular_log("Setting up NoteCard parameters...");

	cellular_log("Set NoteCard Hub mode to off...");

	if (!cellular_send_cmd_and_wait_respond(
	CELLULAR_SET_BLUES_HUB_MODE, 1000,
	CELLULAR_BLUES_HUB_MODE_OFF)) {
		return false;
	}

	osDelay(500);

	cellular_log("Setting ATTN interrupt for hub connection...");

	if (!cellular_send_cmd_and_wait_respond(
	CELLULAR_SET_HUB_CONNECTED_INTERRUPT, 1000)) {
		return false;
	}

	osDelay(500);

	cellular_log("Retrieving NoteCard status...");

	if (!cellular_send_cmd_and_wait_respond(CELLULAR_ACQUIRE_BLUES_STATUS,
			1000)) {
		return false;
	}

	osDelay(500);

	cellular_log("Retrieving NoteCard version...");

	if (!cellular_send_cmd_and_wait_respond(CELLULAR_ACQUIRE_BLUES_VERSION,
			1000)) {
		return false;
	}

	osDelay(500);

	cellular_log("Retrieving NoteCard usage total...");

	if (!cellular_send_cmd_and_wait_respond(
	CELLULAR_ACQUIRE_BLUES_USAGE_TOTAL, 1000)) {
		return false;
	}

	osDelay(500);

	cellular_log_fmt("Set NoteCard Hub production ID (%s) and serial number (%s)...",
			CELLULAR_BLUES_PRODUCT_ID, CELLULAR_BLUES_SERIAL_NUMBER);

	if (!cellular_send_cmd_and_wait_respond(
	CELLULAR_SET_BLUES_PRODUCT_ID_AND_SN, 1000, CELLULAR_BLUES_PRODUCT_ID,
	CELLULAR_BLUES_SERIAL_NUMBER)) {
		return false;
	}

	osDelay(500);

	cellular_log_fmt("Set NoteCard Hub inbound (%d minutes) and outbound (%d minutes) ...",
			CELLULAR_BLUES_INBOUND, CELLULAR_BLUES_OUTBOUND);

	if (!cellular_send_cmd_and_wait_respond(
	CELLULAR_SET_BLUES_OUTBOUND_INBOUND, 1000,
	CELLULAR_BLUES_OUTBOUND, CELLULAR_BLUES_INBOUND)) {
		return false;
	}

	osDelay(500);

	cellular_log("Set NoteCard Hub mode to periodic mode...");

	if (!cellular_send_cmd_and_wait_respond(
	CELLULAR_SET_BLUES_HUB_MODE, 1000,
	CELLULAR_BLUES_HUB_MODE_PERIODIC)) {
		return false;
	}

	osDelay(500);

	return true;
}

/**
 * @brief Handle the hub-connected state and perform one-time NoteCard queries.
 */
static bool cellular_handle_hub_connected() {
	if (!acquired_hub_connection_status) {
		cellular_log("Hub is connected. Acquiring hub connection status from NoteCard...");

		if (!cellular_send_cmd_and_wait_respond(
		CELLULAR_ACQUIRE_HUB_STATUS, 1000)) {
			return false;
		}

		osDelay(500);

		cellular_log("Acquiring hub info from NoteCard...");

		if (!cellular_send_cmd_and_wait_respond(
		CELLULAR_ACQUIRE_HUB_INFO, 1000)) {
			return false;
		}

		cellular_log("Ready to transmit data...");

		acquired_hub_connection_status = true;
	}

	// Turn Indicator LEDs
	HAL_GPIO_WritePin(CELL_HEALTH_LED_PORT, CELL_HEALTH_LED_PIN, GPIO_PIN_SET);

	HAL_GPIO_WritePin(CELL_ERR_LED_PORT, CELL_ERR_LED_PIN, GPIO_PIN_RESET);

	return true;
}

/**
 * @brief Keep the NoteCard alive and transmit queued payloads when available.
 */
static bool cellular_handle_transmit() {
	// Send all data in the can storage
	if (!cellular_send_bulk_data()) {
		return false;
	}

	// Check if it's time to sync with the NoteCard to send the data immediately instead of waiting for the next periodic sync
		if (HAL_GetTick() - last_sync_time_ms > CELLULAR_SYNC_INTERVAL_MS) {
			bool ret_sync = cellular_force_sync_with_note_card();

			if (ret_sync) {
				last_sync_time_ms = HAL_GetTick();
			} else {
				last_error_code = 7;

				return false;
			}
		}

	return true;
}

/**
 * @brief Park the cellular module until it is explicitly re-enabled.
 */
static bool cellular_handle_disable() {
	if (osSemaphoreAcquire(cellular_disabled_sem, osWaitForever) != osOK) {
		return false;
	}

	cellular_state = CELLULAR_STATE_WAIT_READY;

	return true;
}

/**
 * @brief Handle the error state by resetting flags, updating LEDs, and delaying retry.
 */
static bool cellular_handle_error() {
	cellular_log_fmt("AN ERROR HAS OCCURED | Code: %u", last_error_code);

	acquired_hub_connection_status = false;

	HAL_GPIO_WritePin(CELL_HEALTH_LED_PORT, CELL_HEALTH_LED_PIN, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(CELL_ERR_LED_PORT, CELL_ERR_LED_PIN, GPIO_PIN_SET);

	cellular_log("Waiting 3 seconds before trying again...!");

	// Wait 3 seconds before try again
	osDelay(3000);

	return true;
}

/**
 * @brief Format a command, send it over UART, and wait for a response semaphore.
 */
static bool cellular_send_cmd_and_wait_respond(const char *fmt,
		const uint16_t timeout, ...) {
	if (fmt == NULL) {
		cellular_log("Invalid arguments");
		return false;
	}

	/* drain stale signal */
	while (osSemaphoreAcquire(cellular_rx_sem, 0) == osOK) {
	}

	char buf[BULK_BUFFER_SIZE];

	va_list args;
	va_start(args, timeout);
	int len = vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);

	if (len < 0 || len >= (int)sizeof(buf)) {
		cellular_log("Invalid CMD -- truncated");

		return false;
	}

	// Send data into the uart interface that connects to blues
	bool ret = uart_send_data(&blues_uart_driver_state, buf);

	if (!ret) {
		cellular_log("Failed to send data over UART");

		return false;
	}

	// wait for response
	if (osSemaphoreAcquire(cellular_rx_sem, timeout) == osOK) {
		return true;
	} else {
		// Timeout
		cellular_log("Timeout.");

		return false;
	}

	return true;
}

static bool cellular_force_sync_with_note_card() {
	cellular_log("Forcing sync with NoteCard...");

	if (!cellular_send_cmd_and_wait_respond(
	CELLULAR_HUB_FORCE_SYNC, 1000)) {
		cellular_log("Failed to force sync with NoteCard");

		return false;
	}

	cellular_log("Successfully forced sync with NoteCard");

	return true;
}

static bool cellular_send_bulk_data() {
	can_storage_t *can_storage = get_can_storage();

	static char buffer[BULK_BUFFER_SIZE];

	if (osMutexAcquire(can_storage->mutex, osWaitForever) != osOK) {
		cellular_log("Failed to acquire CAN storage mutex");
		return false;
	}

	int offset = 0;

	memset(buffer, 0, BULK_BUFFER_SIZE);

	// Start JSON
	offset +=
			snprintf(&buffer[offset], sizeof(buffer) - offset,
					"{\"req\":\"note.add\",\"file\":\"data.qo\",\"body\":{[");

	if (offset < 0 || offset >= (int) sizeof(buffer)) {
		cellular_log("Failed to build Note JSON: Buffer overflow at JSON start");
		return false;
	}

	// Build JSON body
	// Format: { "id": <id>, "payload": "<hex string>", "len": <len> }, ...
	for (uint16_t i = 0; i < TABLE_SIZE; i++) {
		can_msg_node_t *node = &can_storage->nodes[i];

		if (node->valid) {
			offset += snprintf(&buffer[offset], sizeof(buffer) - offset,
					"{\"id\":%lu,\"payload\":\"",
					(unsigned long) node->key);

			for (size_t j = 0; j < node->value.len; j++) {
				offset += snprintf(&buffer[offset], sizeof(buffer) - offset,
						"%02X", node->value.payload[j]);
			}

			offset += snprintf(&buffer[offset], sizeof(buffer) - offset,
					"\",\"len\":%lu},",
					(unsigned long) node->value.len);

			if (offset >= (int) sizeof(buffer)) {
				cellular_log("Buffer overflow while building bulk data payload");
				osMutexRelease(can_storage->mutex);
				return false;
			}
		}
	}

	// End JSON
	offset += snprintf(&buffer[offset], sizeof(buffer) - offset, "]}\r\n");

	if (offset < 0 || offset >= (int) sizeof(buffer)) {
		cellular_log("Failed to build Note JSON: Buffer overflow at JSON end");
		osMutexRelease(can_storage->mutex);
		return false;
	}

	osMutexRelease(can_storage->mutex);

	return cellular_send_cmd_and_wait_respond(buffer, 3000);
}
