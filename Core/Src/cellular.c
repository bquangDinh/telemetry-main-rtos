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

/**
 * @brief Payload structure for cellular messages, which includes the CAN ID, payload data, and length of the payload. This structure will be used to represent the messages received from the CAN interface that are intended for processing by the cellular module, allowing for organized storage and management of the incoming data.
 */
typedef struct {
	uint32_t id;
	uint8_t payload[CELLULAR_MAX_DATA_LEN];
	uint8_t len;
} cellular_payload_t;

/**
 * @brief Queue structure for managing incoming cellular payloads, which includes a buffer for storing the payloads, indices for the head and tail of the queue, a count of the number of items in the queue, and synchronization primitives (mutex and semaphore) for managing access to the queue in a thread-safe manner. This structure will be used by the cellular task to manage the flow of incoming messages and ensure that they are processed in a timely manner while preventing overflow and managing memory usage effectively.
 */
typedef struct {
	cellular_payload_t buffer[CELLULAR_PAYLOAD_QUEUE_MAX_CAPACITY];
	uint16_t tail;
	uint16_t head;
	uint16_t count;
	osMutexId_t mutex;
	osSemaphoreId_t items;
} cellular_queue_t;

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
 * @brief Helper function to pop a payload from the cellular payload queue for processing. This function will check if there are any items in the queue, and if so, it will acquire the mutex to safely access the queue, retrieve the next payload, update the queue indices and count, release the mutex, and return true. If there are no items in the queue, the function will return false without modifying the output parameter.
 * @param out A pointer to a cellular_payload_t structure where the popped payload will be stored if the operation is successful. The caller should provide a valid pointer to receive the payload data.
 * @return true if a payload was successfully popped from the queue and stored in the output parameter, false if the queue is empty and no payload was retrieved.
 */
static bool cellular_payload_pop(cellular_payload_t *out);

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

	cellular_payload_queue.mutex = osMutexNew(NULL);
	cellular_payload_queue.items = osSemaphoreNew(
	CELLULAR_PAYLOAD_QUEUE_MAX_CAPACITY, 0, NULL);

	cellular_rx_sem = osSemaphoreNew(1, 0, NULL);
	cellular_disabled_sem = osSemaphoreNew(1, 0, NULL);

	cellularTaskHandler = osThreadNew(CELLULAR_Task, NULL, &cellularTaskAttr);
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

/**
 * @brief Queue a payload for later transmission to the NoteCard.
 */
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
					"{\"req\":\"note.add\",\"file\":\"data.qo\",\"sync\":true,\"body\":{");

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
			"\",\"len\":%lu}}\n", (unsigned long) len);

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
				cellular_state = CELLULAR_STATE_ERROR;
			}
			break;
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
	cellular_log("Wait for Blues Note Card to be ready for 3 seconds...");

	osDelay(3000);

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
	cellular_payload_t payload;

	// Ping the cellular module to check if it's still ON
	if (!cellular_send_cmd_and_wait_respond(
			CELLULAR_ACQUIRE_HUB_STATUS, 1000)) {
				return false;
			}

	// Wait for items
	osSemaphoreAcquire(cellular_payload_queue.items, 1000);

	if (cellular_payload_pop(&payload)) {
		cellular_log("Data is available!");

		bool ret = CELLULAR_transmit_data(payload.id, payload.payload,
				payload.len, 3000);

		return ret;
	}

	return true;
}

/**
 * @brief Park the cellular module until it is explicitly re-enabled.
 */
static bool cellular_handle_disable() {
	if (osSemaphoreAcquire(cellular_disabled_sem, 1000) == osOK) {
		cellular_state = CELLULAR_STATE_WAIT_READY;

		current_retry_count = 0;
	}

	return true;
}

/**
 * @brief Handle the error state by resetting flags, updating LEDs, and delaying retry.
 */
static bool cellular_handle_error() {
	cellular_log("AN ERROR HAS OCCURED!");

	acquired_hub_connection_status = false;

	cellular_hub_connected = false;

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

	char buf[128];

	va_list args;
	va_start(args, timeout);
	vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);

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

/**
 * @brief Remove and return the next queued payload, if one is available.
 */
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
