/*
 * uart_logger.c
 *
 *  Created on: Mar 11, 2026
 *      Author: buiqu
 */

#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>

#include "uart_logger.h"
#include "cmsis_os.h"
#include "stm32h7xx_hal.h"

#if USE_USB_INTERFACE
#include "usbd_cdc_if.h"
#endif

typedef struct {
	char msg[MSG_MAX_LEN];
	uint16_t len;
} log_msg_t;

typedef struct {
	log_msg_t buffer[MSG_QUEUE_MAX_CAPACITY];
	uint16_t head;
	uint16_t tail;
	uint16_t count;
	osMutexId_t mutex;
	osSemaphoreId_t items;
	osSemaphoreId_t dma_tx_done;
	volatile bool dma_busy;
	log_msg_t current_tx_msg;
} logger_t;

static logger_t uart_logger_queue;

extern UART_HandleTypeDef huart2;

static void UART_LOGGER_Task(void *argument);

static osThreadId_t uartLoggerTaskHandler;

static const osThreadAttr_t uartLoggerTaskAttr = { .name = "uartLoggerTask",
		.stack_size = 1024 * 4, .priority = (osPriority_t) osPriorityLow };
		
/* Maximum time to wait for a DMA transfer completion callback */
#define UART_LOGGER_DMA_TX_TIMEOUT_MS 1000U

#if USE_UART_INTERFACE
static void uart_print(const log_msg_t *msg);

static bool uart_dma_print(const log_msg_t *msg);

static void uart_print_str(const char *str);
#endif

#if USE_USB_INTERFACE
static void usb_print(const log_msg_t *msg);

static void usb_print_str(const char* str);
#endif

#if USE_USB_INTERFACE
#define print(log_msg) usb_print(log_msg)
#define print_str(log_str) usb_print_str(log_str)
#define print_dma(log_msg) ((bool) 0)
#elif USE_UART_INTERFACE
#define print(log_msg) uart_print(log_msg)
#define print_str(log_str) uart_print_str(log_str)
#define print_dma(log_msg) uart_dma_print(log_msg)
#else
#define print(log_msg) ((void)0)
#define print_str(log_str) ((void)0)
#define print_dma(log_msg) ((void)0)
#endif

static bool msg_pop(log_msg_t *out);

void UART_LOGGER_Task_Init(void) {
	uart_logger_queue.head = 0;
	uart_logger_queue.tail = 0;
	uart_logger_queue.count = 0;
	uart_logger_queue.dma_busy = false;

	uart_logger_queue.mutex = osMutexNew(NULL);
	if (uart_logger_queue.mutex == NULL) {
		return;  // Mutex creation failed
	}

	uart_logger_queue.items = osSemaphoreNew(MSG_QUEUE_MAX_CAPACITY, 0, NULL);
	if (uart_logger_queue.items == NULL) {
		osMutexDelete(uart_logger_queue.mutex);
		return;  // Semaphore creation failed
	}

	uart_logger_queue.dma_tx_done = osSemaphoreNew(1, 0, NULL);
	if (uart_logger_queue.dma_tx_done == NULL) {
		osMutexDelete(uart_logger_queue.mutex);
		osSemaphoreDelete(uart_logger_queue.items);
		return;  // Semaphore creation failed
	}

	uartLoggerTaskHandler = osThreadNew(UART_LOGGER_Task, NULL,
			&uartLoggerTaskAttr);
}

bool uart_logger_add_msg(const char *msg, size_t len) {
#if LOGGER_ENABLE
	if (msg == NULL) {
		return false;
	}

	// Auto-detect length if not provided
	if (len == 0) {
		len = strlen(msg);
	}

	// Truncate if message exceeds max length
	if (len > MSG_MAX_LEN - 1) {
		len = MSG_MAX_LEN - 1;
	}

	osMutexAcquire(uart_logger_queue.mutex, osWaitForever);

	// Check if queue is full
	if (uart_logger_queue.count >= MSG_QUEUE_MAX_CAPACITY) {
		osMutexRelease(uart_logger_queue.mutex);
		return false;
	}

	// Get the head slot and populate it
	log_msg_t *slot = &uart_logger_queue.buffer[uart_logger_queue.head];
	memcpy(slot->msg, msg, len);
	slot->msg[len] = '\0';  // Ensure null termination
	slot->len = len;

	// Update queue state
	uart_logger_queue.head = (uart_logger_queue.head + 1) % MSG_QUEUE_MAX_CAPACITY;
	uart_logger_queue.count++;

	osMutexRelease(uart_logger_queue.mutex);

	// Signal that there is an item to consume
	osSemaphoreRelease(uart_logger_queue.items);
#endif

	return true;
}

bool uart_logger_add_msg_format(const char *fmt, ...) {
#if LOGGER_ENABLE
	if (fmt == NULL) {
		return false;
	}

	char msg[128];
	va_list args;

	va_start(args, fmt);
	int len = vsnprintf(msg, sizeof(msg), fmt, args);
	va_end(args);

	// Check for formatting errors or buffer overflow
	if (len < 0 || len >= (int)sizeof(msg)) {
		return false;
	}

	return uart_logger_add_msg(msg, (size_t)len);
#else
	(void)fmt;
	return true;
#endif
}

void UART_LOGGER_dma_tx_cplt_callback() {
#if USE_UART_INTERFACE
	// UART TX done transferring the last byte
	uart_logger_queue.dma_busy = false;

	osSemaphoreRelease(uart_logger_queue.dma_tx_done);
#endif
}

static void UART_LOGGER_Task(void *argument) {
	(void)argument;  // Unused parameter

	// Print startup banner
	print_str("\r\n------------------------\r\n");
	print_str("       RSC 2026         \r\n");
	print_str("------------------------\r\n");
	print_str("  Telemetry Main Board  \r\n");
	print_str("       (v1.1.0)         \r\n");
	print_str("------------------------\r\n");
	print_str("   Quang. was here :))) \r\n");
	print_str("-----------S2-----------\r\n\r\n");

	log_msg_t msg;

	// Main logging loop
	while (1) {
		// Wait for a message in the queue
		osSemaphoreAcquire(uart_logger_queue.items, osWaitForever);

		// Retrieve the message
		if (!msg_pop(&msg)) {
			continue;  // Skip if queue is empty
		}

		// Attempt DMA transmission
		if (print_dma(&msg)) {
			// Wait for DMA to complete
			if (osSemaphoreAcquire(uart_logger_queue.dma_tx_done,
					UART_LOGGER_DMA_TX_TIMEOUT_MS) != osOK) {
				uart_logger_queue.dma_busy = false;
#if USE_UART_INTERFACE
				(void) HAL_UART_AbortTransmit(UART_LOGGER_INSTANCE);
#endif
			}
		} else {
			// Fall back to blocking transmission
			print(&msg);
		}
	}
}

#if USE_UART_INTERFACE
static void uart_print(const log_msg_t *msg) {
	if (msg == NULL) {
		return;
	}

	HAL_UART_Transmit(UART_LOGGER_INSTANCE, (uint8_t *)msg->msg, msg->len,
			HAL_MAX_DELAY);
}

static void uart_print_str(const char *str) {
	if (str == NULL) {
		return;
	}

	HAL_UART_Transmit(UART_LOGGER_INSTANCE, (uint8_t *)str, strlen(str),
			HAL_MAX_DELAY);
}

static bool uart_dma_print(const log_msg_t *msg) {
	if (msg == NULL || uart_logger_queue.dma_busy) {
		return false;
	}

	// Copy message to DMA buffer
	uart_logger_queue.current_tx_msg = *msg;

	// Initiate DMA transmission
	if (HAL_UART_Transmit_DMA(UART_LOGGER_INSTANCE,
			(uint8_t *)uart_logger_queue.current_tx_msg.msg,
			uart_logger_queue.current_tx_msg.len) == HAL_OK) {
		uart_logger_queue.dma_busy = true;
		return true;
	}

	return false;
}
#endif

#if USE_USB_INTERFACE
static void usb_print(const log_msg_t *msg) {
	if (msg == NULL) return;

	CDC_Transmit_HS((uint8_t*)msg->msg, msg->len);
}

static void usb_print_str(const char* str) {
	if (str == NULL) return;

	CDC_Transmit_HS((uint8_t*)str, strlen(str));
}
#endif

static bool msg_pop(log_msg_t *out) {
	if (out == NULL) {
		return false;
	}

	osMutexAcquire(uart_logger_queue.mutex, osWaitForever);

	// Check if queue is empty
	if (uart_logger_queue.count == 0) {
		osMutexRelease(uart_logger_queue.mutex);
		return false;
	}

	// Retrieve message from tail
	*out = uart_logger_queue.buffer[uart_logger_queue.tail];

	// Update queue state
	uart_logger_queue.tail = (uart_logger_queue.tail + 1) % MSG_QUEUE_MAX_CAPACITY;
	uart_logger_queue.count--;

	osMutexRelease(uart_logger_queue.mutex);

	return true;
}
