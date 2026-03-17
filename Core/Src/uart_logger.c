/*
 * uart_logger.c
 *
 *  Created on: Mar 11, 2026
 *      Author: buiqu
 */

#include <string.h>
#include <stdbool.h>

#include "uart_logger.h"
#include "cmsis_os.h"
#include "stm32h7xx_hal.h"

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
		.stack_size = 512 * 4, .priority = (osPriority_t) osPriorityLow };

static void uart_print(const log_msg_t *msg);

static bool uart_dma_print(const log_msg_t *msg);

static bool msg_pop(log_msg_t *out);

void UART_LOGGER_Task_Init(void) {
	uart_logger_queue.head = 0;
	uart_logger_queue.tail = 0;
	uart_logger_queue.count = 0;
	uart_logger_queue.dma_busy = false;

	uart_logger_queue.mutex = osMutexNew(NULL);
	uart_logger_queue.items = osSemaphoreNew(MSG_QUEUE_MAX_CAPACITY, 0, NULL);
	uart_logger_queue.dma_tx_done = osSemaphoreNew(1, 0, NULL);

	uartLoggerTaskHandler = osThreadNew(UART_LOGGER_Task, NULL,
			&uartLoggerTaskAttr);
}

bool uart_logger_add_msg(const char *msg, size_t len) {
	if (msg == NULL)
		return false;

	osMutexAcquire(uart_logger_queue.mutex, osWaitForever);

	if (uart_logger_queue.count >= MSG_QUEUE_MAX_CAPACITY) {
		osMutexRelease(uart_logger_queue.mutex);

		return false;
	}

	log_msg_t *slot = &uart_logger_queue.buffer[uart_logger_queue.head];

	if (len == 0)
		len = strlen(msg);

	if (len >= MSG_MAX_LEN) {
		len = MSG_MAX_LEN - 1;
	}

	slot->len = len;

	strncpy(slot->msg, msg, MSG_MAX_LEN - 1);

	slot->msg[MSG_MAX_LEN - 1] = '\0';

	uart_logger_queue.head = (uart_logger_queue.head + 1)
			% MSG_QUEUE_MAX_CAPACITY;
	uart_logger_queue.count++;

	osMutexRelease(uart_logger_queue.mutex);

	// Signal that there is item to consume
	osSemaphoreRelease(uart_logger_queue.items);

	return true;
}

void UART_LOGGER_dma_tx_cplt_callback() {
	// UART TX done transferring the last byte
	uart_logger_queue.dma_busy = false;

	osSemaphoreRelease(uart_logger_queue.dma_tx_done);
}

static void UART_LOGGER_Task(void *argument) {
	log_msg_t msg;

	while (1) {
		osSemaphoreAcquire(uart_logger_queue.items, osWaitForever);

		if (msg_pop(&msg)) {
			if (uart_dma_print(&msg)) {
				osSemaphoreAcquire(uart_logger_queue.dma_tx_done,
						osWaitForever);
			} else {
				// Call normal print instead
				uart_print(&msg);
			}
		}
	}
}

static void uart_print(const log_msg_t *msg) {
	HAL_UART_Transmit(UART_LOGGER_INSTANCE, (uint8_t*) msg->msg, msg->len,
			HAL_MAX_DELAY);
}

static bool uart_dma_print(const log_msg_t *msg) {
	if (uart_logger_queue.dma_busy) return false;

	memcpy((void*) &uart_logger_queue.current_tx_msg, (void*) msg,
			sizeof(log_msg_t));

	if (HAL_UART_Transmit_DMA(UART_LOGGER_INSTANCE,
			(uint8_t*) uart_logger_queue.current_tx_msg.msg,
			uart_logger_queue.current_tx_msg.len) == HAL_OK) {
		uart_logger_queue.dma_busy = true;

		return true;
	} else {
		uart_logger_queue.dma_busy = false;
	}

	return false;
}

static bool msg_pop(log_msg_t *out) {
	if (out == NULL)
		return false;

	osMutexAcquire(uart_logger_queue.mutex, osWaitForever);

	if (uart_logger_queue.count == 0) {
		osMutexRelease(uart_logger_queue.mutex);

		return false;
	}

	*out = uart_logger_queue.buffer[uart_logger_queue.tail];
	uart_logger_queue.tail = (uart_logger_queue.tail + 1)
			% MSG_QUEUE_MAX_CAPACITY;
	uart_logger_queue.count--;

	osMutexRelease(uart_logger_queue.mutex);

	return true;
}
