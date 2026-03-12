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

#define UART_LOGGER_INSTANCE (&huart2)

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
} logger_t;

static logger_t uart_logger_queue;

extern UART_HandleTypeDef huart2;

static void UART_LOGGER_Task(void *argument);

static osThreadId_t uartLoggerTaskHandler;

static const osThreadAttr_t uartLoggerTaskAttr = {
		.name = "uartLoggerTask",
		.stack_size = 256 * 4,
		.priority = (osPriority_t) osPriorityLow
};

static void uart_print(const char* str);

static bool msg_pop(log_msg_t* out);

void UART_LOGGER_Task_Init(void) {
	uart_logger_queue.head = 0;
	uart_logger_queue.tail = 0;
	uart_logger_queue.count = 0;

	uart_logger_queue.mutex = osMutexNew(NULL);
	uart_logger_queue.items = osSemaphoreNew(MSG_QUEUE_MAX_CAPACITY, 0, NULL);

	uartLoggerTaskHandler = osThreadNew(UART_LOGGER_Task, NULL, &uartLoggerTaskAttr);
}

bool uart_logger_add_msg(const char* msg, size_t len) {
	if (msg == NULL) return false;

	osMutexAcquire(uart_logger_queue.mutex, osWaitForever);

	if (uart_logger_queue.count >= MSG_QUEUE_MAX_CAPACITY) {
		osMutexRelease(uart_logger_queue.mutex);

		return false;
	}

	log_msg_t* slot = &uart_logger_queue.buffer[uart_logger_queue.head];

	if (len == 0) len = strlen(msg);

	if (len >= MSG_MAX_LEN) {
		len = MSG_MAX_LEN - 1;
	}

	slot->len = len;

	strncpy(slot->msg, msg, MSG_MAX_LEN - 1);

	slot->msg[MSG_MAX_LEN - 1] = '\0';

	uart_logger_queue.head = (uart_logger_queue.head + 1) % MSG_QUEUE_MAX_CAPACITY;
	uart_logger_queue.count++;

	osMutexRelease(uart_logger_queue.mutex);

	// Signal that there is item to consume
	osSemaphoreRelease(uart_logger_queue.items);

	return true;
}

static void UART_LOGGER_Task(void *argument) {
	log_msg_t msg;

	while (1) {
		osSemaphoreAcquire(uart_logger_queue.items, osWaitForever);

		if (msg_pop(&msg)) {
			uart_print(msg.msg);
		}
	}
}

static void uart_print(const char* str) {
	HAL_UART_Transmit(UART_LOGGER_INSTANCE, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}

static bool msg_pop(log_msg_t* out) {
	if (out == NULL) return false;

	osMutexAcquire(uart_logger_queue.mutex, osWaitForever);

	if (uart_logger_queue.count == 0) {
		osMutexRelease(uart_logger_queue.mutex);

		return false;
	}

	*out = uart_logger_queue.buffer[uart_logger_queue.tail];
	uart_logger_queue.tail = (uart_logger_queue.tail + 1) % MSG_QUEUE_MAX_CAPACITY;
	uart_logger_queue.count--;

	osMutexRelease(uart_logger_queue.mutex);

	return true;
}
