/*
 * esp32_driver.c
 *
 *  Created on: Mar 11, 2026
 *      Author: buiqu
 */

#include <string.h>
#include <uart_driver.h>

#include "uart_logger.h"

static void UART_Task(void *argument);

static osThreadId_t uartTaskHandler;

static const osThreadAttr_t uartTaskAttr = { .name = "uartTask", .stack_size =
		256 * 4, .priority = (osPriority_t) osPriorityNormal };

static bool uart_dma_transfer_init(uart_driver_state_t* uart_state);

static void uart_rx_process(uart_driver_state_t* uart_state);

static void uart_rx_process_bytes(uart_driver_state_t* uart_state, const uint8_t *data, size_t len);

static void uart_handle_line(uart_driver_state_t* uart_state, const uint8_t *line, size_t len);

void UART_Task_Init(uart_driver_state_t *init_state) {
	if (init_state == NULL)
		return;

	init_state->rx_size = 0;
	init_state->rx_ready = false;
	init_state->dma_old_pos = 0;

	if (init_state->uart_rx_sem == NULL) {
		init_state->uart_rx_sem = osSemaphoreNew(1, 0, NULL);

		if (init_state->uart_rx_sem == NULL) {
			uart_logger_add_msg(
					"[UART Driver] Failed to allocate memory for semaphore\r\n",
					0);

			return;
		}
	}

	uartTaskHandler = osThreadNew(UART_Task, init_state, &uartTaskAttr);
}

static void UART_Task(void *argument) {
	uart_driver_state_t* uart_state = (uart_driver_state_t*)argument;

	if (!uart_dma_transfer_init(uart_state)) {
		uart_logger_add_msg("[UART Driver] Failed to init DMA transfer\r\n", 0);
	}

	uart_state->initialized = true;

	while (1) {
		osSemaphoreAcquire(uart_state->uart_rx_sem, osWaitForever);

		if (uart_state->rx_ready) {
			uart_state->rx_ready = false;

			uart_rx_process(uart_state);
		}
	}
}

bool uart_send_data(uart_driver_state_t* uart_state, const char *data) {
	return HAL_UART_Transmit(uart_state->huart, (uint8_t*) data,
			strlen(data), HAL_MAX_DELAY) == HAL_OK;
}

void on_uart_rx_callback(uart_driver_state_t* uart_state, size_t size) {
	// Wake uart instance up
	uart_state->rx_size = size;
	uart_state->rx_ready = true;

	if (uart_state->uart_rx_sem) {
		osSemaphoreRelease(uart_state->uart_rx_sem);
	}
}

static bool uart_dma_transfer_init(uart_driver_state_t* uart_state) {
	if (HAL_UARTEx_ReceiveToIdle_DMA(uart_state->huart,
			uart_state->rx_dma_buf, UART_DMA_BUFFER_SIZE) != HAL_OK) {
		return false;
	}

	/* disable half-transfer interrupt noise */
	if (uart_state->huart->hdmarx != NULL) {
		__HAL_DMA_DISABLE_IT(uart_state->huart->hdmarx, DMA_IT_HT);
	}

	return true;
}

static void uart_rx_process(uart_driver_state_t* uart_state) {
	size_t new_pos = UART_DMA_BUFFER_SIZE
			- __HAL_DMA_GET_COUNTER(uart_state->huart->hdmarx);

	if (new_pos == uart_state->dma_old_pos) {
		// Nothing change
		return;
	}

	uint8_t* rx_dma_buf = uart_state->rx_dma_buf;

	if (new_pos > uart_state->dma_old_pos) {
		// No wrapping around
		uart_rx_process_bytes(
				uart_state,
				&rx_dma_buf[uart_state->dma_old_pos],
				new_pos - uart_state->dma_old_pos);
	} else {
		// Wrapping around
		// There are two parts
		// The first part is from old_pos to the end of the buffer
		uart_rx_process_bytes(
				uart_state,
				&rx_dma_buf[uart_state->dma_old_pos],
				UART_DMA_BUFFER_SIZE - uart_state->dma_old_pos);

		// The second part is from the beginning to new_pos
		if (new_pos > 0) {
			uart_rx_process_bytes(uart_state, &rx_dma_buf[0], new_pos);
		}
	}

	uart_state->dma_old_pos = new_pos;
}

static void uart_rx_process_bytes(uart_driver_state_t* uart_state, const uint8_t *data, size_t len) {
	// Append to line buffer
	// If '/n' is detected, it means a full respond has completed
	char ch;

	for (size_t i = 0; i < len; ++i) {
		ch = data[i];

		if (uart_state->rx_line_len < RX_LINE_MAX_LEN) {
			uart_state->rx_line_buf[uart_state->rx_line_len++] =
					ch;
		}

		if (ch == '\n') {
			uart_state->rx_line_buf[uart_state->rx_line_len] =
					'\0';

			uart_handle_line(uart_state, uart_state->rx_line_buf,
					uart_state->rx_line_len);

			uart_state->rx_line_len = 0;
		}
	}

	if (uart_state->rx_callback != NULL) {
		uart_state->rx_callback(data, len);
	}
}

static void uart_handle_line(uart_driver_state_t* uart_state, const uint8_t *line, size_t len) {
	uart_logger_add_msg((char*) line, len);

	if (uart_state->rx_line_callback != NULL) {
		uart_state->rx_line_callback(line, len);
	}
}
