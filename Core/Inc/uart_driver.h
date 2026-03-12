/*
 * esp32_driver.h
 *
 *  Created on: Mar 11, 2026
 *      Author: buiqu
 */

#ifndef INC_UART_DRIVER_H_
#define INC_UART_DRIVER_H_

#include <stdbool.h>

#include "cmsis_os.h"
#include "stm32h7xx_hal.h"

#define UART_DMA_BUFFER_SIZE 512
#define RX_LINE_MAX_LEN 128

typedef struct {
	UART_HandleTypeDef *huart;

	uint8_t rx_dma_buf[UART_DMA_BUFFER_SIZE];

	volatile uint16_t rx_size;
	volatile bool rx_ready;

	uint8_t rx_line_buf[RX_LINE_MAX_LEN];
	size_t rx_line_len;

	osSemaphoreId_t controller_rx_sem;
	osSemaphoreId_t uart_rx_sem;

	size_t dma_old_pos;

	bool initialized;
} uart_driver_state_t;

void UART_Task_Init(uart_driver_state_t* init_state);

bool uart_send_data(uart_driver_state_t* uart_state, const char *data);

void on_uart_rx_callback(uart_driver_state_t* uart_state, size_t size);
#endif /* INC_UART_DRIVER_H_ */
