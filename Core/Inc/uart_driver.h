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

typedef void (*uart_rx_line_callback_t)(const uint8_t* data, size_t len);

typedef struct {
	// UART instance of this UART driver
	UART_HandleTypeDef *huart;

	// Buffer that stores the data received from UART
	// DMA will store its data here whenever data arrived at UART
	// without CPU awareness
	uint8_t rx_dma_buf[UART_DMA_BUFFER_SIZE];

	// Size of the data received from UART
	volatile uint16_t rx_size;

	// Flag to check if data has arrived at UART
	volatile bool rx_ready;

	// Buffer that stores a complete message from UART (whenever /n is detected)
	uint8_t rx_line_buf[RX_LINE_MAX_LEN];

	// Size of the received message
	size_t rx_line_len;

	// Controller's semaphore
	// Used to signal the controller of this driver that message has arrived at UART driver
	osSemaphoreId_t controller_rx_sem;

	// UART's semaphore
	// Used to signal UART driver that a message has arrived
	osSemaphoreId_t uart_rx_sem;

	// Since UART driver uses DMA circular buffer
	// This variable is used to store the old read position in the circular buffer
	size_t dma_old_pos;

	// Check if UART driver has been initialized
	bool initialized;

	// The controller can provide a callback that will be called whenever a complete message arrived
	// at UART
	uart_rx_line_callback_t rx_line_callback;
} uart_driver_state_t;

void UART_Task_Init(uart_driver_state_t* init_state);

bool uart_send_data(uart_driver_state_t* uart_state, const char *data);

void on_uart_rx_callback(uart_driver_state_t* uart_state, size_t size);
#endif /* INC_UART_DRIVER_H_ */
