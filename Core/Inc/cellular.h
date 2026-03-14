/*
 * cellular.h
 *
 *  Created on: Mar 11, 2026
 *      Author: buiqu
 */

#ifndef INC_CELLULAR_H_
#define INC_CELLULAR_H_

#include "stm32h7xx_hal.h"

/* BLUES NOTE CARD UART Configs */
#define CELLULAR_BLUES_PRODUCT_ID "com.gmail.rusolarcarclub:rsctest"
#define CELLULAR_BLUES_SERIAL_NUMBER ""

// The max wait time, in minutes, to sync outbound data from the Notecard. Explicit syncs (e.g. using hub.sync) do not affect this cadence.
// When in periodic or continuous mode this argument is required, otherwise the Notecard will function as if it is in minimum mode as it pertains to syncing behavior.
// Use -1 to reset the value back to its default of 0.
// A value of 0 means that the Notecard will never sync outbound data unless explicitly told to do so (e.g. using hub.sync).
// Same with inbound
#define CELLULAR_BLUES_OUTBOUND 1
#define CELLULAR_BLUES_INBOUND -1

#define CELLULAR_BLUES_UART_INSTANCE USART3
#define	CELLULAR_BLUES_UART (&huart3)

#define CELLULAR_BLUES_ATTN_GPIO_PIN GPIO_PIN_5
#define CELLULAR_BLUES_ATTN_GPIO_PORT GPIOE

void CELLULAR_Task_Init(UART_HandleTypeDef* blues_uart_interface);

void CELLULAR_blues_uart_rx_callback(size_t len);

void CELLULAR_ATTN_callback();

#endif /* INC_CELLULAR_H_ */
