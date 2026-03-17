/*
 * uart_log.h
 *
 *  Created on: Mar 11, 2026
 *      Author: buiqu
 */

#ifndef INC_UART_LOGGER_H_
#define INC_UART_LOGGER_H_

#include <stdbool.h>
#include <stdint.h>

#define UART_LOGGER_INSTANCE (&huart2)
#define UART_LOG_ENABLE
#define MSG_MAX_LEN 128
#define MSG_QUEUE_MAX_CAPACITY 50

bool uart_logger_add_msg(const char* msg, size_t len);

void UART_LOGGER_Task_Init(void);

void UART_LOGGER_dma_tx_cplt_callback();

#endif /* INC_UART_LOGGER_H_ */
