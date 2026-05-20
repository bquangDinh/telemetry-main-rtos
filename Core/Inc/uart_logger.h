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

#define USE_USB_INTERFACE 0
#define USE_UART_INTERFACE 1

#if USE_UART_INTERFACE
/**
 * @brief Instance of the UART logger
 */
#define UART_LOGGER_INSTANCE (&huart2)
#endif

/**
 * @brief Enable logging
 */
#define LOGGER_ENABLE 1

/**
 * @brief Maximum length of a log message
 */
#define MSG_MAX_LEN 256

/**
 * @brief Maximum capacity of the log message queue
 */
#define MSG_QUEUE_MAX_CAPACITY 100

/**
 * @brief Adds a log message to the queue
 * @param msg Pointer to the log message
 * @param len Length of the log message
 * @return true if the message was added successfully, false otherwise
 */
bool uart_logger_add_msg(const char* msg, size_t len);

/**
 * @brief Adds a formatted log message to the queue
 * @param fmt Format string for the log message
 * @param ... Arguments for the format string
 * @return true if the message was added successfully, false otherwise
 */
bool uart_logger_add_msg_format(const char *fmt, ...);

/**
 * @brief Initializes the UART logger task
 */
void UART_LOGGER_Task_Init(void);

/**
 * @brief Callback function called when DMA transmission is complete
 */
void UART_LOGGER_dma_tx_cplt_callback();

#endif /* INC_UART_LOGGER_H_ */
