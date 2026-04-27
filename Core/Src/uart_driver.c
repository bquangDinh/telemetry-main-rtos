/*
 * uart_driver.c
 *
 *  Created on: Mar 11, 2026
 *      Author: buiqu
 */

#include <string.h>
#include <uart_driver.h>

#include "uart_logger.h"

/* Log message constants */
#define LOG_PREFIX "[UART Driver] "
#define LOG_SEM_ALLOC_FAIL LOG_PREFIX "Failed to allocate memory for semaphore\r\n"
#define LOG_DMA_INIT_FAIL LOG_PREFIX "Failed to init DMA transfer\r\n"
#define LOG_ERR_PARITY LOG_PREFIX "Parity err\r\n"
#define LOG_ERR_NOISE LOG_PREFIX "Noise err\r\n"
#define LOG_ERR_FRAMING LOG_PREFIX "Framing err\r\n"
#define LOG_ERR_OVERRUN LOG_PREFIX "Overrun err\r\n"
#define LOG_ERR_DMA LOG_PREFIX "DMA err\r\n"
#define LOG_ERR_TIMEOUT LOG_PREFIX "Timeout err\r\n"
#define LOG_RESETTING LOG_PREFIX "Reseting UART driver...\r\n"
#define LOG_RESET_FAIL LOG_PREFIX "Failed to reset UART\r\n"
#define LOG_RESET_OK LOG_PREFIX "Reset UART\r\n"

/* Forward declarations */
static void UART_Task(void *argument);
static bool uart_dma_transfer_init(uart_driver_state_t *uart_state);
static void uart_rx_process(uart_driver_state_t *uart_state);
static void uart_rx_process_bytes(uart_driver_state_t *uart_state,
		const uint8_t *data, size_t len);
static void uart_handle_line(uart_driver_state_t *uart_state,
		const uint8_t *line, size_t len);
static void uart_handle_error(uart_driver_state_t *uart_state);
static void uart_process_wrapped_buffer(uart_driver_state_t *uart_state,
		size_t new_pos);
static void uart_log_error_type(uint32_t err);

/* Static variables */
static osThreadId_t uartTaskHandler;
static const osThreadAttr_t uartTaskAttr = {
	.name = "uartTask",
	.stack_size = 256 * 4,
	.priority = (osPriority_t) osPriorityNormal
};

/**
 * @brief Initializes the UART receive task
 * @param init_state Pointer to the UART driver state structure
 */
void UART_Task_Init(uart_driver_state_t *init_state) {
	if (init_state == NULL) {
		return;
	}

	/* Initialize state variables */
	init_state->rx_size = 0;
	init_state->rx_ready = false;
	init_state->dma_old_pos = 0;

	/* Create semaphore if not already created */
	if (init_state->uart_rx_sem == NULL) {
		init_state->uart_rx_sem = osSemaphoreNew(1, 0, NULL);

		if (init_state->uart_rx_sem == NULL) {
			uart_logger_add_msg(LOG_SEM_ALLOC_FAIL, 0);
			return;
		}
	}

	/* Create the UART task */
	uartTaskHandler = osThreadNew(UART_Task, init_state, &uartTaskAttr);
}

/**
 * @brief Main UART task that handles receiving and processing UART data
 * @param argument Pointer to the uart_driver_state_t structure
 */
static void UART_Task(void *argument) {
	uart_driver_state_t *uart_state = (uart_driver_state_t*) argument;

	if (uart_state == NULL) {
		return;
	}

	/* Initialize DMA transfer for receiving data */
	if (!uart_dma_transfer_init(uart_state)) {
		uart_logger_add_msg(LOG_DMA_INIT_FAIL, 0);
		return;
	}

	uart_state->initialized = true;

	/* Main task loop - waits for data or error events */
	while (1) {
		/* Wait for semaphore release (triggered by RX or error callback) */
		osSemaphoreAcquire(uart_state->uart_rx_sem, osWaitForever);

		/* Handle any UART errors */
		if (uart_state->uart_err != 0) {
			uart_handle_error(uart_state);
		}

		/* Process received data if ready */
		if (uart_state->rx_ready) {
			uart_state->rx_ready = false;
			uart_rx_process(uart_state);
		}
	}
}

/**
 * @brief Sends null-terminated string via UART
 * @param uart_state Pointer to the UART driver state structure
 * @param data Pointer to the null-terminated string to be sent
 * @return true if data was sent successfully, false otherwise
 */
bool uart_send_data(uart_driver_state_t *uart_state, const char *data) {
	if (uart_state == NULL || data == NULL) {
		return false;
	}

	return HAL_UART_Transmit(uart_state->huart, (uint8_t*) data, strlen(data),
			HAL_MAX_DELAY) == HAL_OK;
}

/**
 * @brief Sends data with specified length via UART
 * @param uart_state Pointer to the UART driver state structure
 * @param data Pointer to the data to be sent
 * @param len Length of the data to be sent
 * @return true if data was sent successfully, false otherwise
 */
bool uart_send_data_w_len(uart_driver_state_t *uart_state, const char *data,
		size_t len) {
	if (uart_state == NULL || data == NULL) {
		return false;
	}

	return HAL_UART_Transmit(uart_state->huart, (uint8_t*) data, len,
			HAL_MAX_DELAY) == HAL_OK;
}

/**
 * @brief Callback function called when UART receives data
 * Signals the UART task to process the received data
 * @param uart_state Pointer to the UART driver state structure
 * @param size Size of the received data
 */
void on_uart_rx_callback(uart_driver_state_t *uart_state, size_t size) {
	if (uart_state == NULL) {
		return;
	}

	/* Mark data as ready for processing */
	uart_state->rx_size = size;
	uart_state->rx_ready = true;

	/* Signal the UART task */
	if (uart_state->uart_rx_sem != NULL) {
		osSemaphoreRelease(uart_state->uart_rx_sem);
	}
}

/**
 * @brief Callback function called when UART transmission completes
 * @param uart_state Pointer to the UART driver state structure
 */
void on_uart_tx_callback(uart_driver_state_t *uart_state) {
	/* Currently no action needed for TX complete */
	(void) uart_state;
}

/**
 * @brief Callback function called when a UART error occurs
 * Signals the UART task to handle the error
 * @param uart_state Pointer to the UART driver state structure
 */
void on_uart_err_callback(uart_driver_state_t *uart_state) {
	if (uart_state == NULL) {
		return;
	}

	/* Capture the error code */
	uart_state->uart_err = HAL_UART_GetError(uart_state->huart);

	/* Signal the UART task to handle the error */
	if (uart_state->uart_rx_sem != NULL) {
		osSemaphoreRelease(uart_state->uart_rx_sem);
	}
}

/**
 * @brief Initializes DMA circular buffer reception for UART
 * @param uart_state Pointer to the UART driver state structure
 * @return true if initialization was successful, false otherwise
 */
static bool uart_dma_transfer_init(uart_driver_state_t *uart_state) {
	if (uart_state == NULL || uart_state->huart == NULL) {
		return false;
	}

	/* Start DMA receive in idle mode (transfer complete on idle line detection) */
	if (HAL_UARTEx_ReceiveToIdle_DMA(uart_state->huart, uart_state->rx_dma_buf,
			UART_DMA_BUFFER_SIZE) != HAL_OK) {
		return false;
	}

	/* Disable half-transfer interrupt to reduce noise */
	if (uart_state->huart->hdmarx != NULL) {
		__HAL_DMA_DISABLE_IT(uart_state->huart->hdmarx, DMA_IT_HT);
	}

	return true;
}

/**
 * @brief Processes received data from the DMA circular buffer
 * Detects buffer wrapping and calls uart_rx_process_bytes for each segment
 * @param uart_state Pointer to the UART driver state structure
 */
static void uart_rx_process(uart_driver_state_t *uart_state) {
	if (uart_state == NULL || uart_state->huart == NULL) {
		return;
	}

	/* Calculate current position in DMA circular buffer */
	size_t new_pos = UART_DMA_BUFFER_SIZE
			- __HAL_DMA_GET_COUNTER(uart_state->huart->hdmarx);

	/* No new data since last processing */
	if (new_pos == uart_state->dma_old_pos) {
		return;
	}

	uint8_t *rx_dma_buf = uart_state->rx_dma_buf;

	if (new_pos > uart_state->dma_old_pos) {
		/* No buffer wrap - continuous data segment */
		uart_rx_process_bytes(uart_state, &rx_dma_buf[uart_state->dma_old_pos],
				new_pos - uart_state->dma_old_pos);
	} else {
		/* Buffer wrapped around - process two segments */
		uart_process_wrapped_buffer(uart_state, new_pos);
	}

	uart_state->dma_old_pos = new_pos;
}

/**
 * @brief Helper function to process wrapped circular buffer
 * Processes data in two segments: from old_pos to end, and from start to new_pos
 * @param uart_state Pointer to the UART driver state structure
 * @param new_pos Current position in the buffer after wrapping
 */
static void uart_process_wrapped_buffer(uart_driver_state_t *uart_state,
		size_t new_pos) {
	uint8_t *rx_dma_buf = uart_state->rx_dma_buf;

	/* Process first segment: from old_pos to end of buffer */
	uart_rx_process_bytes(uart_state, &rx_dma_buf[uart_state->dma_old_pos],
			UART_DMA_BUFFER_SIZE - uart_state->dma_old_pos);

	/* Process second segment: from start to new_pos */
	if (new_pos > 0) {
		uart_rx_process_bytes(uart_state, &rx_dma_buf[0], new_pos);
	}
}

/**
 * @brief Processes received bytes, detects line endings, and calls handlers
 * Accumulates bytes into a line buffer until newline is detected
 * @param uart_state Pointer to the UART driver state structure
 * @param data Pointer to the received data bytes
 * @param len Number of bytes to process
 */
static void uart_rx_process_bytes(uart_driver_state_t *uart_state,
		const uint8_t *data, size_t len) {
	if (uart_state == NULL || data == NULL) {
		return;
	}

	/* Process each byte and accumulate into line buffer */
	for (size_t i = 0; i < len; ++i) {
		char ch = (char) data[i];

		/* Add byte to line buffer if there's space */
		if (uart_state->rx_line_len < RX_LINE_MAX_LEN) {
			uart_state->rx_line_buf[uart_state->rx_line_len++] = ch;
		}

		/* Process complete line when newline is detected */
		if (ch == '\n') {
			uart_state->rx_line_buf[uart_state->rx_line_len] = '\0';
			uart_handle_line(uart_state, uart_state->rx_line_buf,
					uart_state->rx_line_len);
			uart_state->rx_line_len = 0;
		}
	}

	/* Call raw data callback if registered */
	if (uart_state->rx_callback != NULL) {
		uart_state->rx_callback(data, len);
	}
}

/**
 * @brief Handles a complete line of data received via UART
 * Logs the line and calls the registered line callback
 * @param uart_state Pointer to the UART driver state structure
 * @param line Pointer to the received line data
 * @param len Length of the line
 */
static void uart_handle_line(uart_driver_state_t *uart_state,
		const uint8_t *line, size_t len) {
	if (uart_state == NULL || line == NULL) {
		return;
	}

	/* Log the received line */
	uart_logger_add_msg((char*) line, len);

	/* Call the registered line handler callback if present */
	if (uart_state->rx_line_callback != NULL) {
		uart_state->rx_line_callback(line, len);
	}
}

/**
 * @brief Logs specific UART error types based on error flags
 * @param err Error code from HAL_UART_GetError()
 */
static void uart_log_error_type(uint32_t err) {
	if (err & HAL_UART_ERROR_PE) {
		uart_logger_add_msg(LOG_ERR_PARITY, 0);
	}
	if (err & HAL_UART_ERROR_NE) {
		uart_logger_add_msg(LOG_ERR_NOISE, 0);
	}
	if (err & HAL_UART_ERROR_FE) {
		uart_logger_add_msg(LOG_ERR_FRAMING, 0);
	}
	if (err & HAL_UART_ERROR_ORE) {
		uart_logger_add_msg(LOG_ERR_OVERRUN, 0);
	}
	if (err & HAL_UART_ERROR_DMA) {
		uart_logger_add_msg(LOG_ERR_DMA, 0);
	}
#if defined(HAL_UART_ERROR_RTO)
	if (err & HAL_UART_ERROR_RTO) {
		uart_logger_add_msg(LOG_ERR_TIMEOUT, 0);
	}
#endif
}

/**
 * @brief Handles UART errors by clearing flags and resetting the DMA transfer
 * @param uart_state Pointer to the UART driver state structure
 */
static void uart_handle_error(uart_driver_state_t *uart_state) {
	if (uart_state == NULL || uart_state->huart == NULL) {
		return;
	}

	uint32_t err = uart_state->uart_err;

	/* Log the specific error types detected */
	uart_log_error_type(err);

	/* Log reset action */
	uart_logger_add_msg(LOG_RESETTING, 0);

	/* Clear error flags */
	__HAL_UART_CLEAR_FEFLAG(uart_state->huart);
	__HAL_UART_CLEAR_NEFLAG(uart_state->huart);
	__HAL_UART_CLEAR_OREFLAG(uart_state->huart);

	/* Stop any ongoing UART receive operation */
	if (HAL_UART_AbortReceive(uart_state->huart) != HAL_OK) {
		uart_logger_add_msg(LOG_RESET_FAIL, 0);
		return;
	}

	/* Reinitialize DMA transfer */
	if (!uart_dma_transfer_init(uart_state)) {
		uart_logger_add_msg(LOG_RESET_FAIL, 0);
		return;
	}

	/* Clear error state */
	uart_state->uart_err = 0;

	/* Log successful reset */
	uart_logger_add_msg(LOG_RESET_OK, 0);
}
