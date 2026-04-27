/*
 * can_controller.c
 *
 *  Created on: Mar 12, 2026
 *      Author: buiqu
 */
#include <string.h>
#include <stdio.h>

#include "can_controller.h"
#include "can_driver.h"
#include "uart_logger.h"

#include "wifi.h"
#include "cellular.h"

/**
 * @brief Size of the log buffer for CAN messages
 */
#define CAN_LOG_BUFFER_SIZE 256U

/**
 * @brief Maximum number of bytes to show in the pretty-printed CAN message payload. If the payload is longer than this value, only the first few bytes will be shown in the log output, which can help improve readability while still providing useful information about the message content.
 */
#define CAN_PRETTY_PAYLOAD_MAX_BYTES 8U

/**
 * @brief CAN controller task function, which will be responsible for managing the CAN communication, including receiving messages, handling errors, and controlling the activity LEDs. The task will wait for messages to arrive (signaled by the RX semaphore), process the received messages, and handle any errors that occur during CAN communication. The task will also manage the state of the TX and RX LEDs based on the activity of the CAN interface.
 */
static void CAN_CONTROLLER_Task(void *argument);

/**
 * @brief RTOS task handler for the CAN controller task, which will be used to manage the execution of the task within the RTOS environment. The handler will be initialized when the CAN controller task is created and will be used to control the scheduling and execution of the task.
 */
static osThreadId_t canControllerTaskHandler;

/**
 * @brief Semaphores for synchronization between the CAN driver and the CAN controller task. The RX semaphore will be released by the CAN driver when a message is received, allowing the controller task to process the incoming message. The TX semaphore will be released by the CAN driver when a message has been successfully transmitted, allowing the controller task to manage the state of the TX LED and log the transmission event.
 */
static osSemaphoreId_t can_controller_rx_sem;
static osSemaphoreId_t can_controller_tx_sem;

static const osThreadAttr_t canControllerTaskAttr = { .name =
		"canControllerTask", .stack_size = 256 * 4, .priority =
		(osPriority_t) osPriorityRealtime };

/**
 * @brief State structure for the CAN driver, which will hold the necessary information and state variables for managing the CAN communication. This structure will be used by both the CAN driver and the CAN controller task to share information about received messages, transmission status, and error conditions. The structure includes fields for the CAN handle, semaphores for synchronization, message buffers and headers for transmission and reception, state variables for received messages, and an error code for tracking the last CAN operation.
 */
static can_driver_state_t can_driver_state = { 0 };

/**
 * @brief Helper function to format a CAN message into a human-readable string for logging purposes. This function will take the raw CAN message data and length, and format it into a string that includes the length of the message and the payload bytes in hexadecimal format. The formatted string will be used for logging received messages in a clear and concise manner.
 */
static void format_pretty_CAN_message(char *buf, size_t buf_len,
		const uint8_t *data, uint8_t len);

/**
 * @brief Helper function to fan out the received CAN payload to other components of the system, such as the cellular and WiFi modules. This function will take the received CAN message and add it to the respective queues for the cellular and WiFi modules, allowing those components to process the incoming CAN data as needed.
 */
static void fan_out_received_can_payload(void);

/**
 * @brief Helper function to determine the number of blinks for the error LED based on the CAN error code. This function will analyze the error code and return a corresponding number of blinks that can be used to indicate the type of error that occurred, allowing for visual indication of different error conditions through the error LED.
 */
static uint8_t can_get_error_blink_counts(uint32_t err);

/**
 * @brief Helper function to manage the error LED behavior based on the current CAN error code. This function will control the blinking pattern of the error LED to indicate different types of errors, providing a visual indication of the error condition that can be observed without needing to check logs or other outputs.
 */
static void can_error_led_task(void);

#if CAN_LOG_ENABLED
static void can_logln_impl(const char *msg);
static void can_log_raw_impl(const char *msg);

#define can_logln(msg)              can_logln_impl(msg)
#define can_log_raw(msg)            can_log_raw_impl(msg)
#else
#define can_logln(msg)              ((void)0)
#define can_log_raw(msg)            ((void)0)
#endif

/**
 * @brief Initializes the CAN controller task and sets up the necessary state for managing CAN communication. This function will be called to start the CAN controller task, which will handle receiving messages, managing the activity LEDs, and processing errors. The function takes a pointer to the CAN handle as an argument, which will be used to initialize the CAN driver state and allow the controller task to interface with the CAN hardware.
 * @param can Pointer to the CAN handle used for initializing the CAN controller task.
 */
void CAN_CONTROLLER_Task_Init(FDCAN_HandleTypeDef *can) {
	can_driver_state.can = can;

	canControllerTaskHandler = osThreadNew(CAN_CONTROLLER_Task, NULL,
			&canControllerTaskAttr);
}

/**
 * @brief Callback function to be called by the CAN driver when a message is received. This function will handle the reception of a CAN message, update the state of the RX LED, and release the RX semaphore to signal the controller task that a new message has arrived. The function takes a parameter indicating the interrupt status for the RX FIFO 0, which can be used to determine the type of event that occurred (e.g., new message received, FIFO full, etc.).
 * @param RxFifo0ITs Parameter indicating the interrupt status for the RX FIFO 0, which can be used to determine the type of event that occurred (e.g., new message received, FIFO full, etc.).
 * @note The function will handle the received message and release the RX semaphore.
 */
void CAN_CONTROLLER_rx_callback(uint32_t RxFifo0ITs) {
	// Turn RX LED on
	HAL_GPIO_WritePin(CAN_CONTROLLER_RX_LED_PORT, CAN_CONTROLLER_RX_LED_PIN, GPIO_PIN_SET);

	can_rx_callback(&can_driver_state, RxFifo0ITs);
}

/**
 * @brief Callback function to be called by the CAN driver when a message is transmitted. This function will handle the completion of a CAN message transmission, update the state of the TX LED, and release the TX semaphore to signal the controller task that the transmission has completed. The function takes a parameter indicating which transmit buffer(s) have completed transmission, which can be used to manage the transmit buffer state and prepare for the next message to be sent.
 * @param BufferIndexes Parameter indicating which transmit buffer(s) have completed transmission, which can be used to manage the transmit buffer state and prepare for the next message to be sent.
 * @note The function will handle the transmission completion and release the TX semaphore.
 */
void CAN_CONTROLLER_tx_callback(uint32_t BufferIndexes) {
	// Turn TX LED on
	HAL_GPIO_WritePin(CAN_CONTROLLER_TX_LED_PORT, CAN_CONTROLLER_TX_LED_PIN, GPIO_PIN_SET);

	can_tx_callback(&can_driver_state, BufferIndexes);
}

/**
 * @brief Callback function to be called by the CAN driver when an error occurs. This function will handle the occurrence of a CAN error, update the error state for LED indication, and allow the controller task to manage the error condition accordingly. The function will be called when an error is detected in the CAN communication, and it will update the error code in the CAN driver state, which can then be used by the controller task to control the error LED and log the error condition.
 * @note The function will handle the error and update the error state for LED indication.
 */
void CAN_CONTROLLER_error_callback() {
	can_error_callback(&can_driver_state);
}

/**
 * @brief Sends a CAN message with the specified payload and length. This function will use the CAN driver to send a message, and it will block until the message is sent or the specified timeout is reached. The function will also manage the state of the TX LED and log the transmission event. If the message is successfully sent, the TX LED will be turned on, and a log message will indicate that the message was sent. If the transmission fails or times out, a log message will indicate the failure.
 * @param payload Pointer to the data buffer containing the message payload to be sent.
 * @param len Length of the message payload in bytes.
 * @param timeout Timeout value for the transmission attempt.
 */
void CAN_CONTROLLER_send_message(const uint8_t* payload, size_t len, uint16_t timeout) {
	if (!can_send_message(&can_driver_state, payload, len)) {
		can_logln("Failed to send CAN message");
		return;
	}

	// Wait until the message is sent
	if (osSemaphoreAcquire(can_controller_tx_sem, timeout) == osOK) {
		can_logln("Sent CAN message");

		// Turn off TX LED
		HAL_GPIO_WritePin(CAN_CONTROLLER_TX_LED_PORT, CAN_CONTROLLER_TX_LED_PIN, GPIO_PIN_RESET);
	} else {
		can_logln("Failed to send CAN message -- Timeout");
	}
}

/**
 * @brief CAN controller task function, which will be responsible for managing the CAN communication, including receiving messages, handling errors, and controlling the activity LEDs. The task will wait for messages to arrive (signaled by the RX semaphore), process the received messages, and handle any errors that occur during CAN communication. The task will also manage the state of the TX and RX LEDs based on the activity of the CAN interface.
 * @param argument Pointer to any arguments that may be needed for the task (not used in this implementation).
 */
static void CAN_CONTROLLER_Task(void *argument) {
	char msg[CAN_LOG_BUFFER_SIZE];

	can_logln("Initializing CAN driver...");

	can_controller_rx_sem = osSemaphoreNew(1, 0, NULL);
	can_controller_tx_sem = osSemaphoreNew(1, 0, NULL);

	can_driver_state.controller_rx_sem = can_controller_rx_sem;
	can_driver_state.controller_tx_sem = can_controller_tx_sem;

	CAN_Task_Init(&can_driver_state);

	can_logln("Initialized CAN driver...");

	while (1) {
		// Waiting for messages to arrive
		// rx sem can also be released by can driver in case of an error happened
		osSemaphoreAcquire(can_controller_rx_sem, osWaitForever);

		// In case of an error has occurred
		can_error_led_task();

		if (can_driver_state.can_error_code == HAL_FDCAN_ERROR_NONE) {
			// Message arrived
			while (can_get_rx_message(&can_driver_state)) {
				memset(msg, 0, sizeof(msg));

				format_pretty_CAN_message(msg, sizeof(msg), can_driver_state.rx_buf,
						can_driver_state.rx_len);

				can_log_raw(msg);

				fan_out_received_can_payload();
			}

			// Turn off RX LED
			HAL_GPIO_WritePin(CAN_CONTROLLER_RX_LED_PORT, CAN_CONTROLLER_RX_LED_PIN, GPIO_PIN_RESET);
		}

		osDelay(100);
	}
}

/**
 * @brief Helper function to format a CAN message into a human-readable string for logging purposes. This function will take the raw CAN message data and length, and format it into a string that includes the length of the message and the payload bytes in hexadecimal format. The formatted string will be used for logging received messages in a clear and concise manner.
 * @param buf Pointer to the buffer where the formatted string will be stored.
 * @param buf_len Length of the buffer.
 * @param data Pointer to the raw CAN message data.
 * @param len Length of the CAN message data.
 */
static void format_pretty_CAN_message(char *buf, size_t buf_len,
		const uint8_t *data, uint8_t len) {
	int offset = 0;

	offset += snprintf(buf + offset, buf_len - (size_t) offset,
			"len: %u | ", len);

	if (offset < 0 || (size_t) offset >= buf_len) {
		if (buf_len > 0) {
			buf[buf_len - 1] = '\0';
		}
		return;
	}

	int show = (len < CAN_PRETTY_PAYLOAD_MAX_BYTES) ? len : CAN_PRETTY_PAYLOAD_MAX_BYTES;

	for (int i = 0; i < show; ++i) {
		offset += snprintf(buf + offset, buf_len - (size_t) offset, "%02X ",
				data[i]);

		if (offset < 0 || (size_t) offset >= buf_len) {
			if (buf_len > 0) {
				buf[buf_len - 1] = '\0';
			}
			return;
		}
	}

	(void) snprintf(buf + offset, buf_len - (size_t) offset, "\r\n");
}

/**
 * @brief Helper function to fan out the received CAN payload to other components of the system, such as the cellular and WiFi modules. This function will take the received CAN message and add it to the respective queues for the cellular and WiFi modules, allowing those components to process the incoming CAN data as needed.
 */
static void fan_out_received_can_payload(void) {
	if (CELLULAR_add_payload_to_queue(can_driver_state.rx_can_id,
			can_driver_state.rx_buf, can_driver_state.rx_len)) {
		can_logln("Added payload to cellular queue");
	}

	if (WIFI_add_payload_to_queue(WIFI_CAN_MESSAGE, can_driver_state.rx_can_id,
			can_driver_state.rx_buf, can_driver_state.rx_len)) {
		can_logln("Added payload to wifi queue");
	}
}

#if CAN_LOG_ENABLED
/**
 * @brief Log a message to the UART logger with a newline. This function will format the log message with a consistent prefix to indicate that it is related to the CAN controller, and it will add a newline at the end of the message for better readability in the log output.
 * @param msg Pointer to the message string to be logged.
 */
static void can_logln_impl(const char *msg) {
	uart_logger_add_msg_format("[CAN CRL] %s\r\n", msg);
}

/**
 * @brief Log a raw message to the UART logger without adding a newline. This function will format the log message with a consistent prefix to indicate that it is related to the CAN controller, but it will not add a newline at the end of the message, allowing for more flexible logging of messages that may already include their own formatting or newlines.
 * @param msg Pointer to the message string to be logged.
 */
static void can_log_raw_impl(const char *msg) {
	uart_logger_add_msg_format("[CAN CRL] %s", msg);
}
#endif

/**
 * @brief Helper function to determine the number of blinks for the error LED based on the CAN error code. This function will analyze the error code and return a corresponding number of blinks that can be used to indicate the type of error that occurred, allowing for visual indication of different error conditions through the error LED.
 * @param err The CAN error code to be analyzed.
 * @return The number of blinks for the error LED.
 */
static uint8_t can_get_error_blink_counts(uint32_t err) {
	if (err & HAL_FDCAN_ERROR_TIMEOUT)
		return 1;
	if (err & HAL_FDCAN_ERROR_NOT_INITIALIZED)
		return 2;
	if (err & HAL_FDCAN_ERROR_NOT_READY)
		return 3;
	if (err & HAL_FDCAN_ERROR_PARAM)
		return 4;
#ifdef HAL_FDCAN_ERROR_FIFO_FULL
	if (err & HAL_FDCAN_ERROR_FIFO_FULL)
		return 5;
#endif
	return 0;
}

/**
 * @brief Helper function to manage the error LED behavior based on the current CAN error code. This function will control the blinking pattern of the error LED to indicate different types of errors, providing a visual indication of the error condition that can be observed without needing to check logs or other outputs.
 */
static void can_error_led_task(void) {
	static uint32_t last_tick = 0;
	static uint8_t blink_count = 0;
	static uint8_t current_blink = 0;
	static uint8_t led_state = 0;
	static uint8_t in_pause = 0;

	uint32_t can_error_code = can_driver_state.can_error_code;

	uint32_t now = HAL_GetTick();

	// Handle FIFO full error with a fast blink pattern
#ifdef HAL_FDCAN_ERROR_FIFO_FULL
	if (can_error_code & HAL_FDCAN_ERROR_FIFO_FULL) {
		if ((now - last_tick) > 100U) {
			last_tick = now;
			HAL_GPIO_TogglePin(CAN_CONTROLLER_ERROR_LED_PORT,
					CAN_CONTROLLER_ERROR_LED_PIN);
		}
		return;
	}
#endif

	// If no error, reset the LED state and counters
	if (can_error_code == HAL_FDCAN_ERROR_NONE) {
		HAL_GPIO_WritePin(CAN_CONTROLLER_ERROR_LED_PORT,
				CAN_CONTROLLER_ERROR_LED_PIN, GPIO_PIN_RESET);
		blink_count = 0;
		current_blink = 0;
		led_state = 0;
		in_pause = 0;
		return;
	}

	// If in pause between blink cycles, check if it's time to start the next cycle
	if (in_pause) {
		if ((now - last_tick) > 800U) {
			in_pause = 0;
			current_blink = 0;
			led_state = 0;
			blink_count = can_get_error_blink_counts(can_error_code);
			last_tick = now;
		}
		return;
	}

	// If not currently blinking, determine the number of blinks for the current error code
	if (blink_count == 0) {
		blink_count = can_get_error_blink_counts(can_error_code);
		current_blink = 0;
		led_state = 0;
		last_tick = now;
	}

	// If there is no error to indicate, ensure the LED is off and return
	if (blink_count == 0) {
		HAL_GPIO_WritePin(CAN_CONTROLLER_ERROR_LED_PORT,
				CAN_CONTROLLER_ERROR_LED_PIN, GPIO_PIN_RESET);
		return;
	}

	// Handle the blinking pattern for the error LED based on the current blink count and state
	if ((now - last_tick) > 180U) {
		last_tick = now;

		if (led_state == 0) {
			HAL_GPIO_WritePin(CAN_CONTROLLER_ERROR_LED_PORT,
					CAN_CONTROLLER_ERROR_LED_PIN, GPIO_PIN_SET);
			led_state = 1;
		} else {
			HAL_GPIO_WritePin(CAN_CONTROLLER_ERROR_LED_PORT,
					CAN_CONTROLLER_ERROR_LED_PIN, GPIO_PIN_RESET);
			led_state = 0;
			current_blink++;

			if (current_blink >= blink_count) {
				current_blink = 0;
				blink_count = 0;
				in_pause = 1;
			}
		}
	}
}
