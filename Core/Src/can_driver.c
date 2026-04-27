/*
 * can_driver.c
 *
 *  Created on: Mar 12, 2026
 *      Author: buiqu
 */
#include "can_driver.h"
#include "uart_logger.h"

/**
 * @brief Macro to check if the CAN controller is in test mode based on the CCCR register value. This macro will evaluate the CCCR register to determine if the CAN controller is currently operating in test mode, which can affect how the controller behaves and how it should be managed by the driver and controller task.
 */
#define CAN_IN_TEST_MODE(cccr) (cccr & FDCAN_CCCR_TEST)

/**
 * @brief Macro to check if the CAN controller is in bus monitoring mode based on the CCCR register value. This macro will evaluate the CCCR register to determine if the CAN controller is currently operating in bus monitoring mode, which can affect how the controller behaves and how it should be managed by the driver and controller task.
 */
#define CAN_IN_BUS_MONITORING_MODE(cccr) (cccr & FDCAN_CCCR_MON)

/**
 * @brief Macro to check if the CAN controller is in test loopback mode based on the TEST register value. This macro will evaluate the TEST register to determine if the CAN controller is currently operating in test loopback mode, which can affect how the controller behaves and how it should be managed by the driver and controller task.
 */
#define CAN_IN_TEST_LOOPBACK_MODE(test) (test & FDCAN_TEST_LBCK)

/**
 * @brief CAN mode enumeration to represent the different operating modes of the CAN controller. This enumeration will be used to identify the current mode of the CAN controller, which can affect how the driver and controller task manage the communication and handle messages and errors.
 */
typedef enum {
	CAN_MODE_NORMAL,
	CAN_MODE_LOOPBACK,
	CAN_MODE_SILENT,
	CAN_MODE_SILENT_LOOPBACK,
	CAN_MODE_INVALID
} can_mode_t;

/**
 * @brief State structure for the CAN driver, which will hold the necessary information and state variables for managing the CAN communication. This structure will be used by both the CAN driver and the CAN controller task to share information about received messages, transmission status, and error conditions. The structure includes fields for the CAN handle, semaphores for synchronization, message buffers and headers for transmission and reception, state variables for received messages, and an error code for tracking the last CAN operation.
 */
static void CAN_Task(void *argument);

/**
 * @brief RTOS task handler for the CAN task, which will be used to manage the execution of the task within the RTOS environment. The handler will be initialized when the CAN task is created and will be used to control the scheduling and execution of the task.
 */
static osThreadId_t canTaskHandler;

/**
 * @brief Attributes for the CAN task, including its name, stack size, and priority.
 */
static const osThreadAttr_t canTaskAttr = { .name = "canTask", .stack_size = 256
		* 4, .priority = (osPriority_t) osPriorityRealtime };

/**
 * @brief Obtain the current operating mode of the CAN controller by analyzing the CCCR and TEST register values. This function will read the relevant registers from the CAN handle and determine the current mode of operation, which can be used by the driver and controller task to manage communication and handle messages and errors appropriately based on the mode.
 * @param can Pointer to the CAN handle structure for the CAN controller.
 * @return The current operating mode of the CAN controller as a value from the can_mode_t enumeration.
 */
static can_mode_t get_can_mode(FDCAN_HandleTypeDef *can);

/**
 * @brief Obtain CAN baud rate based on the NBTP register value. This function will read the NBTP register from the CAN handle, extract the prescaler and time segment values, and calculate the effective baud rate of the CAN communication, which can be used for logging and debugging purposes.
 * @param can Pointer to the CAN handle structure for the CAN controller.
 * @return The calculated CAN baud rate in bits per second.
 */
static uint32_t get_can_baud_rate(FDCAN_HandleTypeDef *can);

/**
 * @brief Convert the CAN Data Length Code (DLC) to the number of bytes it represents.
 * @param dlc The CAN Data Length Code.
 * @return The number of bytes corresponding to the DLC.
 */
static uint8_t get_can_dlc_to_bytes(uint32_t dlc);

/**
 * @brief Convert the number of bytes to the CAN Data Length Code (DLC).
 * @param bytes The number of bytes.
 * @return The CAN Data Length Code corresponding to the byte count.
 */
static uint32_t get_can_bytes_to_dlc(size_t bytes);

/**
 * @brief Convert CAN mode enum to a readable mode string.
 */
static const char* can_mode_to_string(can_mode_t mode);

/**
 * Logging macros (compile-time switch)
 */
#if CAN_LOG_ENABLED
#define can_log(msg) ((void) uart_logger_add_msg((msg), 0))
#define can_logf(fmt, ...) ((void) uart_logger_add_msg_format((fmt), ##__VA_ARGS__))
#else
#define can_log(msg) ((void) 0)
#define can_logf(fmt, ...) ((void) 0)
#endif

/**
 * @brief Set CAN filter to accept all messages. This function will configure the CAN filter settings to allow all incoming messages to be received by the CAN controller, which can be useful for testing and debugging purposes when you want to capture all CAN traffic without filtering.
 * @param can Pointer to the CAN handle structure for the CAN controller.
 * @return true if the filter was successfully configured, false otherwise.
 */
static bool set_can_filter_accept_all(FDCAN_HandleTypeDef *can);

/**
 * @brief Enable CAN notifications for message reception, transmission completion, and error conditions. This function will configure the CAN controller to generate interrupts for relevant events, allowing the driver and controller task to respond to incoming messages, completed transmissions, and errors in a timely manner.
 * @param can Pointer to the CAN handle structure for the CAN controller.
 * @return true if the notifications were successfully enabled, false otherwise.
 */
static bool set_can_enable_notifications(FDCAN_HandleTypeDef *can);

/**
 * @brief Initializes the CAN task and sets up the necessary state for managing CAN communication. This function will be called to start the CAN task, which will handle receiving messages, managing the activity LEDs, and processing errors. The function takes a pointer to the CAN driver state structure as an argument, which will be used to initialize the CAN driver state and allow the task to interface with the CAN hardware and manage communication effectively.
 * @param init_state Pointer to the CAN driver state structure used for initializing the CAN task.
 */
void CAN_Task_Init(can_driver_state_t *init_state) {
	if (init_state == NULL)
		return;

	init_state->rx_ready = false;
	init_state->rx_len = 0;
	init_state->rx_can_id = 0;
	init_state->can_error_code = 0;

	if (init_state->can_rx_sem == NULL) {
		init_state->can_rx_sem = osSemaphoreNew(1, 0, NULL);

		if (init_state->can_rx_sem == NULL) {
			can_log("[CAN] Failed to allocate memory for semaphore\r\n");

			return;
		}
	}

	canTaskHandler = osThreadNew(CAN_Task, init_state, &canTaskAttr);
}

/**
 * @brief Helper function to retrieve a received CAN message from the CAN controller's receive FIFO. This function will check if there are any messages in the receive FIFO, and if so, it will read the message header and payload into the CAN driver state structure. The function will also update the error code in the state structure if there is an issue with retrieving the message, allowing for error handling and logging in the controller task.
 * @param can_state Pointer to the CAN driver state structure where the received message will be stored.
 * @return true if a message was successfully retrieved, false if there are no messages or if there is an error.
 */
bool can_get_rx_message(can_driver_state_t *can_state) {
	if (can_state == NULL || can_state->can == NULL) {
		return false;
	}

	// Check if FIFO is empty
	if (HAL_FDCAN_GetRxFifoFillLevel(can_state->can, FDCAN_RX_FIFO0) == 0)
		return false;

	// Read the message from the FIFO
	HAL_StatusTypeDef ret = HAL_FDCAN_GetRxMessage(can_state->can,
	FDCAN_RX_FIFO0, &can_state->rx_header, can_state->rx_buf);

	if (ret != HAL_OK) {
		can_state->can_error_code = HAL_FDCAN_GetError(can_state->can);

		return false;
	}

	// If message is successfully read, update the state and return true
	can_state->can_error_code = HAL_FDCAN_ERROR_NONE;
	can_state->rx_can_id = can_state->rx_header.Identifier;
	can_state->rx_len = get_can_dlc_to_bytes(can_state->rx_header.DataLength);

	return true;
}

/**
 * @brief Helper function to send a CAN message with the specified payload and length. This function will use the CAN driver to send a message, and it will return immediately with a success or failure status. The function will check if there is space in the transmit FIFO before attempting to send the message, and it will update the error code in the CAN driver state structure if there is an issue with sending the message, allowing for error handling and logging in the controller task.
 * @param can_state Pointer to the CAN driver state structure used for sending the message.
 * @param payload Pointer to the data buffer containing the message payload to be sent.
 * @param len Length of the message payload in bytes.
 * @return true if the message was successfully queued for transmission, false if there is no space in the transmit FIFO or if there is an error.
 */
bool can_send_message(can_driver_state_t *can_state, const uint8_t *payload,
		size_t len) {
	if (can_state == NULL || can_state->can == NULL || payload == NULL) {
		return false;
	}

	if (HAL_FDCAN_GetTxFifoFreeLevel(can_state->can) == 0U) {
		return false;
	}

	if (len > CAN_MAX_MSG_LEN) {
		can_logf(
				"[CAN] Message payload overflowed by %lu bytes. Truncating to %d bytes\r\n",
				(unsigned long) (len - CAN_MAX_MSG_LEN), CAN_MAX_MSG_LEN);

		len = CAN_MAX_MSG_LEN;
	}

	can_state->tx_header.DataLength = get_can_bytes_to_dlc(len);

	if (HAL_FDCAN_AddMessageToTxFifoQ(can_state->can, &can_state->tx_header,
			payload) == HAL_OK) {
		can_state->can_error_code = HAL_FDCAN_ERROR_NONE;

		return true;
	}

	can_state->can_error_code = HAL_FDCAN_GetError(can_state->can);

	return false;
}

/**
 * @brief Callback function for CAN message reception. This function will be called by the CAN driver when a new message is received in the receive FIFO. The function will update the CAN driver state to indicate that a message is ready, and it will signal the CAN task to wake up and process the received message. The function will also update the error code in the state structure if there is an issue with receiving the message, allowing for error handling and logging in the controller task.
 * @param can_state Pointer to the CAN driver state structure that will be updated with the received message information.
 * @param RxFifo0ITs Bitmask indicating the interrupt status for the receive FIFO 0, which can be used to determine the type of event that occurred (e.g., new message received, FIFO full, etc.).
 * @note The function will handle the received message and release the RX semaphore to signal the controller
 */
void can_rx_callback(can_driver_state_t *can_state, uint32_t RxFifo0ITs) {
	if (can_state == NULL) {
		return;
	}

	if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) {
		can_state->rx_ready = true;
		can_state->can_error_code = HAL_FDCAN_ERROR_NONE;

		// Signal CAN task that a message has arrived, then wake it up
		osSemaphoreRelease(can_state->can_rx_sem);
	}
}

/**
 * @brief Callback function for CAN transmission completion. This function will be called by the CAN driver when a message has been successfully transmitted. The function will update the CAN driver state to indicate that the transmission is complete, and it will signal the CAN task to wake up and process the transmission completion event. The function will also update the error code in the state structure if there is an issue with the transmission, allowing for error handling and logging in the controller task.
 * @param can_state Pointer to the CAN driver state structure that will be updated with the transmission status information.
 * @param BufferIndexes Bitmask indicating which transmit buffer(s) have completed transmission, which can be used to manage the transmit buffer state and prepare for the next message to be sent.
 * @note The function will handle the transmission completion and release the TX semaphore to signal the controller
 */
void can_error_callback(can_driver_state_t *can_state) {
	if (can_state == NULL || can_state->can == NULL) {
		return;
	}

	can_state->can_error_code = HAL_FDCAN_GetError(can_state->can);

	// Wake up the controller sem, notify controller that messages arrived or an error has occurred
	if (can_state->controller_rx_sem != NULL) {
		osSemaphoreRelease(can_state->controller_rx_sem);
	}
}

/**
 * @brief Callback function for CAN error conditions. This function will be called by the CAN driver when an error is detected in the CAN communication. The function will update the CAN driver state with the current error code, and it will signal the CAN task to wake up and process the error condition. The function will also allow the controller task to manage the error condition accordingly, such as updating the error LED state and logging the error condition.
 * @param can_state Pointer to the CAN driver state structure that will be updated with the error code information.
 * @param BufferIndexes Bitmask indicating which transmit buffer(s) have completed transmission, which can be used to manage the transmit buffer state and prepare for the next message to be sent.
 * @note The function will handle the error and update the error state for LED indication and logging
 */
void can_tx_callback(can_driver_state_t *can_state, uint32_t BufferIndexes) {
	(void) BufferIndexes;

	if (can_state == NULL) {
		return;
	}

	can_state->can_error_code = HAL_FDCAN_ERROR_NONE;

	// Signal the controller tx sem that it has sent a message through CAN successfully
	if (can_state->controller_tx_sem) {
		osSemaphoreRelease(can_state->controller_tx_sem);
	}
}

/**
 * @brief RTOS task function for managing CAN communication. This function will run in a loop, waiting for events such as received messages or errors, and it will handle the processing of these events accordingly. The function will manage the state of the activity LEDs based on message reception and transmission events, and it will also handle error conditions by updating the error LED state and logging the error information. The function will use the CAN driver state structure to access information about received messages, transmission status, and error conditions, allowing it to effectively manage the CAN communication within the RTOS environment.
 * @param argument Pointer to the CAN driver state structure that will be used by the task to manage CAN communication and state.
 * @note The function will handle the main loop for the CAN task, including waiting for events, processing received messages, managing activity LEDs, and handling errors.
 */
static void CAN_Task(void *argument) {
	can_driver_state_t *can_state = (can_driver_state_t*) argument;

	if (can_state == NULL || can_state->can == NULL) {
		can_log("[CAN] CAN task init argument is invalid\r\n");
		osThreadExit();
	}

	can_mode_t can_mode = get_can_mode(can_state->can);
	can_logf("[CAN] Operating in %s mode\r\n", can_mode_to_string(can_mode));

	if (can_mode == CAN_MODE_INVALID) {
		can_log(
				"[CAN] CAN MODE INVALID -- You may forgot to set CAN mode. Please set it\r\n");
	}

	uint32_t can_baud_rate = get_can_baud_rate(can_state->can);
	can_logf("[CAN] Can baud rate: %lu bytes / second\r\n",
			(unsigned long) can_baud_rate);

	can_log("[CAN] Configuring CAN filter...\r\n");

	if (!set_can_filter_accept_all(can_state->can)) {
		can_log("[CAN] Failed to set CAN filter\r\n");
	} else {
		can_log("[CAN] Set CAN filter\r\n");
	}

	can_log("[CAN] Configuring CAN notifications...\r\n");

	if (!set_can_enable_notifications(can_state->can)) {
		can_log("[CAN] Failed to set CAN notifications\r\n");
	} else {
		can_log("[CAN] Enabled CAN notifications\r\n");
	}

	can_log("[CAN] Starting CAN...\r\n");

	if (HAL_FDCAN_Start(can_state->can) != HAL_OK) {
		can_log("[CAN] Failed to start CAN\r\n");
	} else {
		can_log("[CAN] Started CAN successfully\r\n");
	}

	while (1) {
		// Sleep until CAN received message
		osSemaphoreAcquire(can_state->can_rx_sem, osWaitForever);

		if (can_state->rx_ready) {
			can_state->rx_ready = false;

			// Wake up the controller sem, notify controller that messages arrived or an error has occurred
			if (can_state->controller_rx_sem != NULL) {
				osSemaphoreRelease(can_state->controller_rx_sem);
			}
		}
	}
}

/**
 * @brief Helper function to retrieve the current operating mode of the CAN controller by analyzing the CCCR and TEST register values. This function will read the relevant registers from the CAN handle and determine the current mode of operation, which can be used by the driver and controller task to manage communication and handle messages and errors appropriately based on the mode.
 * @param can Pointer to the CAN handle structure for the CAN controller.
 * @return The current operating mode of the CAN controller.
 */
static can_mode_t get_can_mode(FDCAN_HandleTypeDef *can) {
	// Read the CCCR and TEST registers to determine the current mode of the CAN controller
	uint32_t cccr = can->Instance->CCCR;
	uint32_t test = can->Instance->TEST;

	// Determine the CAN mode based on the CCCR and TEST register values
	if (!CAN_IN_TEST_MODE(cccr) && !CAN_IN_BUS_MONITORING_MODE(cccr)) {
		return CAN_MODE_NORMAL;
	}

	if (!CAN_IN_TEST_MODE(cccr) && CAN_IN_BUS_MONITORING_MODE(cccr)) {
		return CAN_MODE_SILENT;
	}

	if (CAN_IN_TEST_MODE(
			cccr) && CAN_IN_TEST_LOOPBACK_MODE(test) && !CAN_IN_BUS_MONITORING_MODE(cccr)) {
		return CAN_MODE_LOOPBACK;
	}

	if (CAN_IN_TEST_MODE(
			cccr) && CAN_IN_TEST_LOOPBACK_MODE(test) && CAN_IN_BUS_MONITORING_MODE(cccr)) {
		return CAN_MODE_SILENT_LOOPBACK;
	}

	return CAN_MODE_INVALID;
}

/**
 * @brief Helper function to obtain the CAN baud rate based on the NBTP register value. This function will read the NBTP register from the CAN handle, extract the prescaler and time segment values, and calculate the effective baud rate of the CAN communication, which can be used for logging and debugging purposes.
 * @param can Pointer to the CAN handle structure for the CAN controller.
 * @return The calculated CAN baud rate in bits per second.
 */
static uint32_t get_can_baud_rate(FDCAN_HandleTypeDef *can) {
	uint32_t nbtp = can->Instance->NBTP;

	uint32_t nbrp = (nbtp >> 16) & 0x1FFU;
	uint32_t ntseg1 = (nbtp >> 8) & 0xFFU;
	uint32_t ntseg2 = nbtp & 0x7FU;
//	uint32_t nsjw   = (nbtp >> 25) & 0x7FU;

	uint32_t prescaler = nbrp + 1U;
	uint32_t tseg1 = ntseg1 + 1U;
	uint32_t tseg2 = ntseg2 + 1U;
//	uint32_t sjw       = nsjw + 1U;

	uint32_t tq = 1 + tseg1 + tseg2;

	uint32_t fdcan_clk = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);

	can_logf("[CAN] prescaler: %lu | tseg1: %lu | tseg2: %lu | clock: %lu\r\n",
			(unsigned long) prescaler, (unsigned long) tseg1,
			(unsigned long) tseg2, (unsigned long) fdcan_clk);

	return fdcan_clk / (prescaler * tq);
}

/**
 * @brief Helper function to convert the CAN Data Length Code (DLC) to the number of bytes it represents. This function will take a DLC value and return the corresponding number of bytes that can be represented by that DLC, which is important for correctly interpreting the length of CAN messages based on their DLC value.
 * @param dlc The CAN Data Length Code.
 * @return The number of bytes corresponding to the DLC.
 */
static uint8_t get_can_dlc_to_bytes(uint32_t dlc) {
	switch (dlc) {
	case FDCAN_DLC_BYTES_0:
		return 0;
	case FDCAN_DLC_BYTES_1:
		return 1;
	case FDCAN_DLC_BYTES_2:
		return 2;
	case FDCAN_DLC_BYTES_3:
		return 3;
	case FDCAN_DLC_BYTES_4:
		return 4;
	case FDCAN_DLC_BYTES_5:
		return 5;
	case FDCAN_DLC_BYTES_6:
		return 6;
	case FDCAN_DLC_BYTES_7:
		return 7;
	case FDCAN_DLC_BYTES_8:
		return 8;
	case FDCAN_DLC_BYTES_12:
		return 12;
	case FDCAN_DLC_BYTES_16:
		return 16;
	case FDCAN_DLC_BYTES_20:
		return 20;
	case FDCAN_DLC_BYTES_24:
		return 24;
	case FDCAN_DLC_BYTES_32:
		return 32;
	case FDCAN_DLC_BYTES_48:
		return 48;
	case FDCAN_DLC_BYTES_64:
		return 64;
	default:
		return 0;
	}
}

/**
 * @brief Helper function to convert the number of bytes to the CAN Data Length Code (DLC). This function will take a byte count and return the corresponding DLC value that can represent that number of bytes, which is important for correctly setting the DLC field in CAN message headers when preparing messages for transmission.
 * @param bytes The number of bytes.
 * @return The CAN Data Length Code corresponding to the byte count.
 */
static uint32_t get_can_bytes_to_dlc(size_t len) {
	if (len == 0)
		return FDCAN_DLC_BYTES_0;
	if (len <= 1)
		return FDCAN_DLC_BYTES_1;
	if (len <= 2)
		return FDCAN_DLC_BYTES_2;
	if (len <= 3)
		return FDCAN_DLC_BYTES_3;
	if (len <= 4)
		return FDCAN_DLC_BYTES_4;
	if (len <= 5)
		return FDCAN_DLC_BYTES_5;
	if (len <= 6)
		return FDCAN_DLC_BYTES_6;
	if (len <= 7)
		return FDCAN_DLC_BYTES_7;
	if (len <= 8)
		return FDCAN_DLC_BYTES_8;
	if (len <= 12)
		return FDCAN_DLC_BYTES_12;
	if (len <= 16)
		return FDCAN_DLC_BYTES_16;
	if (len <= 20)
		return FDCAN_DLC_BYTES_20;
	if (len <= 24)
		return FDCAN_DLC_BYTES_24;
	if (len <= 32)
		return FDCAN_DLC_BYTES_32;
	if (len <= 48)
		return FDCAN_DLC_BYTES_48;
	return FDCAN_DLC_BYTES_64;
}

static const char* can_mode_to_string(can_mode_t mode) {
	switch (mode) {
	case CAN_MODE_NORMAL:
		return "NORMAL";
	case CAN_MODE_LOOPBACK:
		return "LOOPBACK";
	case CAN_MODE_SILENT:
		return "SILENT";
	case CAN_MODE_SILENT_LOOPBACK:
		return "SILENT LOOPBACK";
	default:
		return "INVALID";
	}
}

/**
 * @brief Helper function to set the CAN filter to accept all messages. This function will configure the CAN filter settings to allow all incoming messages to be received by the CAN controller, which can be useful for testing and debugging purposes when you want to capture all CAN traffic without filtering. The function will configure both standard and extended ID filters to accept all messages, and it will also set the global filter settings to ensure that all messages are accepted into the receive FIFO.
 * @param can Pointer to the CAN handle structure for the CAN controller.
 * @return true if the filter was successfully configured to accept all messages, false if there was an error during configuration.
 */
static bool set_can_filter_accept_all(FDCAN_HandleTypeDef *can) {
	FDCAN_FilterTypeDef sFilter = { 0 };

	sFilter.IdType = FDCAN_STANDARD_ID;
	sFilter.FilterIndex = 0;
	sFilter.FilterType = FDCAN_FILTER_MASK;
	sFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilter.FilterID1 = 0x000;
	sFilter.FilterID2 = 0x000; /* mask 0 => accept all */

	if (HAL_FDCAN_ConfigFilter(can, &sFilter) != HAL_OK) {
		return false;
	}

	sFilter.IdType = FDCAN_EXTENDED_ID;
	sFilter.FilterIndex = 1;
	sFilter.FilterType = FDCAN_FILTER_MASK;
	sFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilter.FilterID1 = 0x00000000U;
	sFilter.FilterID2 = 0x00000000U;

	if (HAL_FDCAN_ConfigFilter(can, &sFilter) != HAL_OK) {
		return false;
	}

	if (HAL_FDCAN_ConfigGlobalFilter(can,
	FDCAN_ACCEPT_IN_RX_FIFO0,
	FDCAN_ACCEPT_IN_RX_FIFO0,
	FDCAN_FILTER_REMOTE,
	FDCAN_FILTER_REMOTE) != HAL_OK) {
		return false;
	}

	return true;
}

/**
 * @brief Helper function to enable CAN notifications for message reception, transmission completion, and error conditions. This function will configure the CAN controller to generate interrupts for relevant events, allowing the driver and controller task to respond to incoming messages, completed transmissions, and errors in a timely manner. The function will enable notifications for new messages in the receive FIFO, transmission completion, bus-off conditions, and error warnings and passives, which are important events for managing CAN communication effectively.
 * @param can Pointer to the CAN handle structure for the CAN controller.
 * @return true if the notifications were successfully enabled, false if there was an error during configuration.
 */
static bool set_can_enable_notifications(FDCAN_HandleTypeDef *can) {
	return HAL_FDCAN_ActivateNotification(can, FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
	FDCAN_IT_TX_COMPLETE |
	FDCAN_IT_BUS_OFF |
	FDCAN_IT_ERROR_WARNING |
	FDCAN_IT_ERROR_PASSIVE, 0) == HAL_OK;
}
