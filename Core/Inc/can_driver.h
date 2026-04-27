/*
 * can_driver.h
 *
 *  Created on: Mar 12, 2026
 *      Author: buiqu
 */

#ifndef INC_CAN_DRIVER_H_
#define INC_CAN_DRIVER_H_

#include <stdbool.h>

#include "cmsis_os.h"
#include "stm32h7xx_hal.h"

/**
 * @brief Maximum length of a CAN message payload, which should be set according to the maximum data length supported by the CAN controller and your application's requirements.
 */
#define CAN_MAX_MSG_LEN 64

/**
 * @brief Enable or disable logging of CAN operations to the UART logger.
 */
#define CAN_LOG_ENABLED 1

/**
 * @brief Structure for representing the state of the CAN driver, including the CAN handle, semaphores for synchronization, message buffers, and error state for LED indication in the controller task.
 */
typedef struct {
	// CAN handle for interfacing with the hardware
	FDCAN_HandleTypeDef* can;

	// Semaphores for synchronization between CAN driver and controller task
	osSemaphoreId_t controller_rx_sem;
	osSemaphoreId_t controller_tx_sem;
	osSemaphoreId_t can_rx_sem;

	// CAN message buffers and headers for transmission and reception
	FDCAN_TxHeaderTypeDef tx_header;
	FDCAN_RxHeaderTypeDef rx_header;

	// State variables for received message and error handling
	bool rx_ready;
	uint32_t rx_can_id;
	uint8_t rx_len;
	uint8_t rx_buf[CAN_MAX_MSG_LEN];

	// Error code for the last CAN operation, used for error handling and LED indication in the controller task
	uint32_t can_error_code;
} can_driver_state_t;

/**
 * @brief Initializes the CAN driver task and sets up the CAN interface for communication. This function will create the CAN driver task, which will handle the reception of CAN messages, transmission of CAN messages, and error handling. The function takes a pointer to the CAN driver state structure, which will be used to manage the state of the CAN interface and synchronize with the controller task.
 * @param init_state Pointer to the CAN driver state structure, which should be initialized with the appropriate CAN handle and semaphores before calling this function.
 */
void CAN_Task_Init(can_driver_state_t* init_state);

/**
 * @brief Retrieves a received CAN message from the CAN driver. This function checks if there is a new message in the RX FIFO and, if so, retrieves the message and stores it in the CAN driver state structure for processing by the controller task. The function returns true if a new message was retrieved successfully, or false if there was an error (e.g., no new message, FIFO error, etc.).
 * @param can_state Pointer to the CAN driver state structure, which will be used to access the current state of the CAN interface and manage the received message.
 * @return true if a new message was retrieved successfully, false otherwise.
 */
bool can_get_rx_message(can_driver_state_t* can_state);

/**
 * @brief Sends a CAN message with the specified payload and length. This function will prepare the CAN message header and payload, and attempt to transmit the message via the CAN interface. The function will block until the message is sent successfully or the specified timeout is reached. The function returns true if the message was sent successfully, or false if there was an error (e.g., transmission failed, timeout reached, etc.).
 * @param can_state Pointer to the CAN driver state structure, which will be used to access the current state of the CAN interface and manage the transmission state.
 * @param payload Pointer to the data buffer containing the message payload to be sent.
 * @param len Length of the message payload in bytes.
 * @return true if the message was sent successfully, false otherwise.
 */
bool can_send_message(can_driver_state_t* can_state, const uint8_t* payload, size_t len);

/**
 * @brief Callback function to be called by the CAN driver when a message is received. This function will handle the reception of a CAN message, update the CAN driver state with the received message information, and release the RX semaphore to signal the controller task that a new message has been received and is ready for processing.
 * @param can_state Pointer to the CAN driver state structure, which will be used to access the current state of the CAN interface and manage the received message.
 * @param RxFifo0ITs Parameter indicating the interrupt status for the RX FIFO 0, which can be used to determine the type of event that occurred (e.g., new message received, FIFO full, etc.) and handle it accordingly in the reception process.
 * @note This function should be called from a CAN controller interrupt handler when a new message is received, and it should manage the reception process and synchronization with the controller task via the RX semaphore.
 */
void can_rx_callback(can_driver_state_t* can_state, uint32_t RxFifo0ITs);

/**
 * @brief Callback function to be called by the CAN driver when an error occurs. This function will handle the error event, update the CAN driver state with the error information, and release the RX semaphore to signal the controller task that an error has occurred and should be handled (e.g., update error state for LED indication, log the error, etc.).
 * @param can_state Pointer to the CAN driver state structure, which will be used to access the current state of the CAN interface and manage the error state for handling in the controller task.
 * @note This function should be called from a CAN controller interrupt handler when an error occurs, and it should manage the error handling process and synchronization with the controller task via the RX semaphore.
 */
void can_error_callback(can_driver_state_t* can_state);

/**
 * @brief Callback function to be called by the CAN driver when a message transmission is completed. This function will handle the transmission completion event, update the CAN driver state as needed, and release the TX semaphore to signal the controller task that a message has been transmitted successfully and any necessary post-transmission processing can be performed (e.g., prepare for next transmission, log the event, etc.).
 * @param can_state Pointer to the CAN driver state structure, which will be used to access the current state of the CAN interface and manage the transmission state for handling in the controller task.
 * @param BufferIndexes Parameter indicating which transmit buffer(s) have completed transmission, which can be used to manage the transmit buffer state and prepare for the next message to be sent.
 * @note This function should be called from a CAN controller interrupt handler when a message transmission is completed, and it should manage the transmission completion process and synchronization with the controller task via the TX semaphore.
 */
void can_tx_callback(can_driver_state_t* can_state, uint32_t BufferIndexes);

#endif /* INC_CAN_DRIVER_H_ */
