/*
 * can_controller.h
 *
 *  Created on: Mar 12, 2026
 *      Author: buiqu
 */

#ifndef INC_CAN_CONTROLLER_H_
#define INC_CAN_CONTROLLER_H_

#include "stm32h7xx_hal.h"

/**
 * @brief CAN controller definitions, including the CAN instance used for the controller, LED definitions for CAN activity indication, and CAN message queue configuration.
 * The CAN controller is responsible for managing the CAN interface, including sending and receiving CAN messages, and handling errors. The controller will use the specified CAN instance for communication, and it will use the defined LEDs to indicate CAN activity (e.g., TX, RX, error). The controller will also manage a message queue for incoming CAN messages, which can be processed by the controller task.
 */
#define CAN_CONTROLLER_CAN (&hfdcan2)
#define CAN_CONTROLLER_CAN_INSTANCE FDCAN2

/**
 * @brief LED definitions for CAN activity indication. These definitions specify the GPIO ports and pins used for the TX, RX, and error LEDs, which will be controlled by the CAN controller task to indicate the status of CAN communication (e.g., turning on the TX LED when a message is being transmitted, turning on the RX LED when a message is received, and turning on the error LED when an error occurs).
 */
#define CAN_CONTROLLER_TX_LED_PORT GPIOB
#define CAN_CONTROLLER_TX_LED_PIN GPIO_PIN_1

#define CAN_CONTROLLER_RX_LED_PORT GPIOB
#define CAN_CONTROLLER_RX_LED_PIN GPIO_PIN_2

#define CAN_CONTROLLER_ERROR_LED_PORT GPIOA
#define CAN_CONTROLLER_ERROR_LED_PIN GPIO_PIN_4

/**
 * @brief Enable or disable logging of CAN messages and events to the UART logger. When enabled, the CAN controller will log detailed information about received messages, transmitted messages, and errors to the UART logger, which can be useful for troubleshooting and development. When disabled, the CAN controller will operate without logging this information, which can help reduce overhead and improve performance in production environments.
 */
#define CAN_LOG_ENABLED 1

/**
 * @brief CAN message queue configuration for the CAN controller. This definition specifies the maximum capacity of the queue used to store incoming CAN messages before they are processed by the controller task. The queue will be used to manage the flow of incoming messages and ensure that they are processed in a timely manner, while also preventing overflow and managing memory usage effectively.
 */
#define CAN_CONTROLLER_QUEUE_MAX_CAPACITY 20

/**
 * @brief Function prototypes for the CAN controller, including initialization, message sending, and callback functions for CAN events (e.g., message reception, transmission completion, error handling). These functions will be implemented in the corresponding source file (can_controller.c) and will provide the necessary interfaces for managing the CAN communication and integrating with the CAN driver and other components of the system.
 * @param can Pointer to the CAN handle used for initializing the CAN controller task.
 */
void CAN_CONTROLLER_Task_Init(FDCAN_HandleTypeDef* can);

/**
 * @brief Sends a CAN message with the specified payload and length.
 * @param payload Pointer to the data buffer containing the message payload to be sent.
 * @param len Length of the message payload in bytes.
 * @param timeout Timeout value for the transmission attempt.
 * @note The function will block until the message is sent or the timeout is reached.
 */
void CAN_CONTROLLER_send_message(const uint8_t* payload, size_t len, uint16_t timeout);

/**
 * @brief Callback function to be called by the CAN driver when a message is received.
 * @param RxFifo0ITs Parameter indicating the interrupt status for the RX FIFO 0, which can be used to determine the type of event that occurred (e.g., new message received, FIFO full, etc.).
 * @note The function will handle the received message and release the RX semaphore.
 */
void CAN_CONTROLLER_rx_callback(uint32_t RxFifo0ITs);

/**
 * @brief Callback function to be called by the CAN driver when a message is transmitted.
 * @param BufferIndexes Parameter indicating which transmit buffer(s) have completed transmission, which can be used to manage the transmit buffer state and prepare for the next message to be sent.
 * @note The function will handle the transmission completion and release the TX semaphore.
 */
void CAN_CONTROLLER_tx_callback(uint32_t BufferIndexes);

/**
 * @brief Callback function to be called by the CAN driver when an error occurs.
 * @note The function will handle the error and update the error state for LED indication.
 */
void CAN_CONTROLLER_error_callback();

#endif /* INC_CAN_CONTROLLER_H_ */
