/*
 * cellular.h
 *
 *  Created on: Mar 11, 2026
 *      Author: buiqu
 */

#ifndef INC_CELLULAR_H_
#define INC_CELLULAR_H_

#include <stdbool.h>
#include "stm32h7xx_hal.h"

/*
 * @brief Number of retries for cellular operations before giving up and going into error state. This should be set according to the expected reliability of the cellular connection and your application's requirements.
 */
#define CELLULAR_NUM_RETRIES 3

/* BLUES NOTE CARD UART Configs */

/*
 * @brief Product ID for the Blue Note Card
 * This should be set to the product ID of your NoteCard, which can be found in the NoteCard's dashboard on the Blues website. The product ID is used to identify your NoteCard when it connects to the Blues cloud and is required for proper operation of the cellular module.
 */
#define CELLULAR_BLUES_PRODUCT_ID "com.gmail.rusolarcarclub:rsctest"

/*
 * @brief Serial number for the Blue Note Card
 * This should be set to the serial number of your NoteCard, which can be found on the NoteCard itself or in the NoteCard's dashboard on the Blues website. The serial number is used to uniquely identify your NoteCard and is required for proper operation of the cellular module.
 */
#define CELLULAR_BLUES_SERIAL_NUMBER ""

/*
 * @brief Outbound data configuration for the Blue Note Card
 * This should be set to 1 to enable outbound data transmission, or 0 to disable it.
 * When outbound data is enabled, the cellular module will be able to send data to the NoteCard, which can then be forwarded to the Blues cloud for processing and storage. This is typically used for sending telemetry data, sensor readings, or other information from your application to the NoteCard and the Blues cloud.
 * If outbound data is disabled, the cellular module will not be able to send data to the NoteCard, and the NoteCard will not receive any data from your application. This may be useful in certain scenarios where you only want to receive data from the NoteCard (e.g., commands, configuration updates, etc.) and do not need to send any data to the NoteCard.
 */
#define CELLULAR_BLUES_OUTBOUND 1

/*
 * @brief Inbound data configuration for the Blue Note Card
 * This should be set to 1 to enable inbound data reception, or 0 to disable it.
 * When inbound data is enabled, the cellular module will be able to receive data from the NoteCard, which can then be forwarded to the Blues cloud for processing and storage. This is typically used for receiving commands, configuration updates, or other information from the NoteCard and the Blues cloud.
 * If inbound data is disabled, the cellular module will not be able to receive data from the NoteCard, and the NoteCard will not be able to send any data to your application. This may be useful in certain scenarios where you only want to send data to the NoteCard (e.g., telemetry data, sensor readings, etc.) and do not need to receive any data from the NoteCard.
 */
#define CELLULAR_BLUES_INBOUND -1

/**
 * @brief UART instance and GPIO definitions for the Blue Note Card
 */
#define CELLULAR_BLUES_UART_INSTANCE USART3

/**
 * @brief Pointer to the UART handle for the Blue Note Card
 */
#define	CELLULAR_BLUES_UART (&huart3)

/**
 * @brief GPIO pin definition for the ATTN signal of the Blue Note Card
 * The ATTN signal is used by the NoteCard to indicate certain events (e.g., successful connection to the hub, errors, status changes, etc.) to the cellular module. The specific events that trigger the ATTN signal can be configured in the NoteCard's dashboard on the Blues website, and the cellular module should be designed to handle the relevant events and update its state accordingly when the ATTN signal is triggered.
 * In this case, it is used to indicate that the NoteCard's hub has been connected successfully, which is an important event for the cellular module to know in order to update its state and be ready for data transmission to the NoteCard.
 */
#define CELLULAR_BLUES_ATTN_GPIO_PIN GPIO_PIN_5
#define CELLULAR_BLUES_ATTN_GPIO_PORT GPIOE

/**
 * @brief Maximum data length for cellular payloads
 * This should be set according to the maximum data length supported by the NoteCard and your application's requirements.
 * @note Note that 64 bytes is used since CAN messages can only be up to 64 bytes in length, and the cellular module is designed to transmit CAN messages to the NoteCard. If your application requires transmitting larger payloads, you may need to implement a fragmentation and reassembly mechanism to split larger payloads into smaller chunks that can be transmitted within the maximum data length.
 */
#define CELLULAR_MAX_DATA_LEN 64

/**
 * @brief The maximum number of payloads that can be queued for transmission to the NoteCard
 * This should be set according to the expected data throughput and memory constraints of your application.
 */
#define CELLULAR_PAYLOAD_QUEUE_MAX_CAPACITY 20

/**
 * @brief Enable or disable cellular logging.
 */
#define CELLULAR_LOG_ENABLED 1

/**
 * @brief Enumerated type for the states of the cellular module
 */
typedef enum {
	CELLULAR_STATE_RESET,
	CELLULAR_STATE_WAIT_READY,
	CELLULAR_STATE_READY,
	CELLULAR_STATE_CONFIGURED,
	CELLULAR_STATE_HUB_CONNECTED,
	CELLULAR_STATE_READY_TO_TRANSMIT,
	CELLULAR_STATE_ERROR,
	CELLULAR_STATE_DISABLED,
} CellularState_t;

/**
 * @brief Structure for representing the health state of the NoteCard module, used for Watchdog monitoring and error handling
 */
typedef struct {
	uint32_t last_progress;
	uint32_t wait_start;
	CellularState_t current_state;
} cellular_health_state_t;

/*
 * @brief Initializes the cellular task and sets up the UART interface for communication with the Blue Note Card.
 * @param blues_uart_interface Pointer to the UART handle for the Blue Note Card.
 */
void CELLULAR_Task_Init(UART_HandleTypeDef* blues_uart_interface);

/**
 * @brief Adds a payload to the cellular transmission queue.
 * This function takes an ID, a pointer to the data, and the length of the data, and adds it to the queue for later transmission to the NoteCard. The function returns true if the payload was successfully added to the queue, or false if there was an error (e.g., queue full).
 * @param id Unique identifier for the payload.
 * @param data Pointer to the data buffer containing the payload.
 * @param len Length of the data in bytes.
 * @return true if successful, false otherwise.
 */
bool CELLULAR_add_payload_to_queue(const uint32_t id, const uint8_t* data, uint16_t len);

/**
 * @brief Transmits data to the NoteCard.
 * This function takes an ID, a pointer to the data, the length of the data, and a timeout value, and attempts to transmit the data to the NoteCard within the specified timeout period. The function returns true if the data was successfully transmitted, or false if there was an error (e.g., transmission failed, timeout reached).
 * @param id Unique identifier for the payload.
 * @param data Pointer to the data buffer containing the payload.
 * @param len Length of the data in bytes.
 * @param timeout Timeout value in milliseconds.
 * @return true if successful, false otherwise.
 */
bool CELLULAR_transmit_data(const uint32_t id, const uint8_t* data, size_t len, uint16_t timeout);

/**
 * @brief function to be called by the UART driver when data is received from the NoteCard. This function will handle the received data and process it as needed (e.g., parse responses, update state, etc.).
 * @param len Length of the received data in bytes.
 * @note This function will be called by interrupt that is called from main.c when data is received from the NoteCard
 */
void CELLULAR_blues_uart_rx_callback(size_t len);

/**
 * @brief function to be called by the UART driver when data is transmitted to the NoteCard. This function will handle the transmission completion event and perform any necessary actions (e.g., update state, signal waiting tasks, etc.).
 * @note This function will be called by interrupt that is called from main.c when data is transmitted to the NoteCard
 */
void CELLULAR_blues_uart_tx_callback();

/**
 * @brief function to be called by the UART driver when an error occurs in the UART communication with the NoteCard. This function will handle the error event and perform any necessary actions (e.g., update state, signal waiting tasks, etc.).
 * @note This function will be called by interrupt that is called from main.c when an error occurs in the UART communication with the NoteCard
 */
void CELLULAR_blues_uart_error_callback();

/**
 * @brief Callback function to be called when the ATTN signal from the NoteCard is triggered. This function will handle the ATTN event and update the cellular module's state accordingly (e.g., set cellular_hub_connected to true, update cellular_state, etc.).
 * @note The ATTN signal is typically used by the NoteCard to indicate certain events (e.g., successful connection to the hub, errors, status changes, etc.) to the cellular module. The specific events that trigger the ATTN signal can be configured in the NoteCard's dashboard on the Blues website, and the cellular module should be designed to handle the relevant events and update its state accordingly when the ATTN signal is triggered. In this case, it is used to indicate that the NoteCard's hub has been connected successfully, which is an important event for the cellular module to know in order to update its state and be ready for data transmission to the NoteCard.
 * @note This function will be called by the interrupt handler for the ATTN GPIO pin, which should be set up in main.c to trigger on the appropriate edge (e.g., rising edge) when the ATTN signal is triggered by the NoteCard.
 */
void CELLULAR_ATTN_callback();

#endif /* INC_CELLULAR_H_ */
