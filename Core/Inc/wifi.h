/*
 * wifi.h
 *
 *  Created on: Mar 11, 2026
 *      Author: buiqu
 */

#ifndef INC_WIFI_H_
#define INC_WIFI_H_

#include <stdbool.h>

#include "stm32h7xx_hal.h"

/**
 * @brief Enable or disable debug logging for the WiFi module. When enabled, the WiFi module will log detailed information about its operations, which can be useful for troubleshooting and development. When disabled, the WiFi module will operate without logging debug information, which can help reduce overhead and improve performance in production environments.
 */
#define WIFI_ENABLE_DEBUG

/**
 * @brief Timeout value for waiting for the WiFi module to be ready. This value specifies the maximum time (in milliseconds) to wait for the WiFi module to initialize and become ready for communication.
 */
#define WIFI_WAIT_READY_TIMEOUT 10000

/**
 * @brief Number of retries for WiFi operations before giving up and going into error state. This should be set according to the expected reliability of the WiFi connection and your application's requirements.
 */
#define WIFI_NUM_RETRIES 3

/**
 * @brief Number of retries for transmitting a payload to the WiFi module before giving up and going into error state. This should be set according to the expected reliability of the WiFi connection and your application's requirements.
 */
#define WIFI_PAYLOAD_TRANSMIT_RETRIES 3

/**
 * @brief ESP32 UART configuration values. These defines specify the UART instance and handle for communicating with the ESP32 WiFi module.
 */
#define WIFI_ESP32_UART_INSTANCE UART4
#define WIFI_ESP32_UART (&huart4)

/**
 * @brief WiFi connection configuration values. These defines specify the SSID, password, and server details for connecting to the WiFi network.
 */

#define WIFI_SSID "Olympus"
#define WIFI_PASSWORD "Giveme5$"
#define HOST_IP "192.168.86.22"
#define HOST_PORT 8080

/**
 * @brief Maximum length of data that can be sent via WiFi, which should be set according to the maximum data length supported by the WiFi module and your application's requirements.
 */
#define WIFI_MAX_DATA_LEN 64

/**
 * @brief WiFi payload queue configuration. This define specifies the maximum capacity of the queue used to store outgoing payloads that are waiting to be transmitted to the WiFi module. The queue will be used to manage the flow of outgoing data and ensure that it is transmitted in a timely manner, while also preventing overflow and managing memory usage effectively.
 */
#define WIFI_PAYLOAD_QUEUE_MAX_CAPACITY 20

/**
 * @brief Enumerated type for the states of the WiFi module. This enum defines the possible states that the WiFi module can be in, which can be used to manage the state of the WiFi connection and handle different scenarios (e.g., waiting for ready, connecting to AP, connected, error, etc.).
 */
typedef enum {
	WIFI_STATE_RESET,
	WIFI_STATE_WAIT_READY,
	WIFI_STATE_JOIN_AP,
	WIFI_STATE_CONNECTED,
	WIFI_STATE_DISABLED,
	WIFI_STATE_ERROR,
} WifiState_t;

/**
 * @brief Health state of the WiFi module, used for Watchdog monitoring and error handling. This structure contains information about the last progress made by the WiFi module, the time when it started waiting for a certain event, and the current state of the WiFi module. This information can be used by the Watchdog task to monitor the health of the WiFi module and reset the system if the WiFi module is detected to be in an error state or if it fails to report progress within the expected time frame.
 */
typedef struct {
	uint32_t last_progress;
	uint32_t wait_start;
	WifiState_t current_state;
} wifi_health_state_t;

/**
 * @brief Enumerated type for the types of messages that can be sent via WiFi. This enum defines the different types of messages that can be transmitted to the WiFi module, which can be used to manage the payloads and handle them appropriately based on their type (e.g., CAN messages, status updates, etc.).
 */
typedef enum {
	WIFI_CAN_MESSAGE
} wifi_message_type_t;

/**
 * @brief Structure for representing a WiFi payload, which includes the message type, an identifier, the length of the data, and the data itself. This structure is used to represent the payloads that are queued for transmission to the WiFi module, and it can be used to manage the different types of messages and their associated data.
 * @param esp32_uart_interface Pointer to the UART handle for communicating with the ESP32 WiFi module.
 */
void WIFI_Task_Init(UART_HandleTypeDef* esp32_uart_interface);

/**
 * @brief Callback function to be called when data is received via UART from the ESP32 WiFi module. This function will handle the reception of data from the UART interface, process it as needed, and update the state of the WiFi module accordingly. The function takes the length of the received data as a parameter, which can be used to manage the reception process and ensure that the data is handled correctly.
 * @param len Length of the data received via UART, which can be used to manage the reception process and ensure that the data is handled correctly.
 * @note This function should be called from the UART receive interrupt handler when data is received from the ESP32 WiFi module, and it should manage the reception process and update the WiFi state accordingly.
 */
void WIFI_esp32_uart_rx_callback(size_t len);

/**
 * @brief Callback function to be called when a UART transmission to the ESP32 WiFi module is complete. This function will handle the completion of a UART transmission, update the state of the WiFi module accordingly, and manage any necessary cleanup or preparation for the next transmission. The function does not take any parameters, as it is simply a notification that a transmission has completed.
 * @note This function should be called from the UART transmit complete interrupt handler when a transmission to the ESP32 WiFi module is complete, and it should manage the state of the WiFi module accordingly.
 */
void WIFI_esp32_uart_tx_callback();

/**
 * @brief Callback function to be called when a UART error occurs during communication with the ESP32 WiFi module. This function will handle any errors that occur during UART communication, update the state of the WiFi module accordingly, and manage any necessary error handling or recovery processes. The function does not take any parameters, as it is simply a notification that an error has occurred.
 * @note This function should be called from the UART error interrupt handler when an error occurs during communication with the ESP32 WiFi module, and it should manage the state of the WiFi module accordingly and perform any necessary error handling or recovery.
 */
void WIFI_esp32_uart_error_callback();

/**
 * @brief Adds a payload to the WiFi transmission queue. This function takes a message type, an identifier, a pointer to the data, and the length of the data, and adds it to the queue for later transmission to the WiFi module. The function returns true if the payload was successfully added to the queue, or false if there was an error (e.g., queue full).
 * @param message_type The type of the message being added to the queue, which can be used to manage the different types of messages and handle them appropriately when they are transmitted to the WiFi module.
 * @param id Unique identifier for the payload, which can be used to manage and track the payloads in the queue and handle them appropriately when they are transmitted to the WiFi module.
 * @param data Pointer to the data buffer containing the payload, which will be transmitted to the WiFi module when the payload is processed from the queue.
 * @param len Length of the data in bytes, which can be used to manage the payload and ensure that it is transmitted correctly to the WiFi module.
 * @return true if the payload was successfully added to the queue, false otherwise.
 */
bool WIFI_add_payload_to_queue(const wifi_message_type_t message_type, const uint32_t id, const uint8_t* data, uint16_t len);

/**
 * @brief Transmits data to the WiFi module. This function takes a pointer to the data, the length of the data, and a timeout value, and attempts to transmit the data to the WiFi module within the specified timeout period. The function returns true if the data was successfully transmitted, or false if there was an error (e.g., transmission failed, timeout reached).
 * @param data Pointer to the data buffer containing the payload to be transmitted to the WiFi module.
 * @param bytes Length of the data in bytes, which can be used to manage the transmission process and ensure that the data is transmitted correctly to the WiFi module.
 * @param timeout Timeout value in milliseconds, which specifies the maximum time to wait for the transmission to complete before considering it a failure and returning false.
 * @return true if the data was successfully transmitted, false otherwise.
 */
bool WIFI_transmit_data(const char* data, size_t bytes, const uint16_t timeout);
#endif /* INC_WIFI_H_ */
