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

/* General Configs */
#define WIFI_ENABLE_DEBUG
#define WIFI_WAIT_READY_TIMEOUT 10000
#define WIFI_NUM_RETRIES 3
#define WIFI_PAYLOAD_TRANSMIT_RETRIES 3

/* ESP32 UART Configs */
#define WIFI_ESP32_UART_INSTANCE UART4
#define WIFI_ESP32_UART (&huart4)

/* Wifi Connection Configs */
#define WIFI_SSID "Olympus"
#define WIFI_PASSWORD "Giveme5$"
#define HOST_IP "192.168.86.100"
#define HOST_PORT 8080

#define WIFI_MAX_DATA_LEN 64
#define WIFI_PAYLOAD_QUEUE_MAX_CAPACITY 20

typedef enum {
	WIFI_STATE_RESET,
	WIFI_STATE_WAIT_READY,
	WIFI_STATE_JOIN_AP,
	WIFI_STATE_CONNECTED,
	WIFI_STATE_DISABLED,
	WIFI_STATE_ERROR,
} WifiState_t;

typedef struct {
	uint32_t last_progress;
	uint32_t wait_start;
	WifiState_t current_state;
} wifi_health_state_t;

typedef enum {
	WIFI_CAN_MESSAGE
} wifi_message_type_t;

void WIFI_Task_Init(UART_HandleTypeDef* esp32_uart_interface);

void WIFI_esp32_uart_rx_callback(size_t len);

bool WIFI_add_payload_to_queue(const wifi_message_type_t message_type, const uint32_t id, const uint8_t* data, uint16_t len);

bool WIFI_transmit_data(const char* data, size_t bytes, const uint16_t timeout);
#endif /* INC_WIFI_H_ */
