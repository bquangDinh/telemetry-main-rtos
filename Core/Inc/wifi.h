/*
 * wifi.h
 *
 *  Created on: Mar 11, 2026
 *      Author: buiqu
 */

#ifndef INC_WIFI_H_
#define INC_WIFI_H_

#include "stm32h7xx_hal.h"

/* General Configs */
#define WIFI_ENABLE_DEBUG

/* ESP32 UART Configs */
#define WIFI_ESP32_UART_INSTANCE UART4
#define WIFI_ESP32_UART (&huart4)

/* Wifi Connection Configs */
#define WIFI_SSID "Olympus"
#define WIFI_PASSWORD "Giveme5$"
#define HOST_IP "192.168.86.100"
#define HOST_PORT 8080

void WIFI_Task_Init(UART_HandleTypeDef* esp32_uart_interface);

void WIFI_esp32_uart_rx_callback(size_t len);

bool wifi_transmit_data(const void* data, size_t bytes);
#endif /* INC_WIFI_H_ */
