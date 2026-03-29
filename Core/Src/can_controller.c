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

static void CAN_CONTROLLER_Task(void *argument);

static osThreadId_t canControllerTaskHandler;

static osSemaphoreId_t can_controller_rx_sem;
static osSemaphoreId_t can_controller_tx_sem;

static const osThreadAttr_t canControllerTaskAttr = { .name =
		"canControllerTask", .stack_size = 256 * 4, .priority =
		(osPriority_t) osPriorityRealtime };

static can_driver_state_t can_driver_state = { 0 };

static void format_pretty_CAN_message(char *buf, uint8_t *data, uint8_t len);

static uint8_t can_get_error_blink_counts(uint32_t err);

static void can_error_led_task(void);

void CAN_CONTROLLER_Task_Init(FDCAN_HandleTypeDef *can) {
	can_driver_state.can = can;

	canControllerTaskHandler = osThreadNew(CAN_CONTROLLER_Task, NULL,
			&canControllerTaskAttr);
}

void CAN_CONTROLLER_rx_callback(uint32_t RxFifo0ITs) {
	// Turn RX LED on
	HAL_GPIO_WritePin(CAN_CONTROLLER_RX_LED_PORT, CAN_CONTROLLER_RX_LED_PIN, GPIO_PIN_SET);

	can_rx_callback(&can_driver_state, RxFifo0ITs);
}

void CAN_CONTROLLER_tx_callback(uint32_t BufferIndexes) {
	// Turn TX LED on
	HAL_GPIO_WritePin(CAN_CONTROLLER_TX_LED_PORT, CAN_CONTROLLER_TX_LED_PIN, GPIO_PIN_SET);

	can_tx_callback(&can_driver_state, BufferIndexes);
}

void CAN_CONTROLLER_error_callback() {
	can_error_callback(&can_driver_state);
}

void CAN_CONTROLLER_send_message(const uint8_t* payload, size_t len, uint16_t timeout) {
	if (!can_send_message(&can_driver_state, payload, len)) {
		uart_logger_add_msg("[CAN CONTROLLER] Failed to send CAN message\r\n", 0);
	}

	// Wait until the message is sent
	if (osSemaphoreAcquire(can_controller_tx_sem, timeout) == osOK) {
		uart_logger_add_msg("[CAN CONTROLLER] Sent CAN message\r\n", 0);

		// Turn off TX LED
		HAL_GPIO_WritePin(CAN_CONTROLLER_TX_LED_PORT, CAN_CONTROLLER_TX_LED_PIN, GPIO_PIN_RESET);
	} else {
		uart_logger_add_msg("[CAN CONTROLLER] Failed to send CAN message -- Timeout\r\n", 0);
	}
}

static void CAN_CONTROLLER_Task(void *argument) {
	char msg[256];

	uart_logger_add_msg("[CAN CONTROLLER] Initializing CAN driver...\r\n", 0);

	can_controller_rx_sem = osSemaphoreNew(1, 0, NULL);
	can_controller_tx_sem = osSemaphoreNew(1, 0, NULL);

	can_driver_state.controller_rx_sem = can_controller_rx_sem;
	can_driver_state.controller_tx_sem = can_controller_tx_sem;

	CAN_Task_Init(&can_driver_state);

	uart_logger_add_msg("[CAN CONTROLLER] Initialized CAN driver...\r\n", 0);

	while (1) {
		// Waiting for messages to arrive
		// rx sem can also be released by can driver in case of an error happened
		osSemaphoreAcquire(can_controller_rx_sem, osWaitForever);

		// In case of an error has occurred
		can_error_led_task();

		if (can_driver_state.can_error_code == HAL_FDCAN_ERROR_NONE) {
			// Message arrived
			while (can_get_rx_message(&can_driver_state)) {
				memset(msg, 0, 256);

				format_pretty_CAN_message(msg, can_driver_state.rx_buf,
						can_driver_state.rx_len);

				uart_logger_add_msg(msg, 0);

				// Sending message to Cellular and Wifi
				if (CELLULAR_add_payload_to_queue(can_driver_state.rx_can_id, can_driver_state.rx_buf, can_driver_state.rx_len)) {
					uart_logger_add_msg("[CAN CONTROLLER] Added payload to cellular queue\r\n", 0);
				}

				if (WIFI_add_payload_to_queue(WIFI_CAN_MESSAGE, can_driver_state.rx_can_id, can_driver_state.rx_buf, can_driver_state.rx_len)) {
					uart_logger_add_msg("[CAN CONTROLLER] Added payload to wifi queue\r\n", 0);
				}
			}

			// Turn off RX LED
			HAL_GPIO_WritePin(CAN_CONTROLLER_RX_LED_PORT, CAN_CONTROLLER_RX_LED_PIN, GPIO_PIN_RESET);
		}

		osDelay(100);
	}
}

static void format_pretty_CAN_message(char *buf, uint8_t *data, uint8_t len) {
	int offset = 0;

	offset += sprintf(buf + offset, "[CAN CONTROLLER] len: %u | ", len);

	int show = (len < 8) ? len : 8;

	for (int i = 0; i < show; ++i) {
		offset += sprintf(buf + offset, "%02X ", data[i]);
	}

	sprintf(buf + offset, "\r\n");
}

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

static void can_error_led_task(void) {
	static uint32_t last_tick = 0;
	static uint8_t blink_count = 0;
	static uint8_t current_blink = 0;
	static uint8_t led_state = 0;
	static uint8_t in_pause = 0;

	uint32_t can_error_code = can_driver_state.can_error_code;

	uint32_t now = HAL_GetTick();

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

	if (can_error_code == HAL_FDCAN_ERROR_NONE) {
		HAL_GPIO_WritePin(CAN_CONTROLLER_ERROR_LED_PORT,
				CAN_CONTROLLER_ERROR_LED_PIN, GPIO_PIN_RESET);
		blink_count = 0;
		current_blink = 0;
		led_state = 0;
		in_pause = 0;
		return;
	}

	if (blink_count == 0) {
		blink_count = can_get_error_blink_counts(can_error_code);
		current_blink = 0;
		led_state = 0;
		in_pause = 0;
		last_tick = now;
	}

	if (blink_count == 0) {
		HAL_GPIO_WritePin(CAN_CONTROLLER_ERROR_LED_PORT,
				CAN_CONTROLLER_ERROR_LED_PIN, GPIO_PIN_RESET);
		return;
	}

	if (in_pause) {
		if ((now - last_tick) > 800U) {
			in_pause = 0;
			current_blink = 0;
			led_state = 0;
			last_tick = now;
		}
		return;
	}

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
