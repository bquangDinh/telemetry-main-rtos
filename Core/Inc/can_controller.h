/*
 * can_controller.h
 *
 *  Created on: Mar 12, 2026
 *      Author: buiqu
 */

#ifndef INC_CAN_CONTROLLER_H_
#define INC_CAN_CONTROLLER_H_

#include "stm32h7xx_hal.h"

#define CAN_CONTROLLER_CAN (&hfdcan2)
#define CAN_CONTROLLER_CAN_INSTANCE FDCAN2

#define CAN_CONTROLLER_TX_LED_PORT GPIOB
#define CAN_CONTROLLER_TX_LED_PIN GPIO_PIN_1

#define CAN_CONTROLLER_RX_LED_PORT GPIOB
#define CAN_CONTROLLER_RX_LED_PIN GPIO_PIN_2

#define CAN_CONTROLLER_ERROR_LED_PORT GPIOA
#define CAN_CONTROLLER_ERROR_LED_PIN GPIO_PIN_4

#define CAN_CONTROLLER_QUEUE_MAX_CAPACITY 20

void CAN_CONTROLLER_Task_Init(FDCAN_HandleTypeDef* can);

void CAN_CONTROLLER_send_message(const uint8_t* payload, size_t len, uint16_t timeout);

void CAN_CONTROLLER_rx_callback(uint32_t RxFifo0ITs);
void CAN_CONTROLLER_tx_callback(uint32_t BufferIndexes);
void CAN_CONTROLLER_error_callback();

#endif /* INC_CAN_CONTROLLER_H_ */
