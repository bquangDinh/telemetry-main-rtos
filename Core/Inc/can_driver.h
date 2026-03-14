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

#define CAN_MAX_MSG_LEN 64

typedef struct {
	FDCAN_HandleTypeDef* can;

	osSemaphoreId_t controller_rx_sem;
	osSemaphoreId_t controller_tx_sem;
	osSemaphoreId_t can_rx_sem;

	FDCAN_TxHeaderTypeDef tx_header;

	FDCAN_RxHeaderTypeDef rx_header;
	bool rx_ready;
	uint32_t rx_can_id;
	uint8_t rx_len;
	uint8_t rx_buf[CAN_MAX_MSG_LEN];

	uint32_t can_error_code;
} can_driver_state_t;

void CAN_Task_Init(can_driver_state_t* init_state);

bool can_get_rx_message(can_driver_state_t* can_state);
bool can_send_message(can_driver_state_t* can_state, const uint8_t* payload, size_t len);

void can_rx_callback(can_driver_state_t* can_state, uint32_t RxFifo0ITs);
void can_error_callback(can_driver_state_t* can_state);
void can_tx_callback(can_driver_state_t* can_state, uint32_t BufferIndexes);

#endif /* INC_CAN_DRIVER_H_ */
