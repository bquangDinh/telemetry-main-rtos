/*
 * can_driver.c
 *
 *  Created on: Mar 12, 2026
 *      Author: buiqu
 */
#include <stdio.h>

#include "can_driver.h"
#include "uart_logger.h"

#define CAN_IN_TEST_MODE(cccr) (cccr & FDCAN_CCCR_TEST)
#define CAN_IN_BUS_MONITORING_MODE(cccr) (cccr & FDCAN_CCCR_MON)
#define CAN_IN_TEST_LOOPBACK_MODE(test) (test & FDCAN_TEST_LBCK)

// The first 0-7 bits of NBTP's register value is prescaler
#define CAN_NBTP_PRESCALER(nbtp) ((nbtp & 0x1FF) + 1)

// The 16-23 bits are time segment 1
#define CAN_NBTP_TIME_SEGMENT_1(nbtp) (((nbtp >> 16) & 0xFF) + 1)

// The 8-14 bits are time segment 2
#define CAN_NBTP_TIME_SEGMENT_2(nbtp) (((nbtp >> 8) & 0x7F) + 1)

typedef enum {
	CAN_MODE_NORMAL,
	CAN_MODE_LOOPBACK,
	CAN_MODE_SILENT,
	CAN_MODE_SILENT_LOOPBACK,
	CAN_MODE_INVALID
} can_mode_t;

static void CAN_Task(void *argument);

static osThreadId_t canTaskHandler;

static const osThreadAttr_t canTaskAttr = { .name = "canTask", .stack_size = 256
		* 4, .priority = (osPriority_t) osPriorityRealtime };

static can_mode_t get_can_mode(FDCAN_HandleTypeDef *can);

static uint32_t get_can_baud_rate(FDCAN_HandleTypeDef *can);

static uint8_t get_can_dlc_to_bytes(uint32_t dlc);

static uint32_t get_can_bytes_to_dlc(size_t bytes);

static bool set_can_filter_accept_all(FDCAN_HandleTypeDef *can);

static bool set_can_enable_notifications(FDCAN_HandleTypeDef *can);

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
			uart_logger_add_msg(
					"[CAN] Failed to allocate memory for semaphore\r\n", 0);

			return;
		}
	}

	canTaskHandler = osThreadNew(CAN_Task, init_state, &canTaskAttr);
}

bool can_get_rx_message(can_driver_state_t *can_state) {
	// Check if FIFO is empty
	if (HAL_FDCAN_GetRxFifoFillLevel(can_state->can, FDCAN_RX_FIFO0) == 0)
		return false;

	HAL_StatusTypeDef ret = HAL_FDCAN_GetRxMessage(can_state->can,
	FDCAN_RX_FIFO0, &can_state->rx_header, can_state->rx_buf);

	if (ret != HAL_OK) {
		can_state->can_error_code = HAL_FDCAN_GetError(can_state->can);

		return false;
	}

	can_state->can_error_code = HAL_FDCAN_ERROR_NONE;
	can_state->rx_can_id = can_state->rx_header.Identifier;
	can_state->rx_len = get_can_dlc_to_bytes(can_state->rx_header.DataLength);

	return true;
}

bool can_send_message(can_driver_state_t *can_state, const uint8_t *payload,
		size_t len) {
	if (HAL_FDCAN_GetTxFifoFreeLevel(can_state->can) == 0U) {
		return false;
	}

	char msg[100];

	if (len > CAN_MAX_MSG_LEN) {
		sprintf(msg,
				"[CAN] Message payload is overflow of %d bytes. Maximum is %d bytes\r\n",
				len - CAN_MAX_MSG_LEN, CAN_MAX_MSG_LEN);

		uart_logger_add_msg(msg, 0);
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

void can_rx_callback(can_driver_state_t *can_state, uint32_t RxFifo0ITs) {
	if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) {
		can_state->rx_ready = true;
		can_state->can_error_code = HAL_FDCAN_ERROR_NONE;

		// Signal CAN task that a message has arrived, then wake it up
		osSemaphoreRelease(can_state->can_rx_sem);
	}
}

void can_error_callback(can_driver_state_t *can_state) {
	can_state->can_error_code = HAL_FDCAN_GetError(can_state->can);

	// Wake up the controller sem, notify controller that messages arrived or an error has occurred
	if (can_state->controller_rx_sem != NULL) {
		osSemaphoreRelease(can_state->controller_rx_sem);
	}
}

void can_tx_callback(can_driver_state_t *can_state, uint32_t BufferIndexes) {
	can_state->can_error_code = HAL_FDCAN_ERROR_NONE;

	// Signal the controller tx sem that it has sent a message through CAN successfully
	if (can_state->controller_tx_sem) {
		osSemaphoreRelease(can_state->controller_tx_sem);
	}
}

static void CAN_Task(void *argument) {
	char msg[100];

	can_driver_state_t *can_state = (can_driver_state_t*) argument;

	can_mode_t can_mode = get_can_mode(can_state->can);

	switch (can_mode) {
	case CAN_MODE_NORMAL:
		uart_logger_add_msg("[CAN] Operating in NORMAL mode\r\n", 0);
		break;
	case CAN_MODE_SILENT:
		uart_logger_add_msg("[CAN] Operating in SILENT mode\r\n", 0);
		break;
	case CAN_MODE_LOOPBACK:
		uart_logger_add_msg("[CAN] Operating in LOOPBACK mode\r\n", 0);
		break;
	case CAN_MODE_SILENT_LOOPBACK:
		uart_logger_add_msg("[CAN] Operating in SILENT LOOPBACK \r\n", 0);
		break;
	case CAN_MODE_INVALID:
		uart_logger_add_msg(
				"[CAN] CAN MODE INVALID -- You may forgot to set CAN mode. Please set it\r\n",
				0);
		break;
	}

	uint32_t can_baud_rate = get_can_baud_rate(can_state->can);

	sprintf(msg, "[CAN] Can baud rate: %lu bytes / second\r\n", can_baud_rate);

	uart_logger_add_msg(msg, 0);

	uart_logger_add_msg("[CAN] Configuring CAN filter...\r\n", 0);

	if (!set_can_filter_accept_all(can_state->can)) {
		uart_logger_add_msg("[CAN] Failed to set CAN filter\r\n", 0);
	} else {
		uart_logger_add_msg("[CAN] Set CAN filter\r\n", 0);
	}

	uart_logger_add_msg("[CAN] Configuring CAN notifications...\r\n", 0);

	if (!set_can_enable_notifications(can_state->can)) {
		uart_logger_add_msg("[CAN] Failed to set CAN notifications\r\n", 0);
	} else {
		uart_logger_add_msg("[CAN] Enabled CAN notifications\r\n", 0);
	}

	uart_logger_add_msg("[CAN] Starting CAN...\r\n", 0);

	if (HAL_FDCAN_Start(can_state->can) != HAL_OK) {
		uart_logger_add_msg("[CAN] Failed to start CAN\r\n", 0);
	} else {
		uart_logger_add_msg("[CAN] Started CAN successfully\r\n", 0);
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

static can_mode_t get_can_mode(FDCAN_HandleTypeDef *can) {
	uint32_t cccr = can->Instance->CCCR;
	uint32_t test = can->Instance->TEST;

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

static uint32_t get_can_baud_rate(FDCAN_HandleTypeDef *can) {
	char msg[100];

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

	sprintf(msg,
			"[CAN] prescaler: %lu | tseg1: %lu | tseg2: %lu | clock: %lu\r\n",
			prescaler, tseg1, tseg2, fdcan_clk);

	uart_logger_add_msg(msg, 0);

	return fdcan_clk / (prescaler * tq);
}

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

static uint32_t get_can_bytes_to_dlc(size_t len) {
	if (len <= 0)
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

static bool set_can_enable_notifications(FDCAN_HandleTypeDef *can) {
	return HAL_FDCAN_ActivateNotification(can, FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
	FDCAN_IT_TX_COMPLETE |
	FDCAN_IT_BUS_OFF |
	FDCAN_IT_ERROR_WARNING |
	FDCAN_IT_ERROR_PASSIVE, 0) == HAL_OK;
}
