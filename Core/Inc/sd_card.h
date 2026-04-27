/*
 * sd_card.h
 *
 *  Created on: Mar 29, 2026
 *      Author: buiqu
 */

#ifndef INC_SD_CARD_H_
#define INC_SD_CARD_H_

#include "stm32h7xx_hal.h"

/**
 * @brief Instance of the SD card
 */
#define SDCARD_INSTANCE &hsd1

/**
 * @brief Number of retries for SD card operations before giving up and going into error state
 */
#define SDCARD_NUM_RETRIES 3

/**
 * @brief Maximum data length for SD card payloads, which should be set according to the maximum data length supported by the SD card and your application's requirements
 */
#define SDCARD_MAX_DATA_LEN 128

/**
 * @brief SD card payload queue configuration
 */
#define SDCARD_PAYLOAD_QUEUE_MAX_CAPACITY 10

/**
 * @brief Number of retries for writing payloads to the SD card
 */
#define SDCARD_PAYLOAD_WRITE_RETRIES 3

/**
 * @brief Enable or disable logging of SD card operations to the UART logger
 */
#define SD_LOG_ENABLED 1

/**
 * @brief Enumerates the possible states of the SD card
 */
typedef enum {
	SDCARD_STATE_RESET,
	SDCARD_STATE_MOUNT_AND_OPEN,
	SDCARD_STATE_READY,
	SDCARD_STATE_ERROR,
	SDCARD_STATE_DISABLED
} SDCardState_t;

/**
 * @brief Health state of the SD card, used for Watchdog monitoring and error handling
 */
typedef struct {
	uint32_t last_progress;
	uint32_t wait_start;
	SDCardState_t current_state;
} sdcard_health_state_t;

/**
 * @brief Initializes the SD card task with the provided SD card instance
 * @param sdcard_instance Pointer to the SD card handle
 */
void SDCARD_Task_Init(SD_HandleTypeDef* sdcard_instace);

/**
 * @brief Adds a payload to the SD card write queue
 * @param message Pointer to the data buffer containing the payload
 * @param len Length of the data in bytes
 * @return true if the payload was successfully added to the queue, false otherwise
 */
bool SDCARD_add_payload_to_queue(const char* message, const uint16_t len);

#endif /* INC_SD_CARD_H_ */
