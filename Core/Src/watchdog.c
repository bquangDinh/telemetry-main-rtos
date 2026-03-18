/*
 * watchdog.c
 *
 *  Created on: Mar 17, 2026
 *      Author: buiqu
 */

#include "watchdog.h"
#include "cmsis_os.h"

#include "wifi.h"
#include "cellular.h"

#define WATCHDOG_LED_PORT GPIOB
#define WATCHDOG_LED_PIN GPIO_PIN_5

static void WATCHDOG_Task(void *argument);

static osThreadId_t watchdogTaskHandler;

static const osThreadAttr_t watchdogTaskAttr =
		{ .name = "watchdogTask", .stack_size = 256 * 4, .priority =
				(osPriority_t) osPriorityBelowNormal };

static IWDG_HandleTypeDef *watchdog_timer = NULL;

extern wifi_health_state_t wifi_health_state;

extern cellular_health_state_t cellular_health_state;

static uint8_t is_wifi_ok();

static uint8_t is_cellular_ok();

static void blink_green_led();

void WATCHDOG_Task_Init(IWDG_HandleTypeDef* _watchdog_timer) {
	watchdog_timer = _watchdog_timer;

	watchdogTaskHandler = osThreadNew(WATCHDOG_Task, NULL, &watchdogTaskAttr);
}

static void WATCHDOG_Task(void *argument) {
	uint8_t wifi_ok = 0;
	uint8_t cellular_ok = 0;

	while (1) {
		wifi_ok = is_wifi_ok();

		cellular_ok = is_cellular_ok();

		if (wifi_ok && cellular_ok) {
			blink_green_led();

			HAL_IWDG_Refresh(watchdog_timer);
		}

		osDelay(100);
	}
}

static uint8_t is_wifi_ok() {
	uint32_t now = osKernelGetTickCount();
	uint8_t wifi_ok = 0;

	// Check Wifi module
	if (wifi_health_state.current_state == WIFI_STATE_RESET
			|| wifi_health_state.current_state == WIFI_STATE_WAIT_READY
			|| wifi_health_state.current_state == WIFI_STATE_JOIN_AP) {
		// The initial process should not take longer than 10 seconds
		if ((now - wifi_health_state.last_progress) > 10000) {
			// Wifi takes longer than expected value
			// means it could have stuck
			wifi_ok = 0;
		} else {
			// Wifi still doing ok
			wifi_ok = 1;
		}
	} else {
		if ((now - wifi_health_state.last_progress) > 1500) {
			// Wifi takes longer than expected value
			// means it could have stuck
			wifi_ok = 0;
		} else {
			// Wifi still doing ok
			wifi_ok = 1;
		}
	}

	return wifi_ok;
}

static uint8_t is_cellular_ok() {
	uint32_t now = osKernelGetTickCount();
	uint8_t cellular_ok = 0;

	// Check Wifi module
	if (cellular_health_state.current_state == CELLULAR_STATE_RESET
			|| cellular_health_state.current_state == CELLULAR_STATE_WAIT_READY
			|| cellular_health_state.current_state == CELLULAR_STATE_READY
			|| cellular_health_state.current_state == CELLULAR_STATE_HUB_CONNECTED) {
		// The initial process should not take longer than 2 minutes
		if ((now - wifi_health_state.last_progress) > 120000) {
			// Cellular takes longer than expected value
			// means it could have stuck
			cellular_ok = 0;
		} else {
			// Wifi still doing ok
			cellular_ok = 1;
		}
	} else {
		if ((now - cellular_health_state.last_progress) > 1500) {
			// Cellular takes longer than expected value
			// means it could have stuck
			cellular_ok = 0;
		} else {
			// Wifi still doing ok
			cellular_ok = 1;
		}
	}

	return cellular_ok;
}

static void blink_green_led() {
	HAL_GPIO_WritePin(WATCHDOG_LED_PORT, WATCHDOG_LED_PIN, GPIO_PIN_SET);

	osDelay(100);

	HAL_GPIO_WritePin(WATCHDOG_LED_PORT, WATCHDOG_LED_PIN, GPIO_PIN_RESET);

	osDelay(100);

	HAL_GPIO_WritePin(WATCHDOG_LED_PORT, WATCHDOG_LED_PIN, GPIO_PIN_SET);
}
