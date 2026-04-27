/*
 * watchdog.h
 *
 *  Created on: Mar 17, 2026
 *      Author: buiqu
 */

#ifndef INC_WATCHDOG_H_
#define INC_WATCHDOG_H_

#include "stm32h7xx_hal.h"

/**
 * @brief Instance of the watchdog timer used for monitoring the health of the system and resetting it in case of a failure. The watchdog timer should be configured with an appropriate timeout value based on the expected operation time of the tasks and the requirements of your application. The watchdog task will periodically check the health state of the system components (e.g., cellular module, CAN controller, SD card, etc.) and reset the system if any component is detected to be in an error state or if a component fails to report progress within the expected time frame.
 */
#define WATCHDOG_INSTANCE (&hiwdg1)

/**
 * @brief Initializes the watchdog task and sets up the watchdog timer for monitoring the health of the system. This function will create the watchdog task, which will periodically check the health state of the system components and reset the system if necessary. The function takes a pointer to the watchdog timer handle, which will be used to manage the watchdog timer and perform resets when needed.
 */
void WATCHDOG_Task_Init();

#endif /* INC_WATCHDOG_H_ */
