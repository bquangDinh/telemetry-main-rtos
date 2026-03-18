/*
 * watchdog.h
 *
 *  Created on: Mar 17, 2026
 *      Author: buiqu
 */

#ifndef INC_WATCHDOG_H_
#define INC_WATCHDOG_H_

#include "stm32h7xx_hal.h"

#define WATCHDOG_INSTANCE (&hiwdg1)

void WATCHDOG_Task_Init();

#endif /* INC_WATCHDOG_H_ */
