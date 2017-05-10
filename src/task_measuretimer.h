/*
 * tak_measuretimer.h
 *
 *  Created on: 24 марта 2017 г.
 *      Author: Кочкин
 */

#ifndef TASK_MEASURETIMER_H_
#define TASK_MEASURETIMER_H_

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "semphr.h"
void vMeasureTimer(void * pvParameters);
void initMeasure();
xSemaphoreHandle  xMeasureToggle;

#endif /* TASK_MEASURETIMER_H_ */
