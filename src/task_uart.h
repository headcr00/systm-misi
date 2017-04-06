/*
 * task_uart.h
 *
 *  Created on: 16 мар. 2017 г.
 *      Author: Кочкин
 */

#ifndef TASK_UART_H_
#define TASK_UART_H_

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "queue.h"

xQueueHandle voltageBuffer;
xQueueHandle currentBuffer;

void vUart(void *pvParameters);

#endif /* TASK_UART_H_ */
