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
#include "semphr.h"

void uart_init(void);
void SendDMAUART(char * sendbuffer, uint8_t len);
uint8_t TXUARTReady();
QueueHandle_t TXDMAQueue;
#endif /* TASK_UART_H_ */
