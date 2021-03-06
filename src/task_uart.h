/*
 * task_uart.h
 *
 *  Created on: 16 ���. 2017 �.
 *      Author: ������
 */

#ifndef TASK_UART_H_
#define TASK_UART_H_

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
void send_to_uart(char * buffer, uint16_t len);
void uart_init(void);
void SendDMAUART(char * sendbuffer, uint8_t len);
void vUartTask(void *pvParameters);
uint8_t TXUARTReady();
QueueHandle_t TXDMAQueue;
#endif /* TASK_UART_H_ */
