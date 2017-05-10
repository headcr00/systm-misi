/*
 * task_uart.c
 *
 *  Created on: 16 мар. 2017 г.
 *      Author: Кочкин
 */

#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "task_uart.h"
#include "stdio.h"
#include "semphr.h"
#include "stdlib.h"
#include "queue.h"
#define UART_RXDR 0x40004804
#define UART_TXDR 0x40004804
char rxbuffer[12] = {};
uint8_t UARTTXBusyFlag = 0;

QueueHandle_t sendqueue;


void uart_init(void) {
	GPIO_InitTypeDef gpio;
	USART_InitTypeDef usart;
	DMA_InitTypeDef dma1;
	NVIC_InitTypeDef nvic;

	RCC_APB2PeriphClockCmd(
	RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	//USART TX
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOC, &gpio);

	//USART RX
	gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOC, &gpio);
	GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);

	usart.USART_BaudRate = 256000;
	usart.USART_WordLength = USART_WordLength_9b;
	usart.USART_StopBits = USART_StopBits_1;
	usart.USART_Parity = USART_Parity_No;
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
	NVIC_EnableIRQ(USART3_IRQn);
	USART_Init(USART3, &usart);
	USART_DMACmd(USART3, USART_DMAReq_Rx , ENABLE);
	USART_Cmd(USART3, ENABLE);
	sendqueue = xQueueCreate(50, sizeof(uint32_t));
	DMA_DeInit(DMA1_Channel3);
	dma1.DMA_PeripheralBaseAddr = UART_RXDR;
	dma1.DMA_MemoryBaseAddr = (uint32_t) &rxbuffer;
	dma1.DMA_DIR = DMA_DIR_PeripheralSRC;
	dma1.DMA_BufferSize = sizeof(rxbuffer);
	dma1.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma1.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma1.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;
	dma1.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	dma1.DMA_Mode = DMA_Mode_Circular;
	dma1.DMA_Priority = DMA_Priority_Medium;
	dma1.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel3, &dma1);
	DMA_Cmd(DMA1_Channel3, ENABLE);


}

void send_to_uart(char * buffer, uint16_t len)
{
	char * arr;
	portBASE_TYPE xStatus;
	if (xQueueIsQueueFullFromISR(sendqueue))
			return;
	arr = pvPortMalloc(len+1);

	*arr = len;
	memcpy((arr+1),buffer,len);
	xStatus = xQueueSendFromISR(sendqueue, &arr ,0);
	if( xStatus != pdPASS )
	{
		vPortFree(arr);
		return;
	}
}

void vUartTask(void *pvParameters)
{
	uint32_t arr2;
	char *ptr = 0;
	for(;;)
	{
		while (TXUARTReady())
			vTaskDelay(1);
		xQueueReceive(sendqueue,&arr2,portMAX_DELAY);
		ptr = arr2;
		SendDMAUART((ptr+1), (*ptr)-1);
		while (TXUARTReady())
			vTaskDelay(1);
		vPortFree(ptr);

	}
	vTaskDelete( NULL );
}

void SendDMAUART(char * sendbuffer, uint8_t len)
{

	DMA_InitTypeDef dma1;
	while (UARTTXBusyFlag == 1);
	UARTTXBusyFlag = 1;

	USART_DMACmd(USART3, USART_DMAReq_Tx, DISABLE);
	DMA_Cmd(DMA1_Channel2, DISABLE);
	dma1.DMA_PeripheralBaseAddr = (uint32_t)&(USART3->DR);
	dma1.DMA_MemoryBaseAddr = (uint32_t)sendbuffer;
	dma1.DMA_DIR = DMA_DIR_PeripheralDST;
	dma1.DMA_BufferSize = len--;
	dma1.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma1.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma1.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;
	dma1.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	dma1.DMA_Mode = DMA_Mode_Normal;
	dma1.DMA_Priority = DMA_Priority_High;
	dma1.DMA_M2M = DMA_M2M_Disable;

	DMA_Init(DMA1_Channel2, &dma1);
	DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);
	NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
	DMA_Cmd(DMA1_Channel2, ENABLE);

}

uint8_t TXUARTReady()
{
	return UARTTXBusyFlag;
}
void USART3_IRQHandler()
{
	static uint8_t i = 0;

	if ((USART3->DR) == '!')
	{
		i = DMA_GetCurrDataCounter(DMA1_Channel3);

	}
}

void DMA1_Channel2_IRQHandler()
{
	if (DMA_GetITStatus(DMA1_IT_TC2))
	{
		if (USART_GetFlagStatus(USART3,USART_FLAG_TC)!=1)
			return;
		DMA_ClearITPendingBit(DMA1_IT_TC2);
		UARTTXBusyFlag = 0;

	}
}
