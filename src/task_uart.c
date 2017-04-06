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

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


void uart_init(void);

void uart_init(void) {
	GPIO_InitTypeDef gpio;
	USART_InitTypeDef usart;

	RCC_APB2PeriphClockCmd(
	RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
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

	usart.USART_BaudRate = 115200;
	usart.USART_WordLength = USART_WordLength_8b;
	usart.USART_StopBits = USART_StopBits_1;
	usart.USART_Parity = USART_Parity_No;
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART3, &usart);
	USART_Cmd(USART3, ENABLE);

	voltageBuffer = xQueueCreate(10, 4);
	currentBuffer = xQueueCreate(10, 4);

}

void vUart(void *pvParameters) {
	/* TODO: FIFO Queueue */
	uint16_t voltageValue;
	uint16_t currentValue;
	uart_init();
	for (;;) {
			xQueueReceive( currentBuffer, &currentValue, portMAX_DELAY );
			xQueueReceive( voltageBuffer, &voltageValue, portMAX_DELAY );
			printf("%i %i\r", (uint32_t) currentValue, (uint32_t)voltageValue);
	}
}

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART3, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
  {
	  taskYIELD();
  }

  return ch;
}


