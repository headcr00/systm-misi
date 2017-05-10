#include "task_measuretimer.h"
#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "task.h"
#include "stdio.h"

uint8_t txbuffer[12];
void vMeasureTimer (void * pvParameters)
{
	uint8_t measure_number = 0;
	uint8_t iterator = 0;
	initMeasure();
	for(;;)
	{
		switch(iterator)
		{
		case 0:
			/*Measure Time! Give Semaphore!*/
			measure_number++;
			send_to_uart(txbuffer, sprintf(txbuffer,"ADC0\tADC1\tREF\tV0\tV1\tVREF\tRsh\tRbn\tMEAS:%u\r\n", measure_number));
			GPIO_SetBits(GPIOC, GPIO_Pin_5);
			GPIO_SetBits(GPIOB, GPIO_Pin_8);
			xSemaphoreGive(xMeasureToggle);
			break;
		case 1:
			/*Break Time! Take semaphore!*/
			xSemaphoreTake(xMeasureToggle, portMAX_DELAY);
			GPIO_ResetBits(GPIOC, GPIO_Pin_5);
			GPIO_ResetBits(GPIOB, GPIO_Pin_8);
			send_to_uart(txbuffer,sprintf(txbuffer,"ENDM_NO_%u\r\n", measure_number));
			break;
		}
		iterator = (~iterator & 0x01);
		vTaskDelay(1000);//Ten seconds delay between measurements
	}
	vTaskDelete( NULL );
}


void initMeasure()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitTypeDef gpio;
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init(GPIOC, &gpio);

	gpio.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOB, &gpio);
	vSemaphoreCreateBinary( xMeasureToggle );
	//xSemaphoreTake(xMeasureToggle, portMAX_DELAY);
}
