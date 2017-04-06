/*
 * task_adc.c
 *
 *  Created on: 22 марта 2017 г.
 *      Author: Кочкин
 */

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "task_adc.h"
#include "task_uart.h"
#include "task_measuretimer.h"
#define ADC1_DR_Address ((uint32_t)0x4001244C)
#define MAXCHANNELS 2
void init_adc(void);
uint16_t search_max(uint16_t * buffer, uint16_t len, uint8_t channel);
uint16_t search_min(uint16_t * buffer, uint16_t len, uint8_t channel);

uint16_t ADCConvertedValue[256] = {0};

void vADC(void *pvParameters)
{
	uint32_t minampl= 0;
	uint32_t maxampl = 0;
	uint32_t ampl = 0;

	init_adc();
	for(;;)
	{


		while (DMA_GetFlagStatus(DMA1_FLAG_TC1) == RESET )
		{
			taskYIELD();
		}
		DMA_ClearFlag(DMA1_FLAG_TC1);
		//TIM_Cmd(TIM2, DISABLE);
		xSemaphoreTake(xMeasureToggle, portMAX_DELAY);

		maxampl = search_max(ADCConvertedValue, 255, 1);
		minampl = search_min(ADCConvertedValue, 255, 1);
		maxampl = maxampl * 3331/ 0x0fff;
		minampl = minampl * 3331 / 0xfff;
		ampl = maxampl-minampl;
		xQueueSendToBack(amplQueue, &ampl, 0);
		xQueueSendToBack(minAmplQueue, &minampl, 0);

		maxampl = 0;
		minampl = 0;
		ampl = 0;
		xSemaphoreGive(xMeasureToggle);
		//TIM_Cmd(TIM2, ENABLE);
		vTaskDelay(200);
	}
}

uint16_t search_max(uint16_t * buffer, uint16_t len, uint8_t channel)
{
	uint16_t retval = 0;
	for (uint16_t i = channel; i < len; i = i + MAXCHANNELS)
	{
		if (i > len)
				break;
		if (retval < *(buffer + i))
			retval = *(buffer + i);
	}
	return retval;
}

uint16_t search_min(uint16_t * buffer, uint16_t len, uint8_t channel)
{
	uint16_t retval = 0xFFFF;
	for (uint16_t i = channel; i < len; i = i + MAXCHANNELS)
	{
		if (i > len)
			break;
		if ( *(buffer + i) < retval)
			retval = *(buffer + i);
	}
	return retval;
}

void init_adc(void)
{
	GPIO_InitTypeDef gpio;
	ADC_InitTypeDef adc;
	TIM_TimeBaseInitTypeDef tim_base;
	TIM_OCInitTypeDef tim_oc;

	//RCC setup
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	//ADC Input PC4
	gpio.GPIO_Mode = GPIO_Mode_AIN;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_Init(GPIOA, &gpio);

	DMA_InitTypeDef dma1;
		//ADC DMA Config
	DMA_DeInit(DMA1_Channel1);
	dma1.DMA_PeripheralBaseAddr = ADC1_DR_Address;
	dma1.DMA_MemoryBaseAddr = (uint32_t) &ADCConvertedValue;
	dma1.DMA_DIR = DMA_DIR_PeripheralSRC;
	dma1.DMA_BufferSize = sizeof(ADCConvertedValue)/2;
	dma1.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma1.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma1.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	dma1.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	dma1.DMA_Mode = DMA_Mode_Circular;
	dma1.DMA_Priority = DMA_Priority_High;
	dma1.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &dma1);
	DMA_Cmd(DMA1_Channel1, ENABLE);


	TIM_TimeBaseStructInit(&tim_base);
	tim_base.TIM_Period = 0xBB;
	tim_base.TIM_Prescaler = 64;
	tim_base.TIM_ClockDivision = 0x0;
	tim_base.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &tim_base);

	TIM_OCStructInit(&tim_oc);
	// TIM_Pulse - при каком значении счётчика таймера появиться событие TIM2_CC2
	tim_oc.TIM_Pulse = 187;
	tim_oc.TIM_OCMode = TIM_OCMode_Toggle;
	TIM_OC2Init(TIM2, &tim_oc);
	TIM2->CCER |= TIM_CCER_CC2E;


	adc.ADC_Mode = ADC_Mode_Independent;
	adc.ADC_ScanConvMode = ENABLE;
	adc.ADC_ContinuousConvMode = DISABLE;
	adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_CC2;
	adc.ADC_DataAlign = ADC_DataAlign_Right;
	adc.ADC_NbrOfChannel = 2;
	ADC_Init(ADC1, &adc);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_71Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_71Cycles5);

	ADC_DMACmd(ADC1, ENABLE);

	ADC_ExternalTrigConvCmd(ADC1, ENABLE);

	ADC_Cmd(ADC1, ENABLE);
	/* Enable ADC1 reset calibration register */
	ADC_ResetCalibration(ADC1);
	/* Check the end of ADC1 reset calibration register */
	while (ADC_GetResetCalibrationStatus(ADC1))
		;

	/* Start ADC1 calibration */
	ADC_StartCalibration(ADC1);
	/* Check the end of ADC1 calibration */
	while (ADC_GetCalibrationStatus(ADC1))
		;


	vTaskDelay(300);



	//ADC_SoftwareStartConvCmd(ADC1, ENABLE);

	TIM_Cmd(TIM2, ENABLE);


	//ADC is ready to shoot, USART is ready to spit data.

}

void TIM2_IRQHandler(void){

    if (TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET) {

         TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
    }
}
