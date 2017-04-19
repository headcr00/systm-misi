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
#include "math.h"
#include "stdio.h"
#define ADC1_DR_Address ((uint32_t)0x4001244C)
#define MAXCHANNELS 3
#define ARRAYSIZE 128
#define REFERENCECHANNEL 2

void init_adc(void);
uint16_t search_max(uint16_t * buffer, uint16_t len, uint8_t channel);
uint16_t search_min(uint16_t * buffer, uint16_t len, uint8_t channel);
uint16_t search_max_array(uint16_t * buffer, uint16_t len);
uint16_t search_min_array(uint16_t * buffer, uint16_t len);
uint16_t median(uint16_t * buffer, uint16_t len, uint8_t channel);
void reject_filter(uint16_t * buffer, uint16_t * outbuffer, uint16_t len, uint8_t channel);

uint16_t ADCConvertedValue[ARRAYSIZE * MAXCHANNELS] = {0};
uint16_t ADCFiltered50[ARRAYSIZE] = {0};

void vADC(void *pvParameters)
{
	uint32_t minampl= 0;
	uint32_t maxampl = 0;
	uint32_t ampl = 0;
	uint32_t dcval = 0;
	uint32_t dcfiltval = 0;
	uint32_t amplfilt = 0;
	uint32_t vref = 0;
	uint16_t datalength = 0;
	datalength = sizeof(ADCConvertedValue)/2;
	init_adc();
	for(;;)
	{
		xSemaphoreTake(xMeasureToggle, portMAX_DELAY);
		//Measure reference voltage
		reject_filter(ADCConvertedValue, ADCFiltered50, datalength, REFERENCECHANNEL);
		//Search max and min from the middle of array, because of filter characteristics
		maxampl = search_max_array(&(*(ADCFiltered50+ARRAYSIZE/2)), ARRAYSIZE/2);
		minampl = search_min_array(&(*(ADCFiltered50+ARRAYSIZE/2)), ARRAYSIZE/2);
		amplfilt = maxampl-minampl;
		//Measure effective ripple value
		vref = (uint32_t)((float)minampl + (float)(amplfilt)/sqrt(2.0));


		for (uint8_t i = 0; i < MAXCHANNELS; i++){

			maxampl = search_max(ADCConvertedValue, datalength, i);
			minampl = search_min(ADCConvertedValue, datalength, i);
			ampl = maxampl-minampl;

			dcval = (uint32_t)((float)minampl + (float)ampl/sqrt(2.0));
			if (i == REFERENCECHANNEL)
				{
				ampl = ampl * 3331 / 0xFFF;
				dcval = dcval * 3331 / 0xFFF;
				printf("REF:AMP:%u;DCV:%u\n\r", ampl, dcval);
				break;
				}
			ampl = ampl * 2500/vref;
			dcval = dcval * 2500/vref;

			reject_filter(ADCConvertedValue, ADCFiltered50, datalength, i);

			maxampl = search_max_array(&(*(ADCFiltered50+ARRAYSIZE/2)), ARRAYSIZE/2);
			minampl = search_min_array(&(*(ADCFiltered50+ARRAYSIZE/2)), ARRAYSIZE/2);
			amplfilt = maxampl-minampl;
			dcfiltval = (uint32_t)((float)minampl + (float)(amplfilt)/sqrt(2.0));
			amplfilt = amplfilt * 2500/vref;
			dcfiltval = dcfiltval * 2500/vref;

			printf("CH%u:AMP:%u;DCV:%u;FAMP:%u;FDC:%u\n\r",i,ampl, dcval,amplfilt,dcfiltval);
		}
		maxampl = 0;
		minampl = 0;
		ampl = 0;
		dcval = 0;
		dcfiltval =0;
		vref = 0;
		TIM_Cmd(TIM2, ENABLE);
		xSemaphoreGive(xMeasureToggle);

		vTaskDelay(1000);
	}
}
uint16_t median(uint16_t * buffer, uint16_t len, uint8_t channel)
{
	uint16_t retval = 0;
	for (uint16_t i = channel; i < len; i = i + MAXCHANNELS)
	{
		if (i >= len)
				break;
			retval = (retval + *(buffer + i))/2;
	}
	return retval;
}
uint16_t search_max(uint16_t * buffer, uint16_t len, uint8_t channel)
{
	/* returns maximum value of array */
	uint16_t retval = 0;
	for (uint16_t i = channel; i < len; i = i + MAXCHANNELS)
	{
		if (i >= len)
				break;
		if (retval < *(buffer + i))
			retval = *(buffer + i);
	}
	return retval;
}
uint16_t search_max_array(uint16_t * buffer, uint16_t len)
{
	/* returns maximum value of array */
	uint16_t retval = 0;
	for (uint16_t i = 0; i < len; i++)
	{
		if (i >= len)
				break;
		if (retval < *(buffer + i))
			retval = *(buffer + i);
	}
	return retval;
}


uint16_t search_min(uint16_t * buffer, uint16_t len, uint8_t channel)
{
	/* returns minimum value of array */
	uint16_t retval = 0xFFFF;
	for (uint16_t i = channel; i < len; i = i + MAXCHANNELS)
	{
		if (i >= len)
			break;
		if ( *(buffer + i) < retval)
			retval = *(buffer + i);
	}
	return retval;
}

uint16_t search_min_array(uint16_t * buffer, uint16_t len)
{
	/* returns minimum value of array */
	uint16_t retval = 0xFFFF;
	for (uint16_t i = 0; i < len; i++)
	{
		if (i >= len)
			break;
		if ( *(buffer + i) < retval)
			retval = *(buffer + i);
	}
	return retval;
}

void reject_filter(uint16_t * buffer, uint16_t * outbuffer, uint16_t len, uint8_t channel)
{
	uint32_t Na = 15;
	uint32_t Nb = 1;
	uint32_t k = 4;

	uint16_t cnt = 1;
	for (uint16_t i = channel; i < len; i = i + MAXCHANNELS)
		{
			if (i >= len)
				break;
			*(outbuffer + cnt) = (Na * (*(outbuffer + cnt - 1)) + (Nb * (*(buffer + i))) ) >> k;
			cnt++;
		}
	__asm("nop");
}

void init_adc(void)
{
	GPIO_InitTypeDef gpio;
	ADC_InitTypeDef adc;
	TIM_TimeBaseInitTypeDef tim_base;
	TIM_OCInitTypeDef tim_oc;
	NVIC_InitTypeDef nvic;
	//RCC setup
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	//ADC Input PC4
	gpio.GPIO_Mode = GPIO_Mode_AIN;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4;
	GPIO_Init(GPIOA, &gpio);

	DMA_InitTypeDef dma1;
		//ADC DMA Config
	DMA_DeInit(DMA1_Channel1);
	dma1.DMA_PeripheralBaseAddr = ADC1_DR_Address;
	dma1.DMA_MemoryBaseAddr = (uint32_t) &ADCConvertedValue;
	dma1.DMA_DIR = DMA_DIR_PeripheralSRC;
	dma1.DMA_BufferSize = ARRAYSIZE * MAXCHANNELS;//sizeof(ADCConvertedValue)/2;
	dma1.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma1.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma1.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	dma1.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	dma1.DMA_Mode = DMA_Mode_Circular;
	dma1.DMA_Priority = DMA_Priority_High;
	dma1.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &dma1);
	DMA_Cmd(DMA1_Channel1, ENABLE);
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
	nvic.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&nvic);


	TIM_TimeBaseStructInit(&tim_base);
	tim_base.TIM_Period = 0x3A7;
	tim_base.TIM_Prescaler = 64;
	tim_base.TIM_ClockDivision = 0x0;
	tim_base.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &tim_base);

	TIM_OCStructInit(&tim_oc);
	// TIM_Pulse - при каком значении счётчика таймера появиться событие TIM2_CC2
	tim_oc.TIM_Pulse = 0x3A7;
	tim_oc.TIM_OCMode = TIM_OCMode_Toggle;
	TIM_OC2Init(TIM2, &tim_oc);
	TIM2->CCER |= TIM_CCER_CC2E;
	TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);



	adc.ADC_Mode = ADC_Mode_Independent;
	adc.ADC_ScanConvMode = ENABLE;
	adc.ADC_ContinuousConvMode = DISABLE;
	adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_CC2;
	adc.ADC_DataAlign = ADC_DataAlign_Right;
	adc.ADC_NbrOfChannel = 3;
	ADC_Init(ADC1, &adc);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_13Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_13Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 3, ADC_SampleTime_13Cycles5);
//ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 4, ADC_SampleTime_13Cycles5);
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






	//ADC_SoftwareStartConvCmd(ADC1, ENABLE);

	TIM_Cmd(TIM2, ENABLE);


	//ADC is ready to shoot, USART is ready to spit data.

}

void DMA1_Channel1_IRQHandler()
{
	if (DMA_GetITStatus(DMA1_IT_TC1))
	{
		DMA_ClearITPendingBit(DMA1_IT_TC1);
		TIM_Cmd(TIM2, DISABLE);
	}
}

