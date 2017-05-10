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
#include "array_functions.h"
#include "isol_math.h"
#define ADC1_DR_Address ((uint32_t)0x4001244C)


uint8_t dma_tc = 0;
uint16_t ADCConvertedValue[ARRAYSIZE * MAXCHANNELS] = {0};
uint16_t ADCFiltered50[ARRAYSIZE] = {0};
char txbuffer[64];

void vADC(void *pvParameters)
{
	uint32_t adc0;
	uint32_t adc1;
	uint32_t ref;
	uint32_t voltage0;
	uint32_t voltage1;
	uint32_t voltageref;
	uint32_t Rshunt;
	uint32_t Rbn;
	uint16_t datalength = 0;
	uint16_t array_length;
	datalength = sizeof(ADCConvertedValue)/2;

	for(;;)
	{
		xSemaphoreTake(xMeasureToggle, portMAX_DELAY);
		//Measure reference voltage

		if(dma_tc == 1)
			{
			adc0 = median(ADCConvertedValue,datalength,0);
			adc1 = median(ADCConvertedValue,datalength,1);
			ref = median(ADCConvertedValue,datalength,2);
			voltage0 = adc0 * 2506 / ref;
			voltage1 = adc1 * 2506 / ref;
			voltageref = ref * 3333 / 4096;
			Rshunt = calculate_shunt(voltage1);
			Rbn = calculate_blocknaze(voltage0, voltage1, voltageref);
			array_length = sprintf(txbuffer,"%u\t%u\t%u\t%u\t%u\t%u\t%u\t%u\t\r\n",adc0,adc1, ref, voltage0, voltage1, voltageref, Rshunt, Rbn);
			send_to_uart(txbuffer, array_length);

			dma_tc = 0;
			}
		TIM_Cmd(TIM2,ENABLE);

		xSemaphoreGive(xMeasureToggle);

		vTaskDelay(1);
	}
	vTaskDelete( NULL );
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
	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	TIM_TimeBaseStructInit(&tim_base);
	tim_base.TIM_Period = 0x3A7;
	tim_base.TIM_Prescaler = 64 / MAXCHANNELS;
	tim_base.TIM_ClockDivision = 0x0;
	tim_base.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &tim_base);

	TIM_OCStructInit(&tim_oc);
	// TIM_Pulse - при каком значении счётчика таймера появится событие TIM2_CC2
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

	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_41Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_41Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 3, ADC_SampleTime_41Cycles5);
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

	TIM_Cmd(TIM2, ENABLE);


	//ADC is ready to shoot, USART is ready to spit data.

}


void DMA1_Channel1_IRQHandler()
{
	if (DMA_GetITStatus(DMA1_IT_TC1))
	{
		DMA_ClearITPendingBit(DMA1_IT_TC1);
		TIM_Cmd(TIM2, DISABLE);
		dma_tc = 1;
	}
}

void TIM2_IRQHandler()
{
	static uint8_t bit = 0;
	if (TIM_GetITStatus(TIM2, TIM_IT_CC2))
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
		GPIO_WriteBit(GPIOC, GPIO_Pin_9, bit&0x01);
		bit++;
	}

}

