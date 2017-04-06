/*
 * task_adc.c
 *
 *  Created on: 22 ����� 2017 �.
 *      Author: ������
 */

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "task_adc.h"
#include "task_uart.h"
#include "task_measuretimer.h"
#define ADC1_DR_Address ((uint32_t)0x4001244C)
#define MAXCHANNELS 1
void init_adc(void);
uint16_t search_max(uint16_t * buffer, uint16_t len, uint8_t channel);
uint16_t search_min(uint16_t * buffer, uint16_t len, uint8_t channel);

uint16_t ADCConvertedValue[256] = {0};

void vADC(void *pvParameters)
{
	uint32_t minampl= 0;
	uint32_t maxampl = 0;

	init_adc();
	for(;;)
	{


		while (DMA_GetFlagStatus(DMA1_FLAG_TC1) == RESET )
		{
			taskYIELD();
		}
		DMA_ClearFlag(DMA1_FLAG_TC1);
		xSemaphoreTake(xMeasureToggle, portMAX_DELAY);

		maxampl = search_max(ADCConvertedValue, 255, 0);
		minampl= search_min(ADCConvertedValue, 255, 0);
		xQueueSendToBack(voltageBuffer, &maxampl, 0);
		xQueueSendToBack(currentBuffer, &minampl, 0);
		for (uint8_t i = 0; i < (254); i += 2)
		{
			//voltageresult = (voltageresult + ADCConvertedValue[i]) / 2;
			//currentresult = (currentresult + ADCConvertedValue[i + 1]) / 2;
			//referencevoltage = (referencevoltage + ADCConvertedValue[i + 3]) / 2;

			xQueueSendToBack(voltageBuffer, &ADCConvertedValue[i], 0);
			xQueueSendToBack(currentBuffer, &ADCConvertedValue[i+1], 0);
		}
		//voltageresult = voltageresult * 3331/ 0x0fff;
		//currentresult = currentresult * 3331 / 0xfff;
		//xQueueSendToBack(voltageBuffer, &voltageresult, 0);

		maxampl = 0;
		minampl = 0;
		xSemaphoreGive(xMeasureToggle);

		vTaskDelay(1000);
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
		if (retval > *(buffer + i))
			retval = *(buffer + i);
	}
	return retval;
}

void init_adc(void)
{
	GPIO_InitTypeDef gpio;
	ADC_InitTypeDef adc;


	//RCC setup
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);
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
	dma1.DMA_Mode = DMA_Mode_Normal;
	dma1.DMA_Priority = DMA_Priority_High;
	dma1.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &dma1);

	adc.ADC_Mode = ADC_Mode_Independent;
	adc.ADC_ScanConvMode = ENABLE;
	adc.ADC_ContinuousConvMode = ENABLE;
	adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	adc.ADC_DataAlign = ADC_DataAlign_Right;
	adc.ADC_NbrOfChannel = 2;
	ADC_Init(ADC1, &adc);
	ADC_TempSensorVrefintCmd(DISABLE);


	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_41Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_41Cycles5);
	//ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 3, ADC_SampleTime_28Cycles5);
	//ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 4, ADC_SampleTime_28Cycles5);



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

	DMA_Cmd(DMA1_Channel1, ENABLE);
	ADC_DMACmd(ADC1, ENABLE);
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);

	//ADC is ready to shoot, USART is ready to spit data.

}