/*
 * task_adc.h
 *
 *  Created on: 22 марта 2017 г.
 *      Author: Кочкин
 */

#ifndef TASK_ADC_H_
#define TASK_ADC_H_

#include "stm32f10x.h"
#include "FreeRTOS.h"
//ADC and buffer parameters
#define MAXCHANNELS 3
#define ARRAYSIZE 128
#define REFERENCECHANNEL 2

void vADC(void *pvParameters);

#endif /* TASK_ADC_H_ */
