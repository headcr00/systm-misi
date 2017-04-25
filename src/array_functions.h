/*
 * array_functions.h
 *
 *  Created on: 20 апр. 2017 г.
 *      Author: Кочкин
 */

#ifndef ARRAY_FUNCTIONS_H_
#define ARRAY_FUNCTIONS_H_

#include "stm32f10x.h"
#include "task_adc.h"

uint16_t search_max(uint16_t * buffer, uint16_t len, uint8_t channel);
uint16_t search_min(uint16_t * buffer, uint16_t len, uint8_t channel);
uint16_t search_max_array(uint16_t * buffer, uint16_t len);
uint16_t search_min_array(uint16_t * buffer, uint16_t len);
uint16_t median(uint16_t * buffer, uint16_t len, uint8_t channel);
void reject_filter(uint16_t * buffer, uint16_t * outbuffer, uint16_t len, uint8_t channel);


#endif /* ARRAY_FUNCTIONS_H_ */
