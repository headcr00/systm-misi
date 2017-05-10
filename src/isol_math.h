/*
 * isol_math.h
 *
 *  Created on: 10 мая 2017 г.
 *      Author: Кочкин
 */

#ifndef ISOL_MATH_H_
#define ISOL_MATH_H_
#include "stm32f10x.h"
uint32_t calculate_shunt(uint32_t voltage_mv);
uint32_t calculate_blocknaze(uint32_t voltage_mv, uint32_t voltage_shunt, uint32_t voltage_ref);
#endif /* ISOL_MATH_H_ */
