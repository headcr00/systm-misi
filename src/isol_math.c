/*
 * isol_math.c
 *
 *  Created on: 10 мая 2017 г.
 *      Author: Кочкин
 */
#define RZASH (uint32_t)1001460
#define RSHUNT (uint32_t)32396
#define UIN (uint32_t)59960
#define RBN (uint32_t)13544900
#define RBN2 (uint32_t) 51000
#define Ku (float)10.2
#include "stm32f10x.h"

uint32_t calculate_shunt(uint32_t voltage_mv)
{	float p1;
	float p2;
	p1 = (((((float)UIN*(float)RSHUNT)/(float)voltage_mv)-(float)RZASH-(float)RSHUNT)*(float)RBN);
	p2 = ((float)RBN-(((float)UIN*RSHUNT)/(float)voltage_mv)+(float)RZASH+(float)RSHUNT);
	return (uint32_t)(p1/p2);
}

uint32_t calculate_blocknaze(uint32_t voltage_mv, uint32_t voltage_shunt, uint32_t voltage_ref)
{
	float realv;
	realv = ((float) voltage_ref - (float)voltage_mv) / Ku;
	return (uint32_t)((realv*(float)RBN*(float)RSHUNT)/(((float)voltage_shunt*(float)RBN2)-(realv*(float)RSHUNT)));


}
