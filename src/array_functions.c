/*
 * array_functions.c
 *
 *  Created on: 20 апр. 2017 г.
 *      Author: Кочкин
 */


#include "array_functions.h"

uint16_t median(uint16_t * buffer, uint16_t len, uint8_t channel)
{
	uint32_t retval = 0;
	for (uint16_t i = channel; i < len; i = i + MAXCHANNELS)
	{
		if (i >= len)
				break;
			retval = (retval + *(buffer + i))/2;
	}
	return (uint16_t)retval;
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
}

