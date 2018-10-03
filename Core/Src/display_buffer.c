/*
 * display_buffer.c
 *
 *  Created on: 27 сент. 2018 г.
 *      Author: ziva
 */

#include "display_buffer.h"

#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#define N_BITS_IN_BYTE 8

void DisplayBufferCreate(DisplayBuffer *buffer,
                   		 size_t logical_size)
{
	/* DisplayBuffer buffer structure:
	 *  symbols            dots
	 * |5|4|3|2|1|0| xxxx |0|1|2|3|4|5|
	 * N ............................ 0
	 * */

	buffer->logical_size = logical_size;
	buffer->symbols_buffer_size = logical_size;
	buffer->dots_buffer_size = (logical_size / (sizeof(uint16_t) * N_BITS_IN_BYTE) + 1);
	buffer->buffer_size = (buffer->symbols_buffer_size + buffer->dots_buffer_size) * sizeof(uint16_t);
    buffer->buffer = malloc(buffer->buffer_size);

    memset(buffer->buffer, 0, buffer->buffer_size);
}

void DisplayBufferDispose(DisplayBuffer *buffer)
{
    buffer->logical_size = 0;
    buffer->symbols_buffer_size = 0;
    buffer->dots_buffer_size = 0;
    buffer->buffer_size = 0;

    free(buffer->buffer);
}

void DisplayBufferWrite(DisplayBuffer *buffer,
                  		Character str[],
                  		Dot dot[],
                  		size_t logical_size)
{
	DisplayBufferWriteDots(buffer, dot, logical_size);
	DisplayBufferWriteCharacters(buffer, str, logical_size);
}

void DisplayBufferWriteDots(DisplayBuffer *buffer,
							Dot dot[],
							size_t logical_size)
{
	size_t dot_buff_size = (logical_size / N_BITS_IN_BYTE) + 1;
	uint8_t *dot_buff = malloc(dot_buff_size);
	size_t k, l;

	/* Dots buffer */
	memset(dot_buff, 0, dot_buff_size);

	for (size_t i = 0, j = logical_size-1;
		 i != logical_size; ++i, --j)
	{
		k = i / 8;
		l = i % 8;

		if (dot[j] == DOT_HIGHLIGHT) {
			dot_buff[k] |= POW2(l);
		}
	}

	memcpy(buffer->buffer, dot_buff, dot_buff_size);
	free(dot_buff);
}

void DisplayBufferWriteCharacters(DisplayBuffer *buffer,
							 	  Character str[],
							 	  size_t logical_size)
{
	/* Symbol buffer */
	for (size_t i = buffer->dots_buffer_size;
		 i != logical_size + buffer->dots_buffer_size;
		 ++i)
	{
		((uint16_t *)buffer->buffer)[i] = str[i-buffer->dots_buffer_size];
	}
}
