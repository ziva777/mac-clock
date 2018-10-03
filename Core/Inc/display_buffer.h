/*
 * display_buffer.h
 *
 *  Created on: 27 сент. 2018 г.
 *      Author: ziva
 */

#ifndef INC_DISPLAY_BUFFER_H_
#define INC_DISPLAY_BUFFER_H_

#include <stddef.h>

#include "character.h"

/*
 * DisplayBuffer
 * Internal buffer for Display to holding raw bits.
 */
typedef struct {
    void *buffer;				/* Raw buffer */

    size_t logical_size; 		/* number of places */
    size_t buffer_size; 		/* buffer size in bytes */

    size_t symbols_buffer_size; /* symbols chunk size in 16bits */
    size_t dots_buffer_size;	/* dots chunk size in 16bits */
} DisplayBuffer;


void DisplayBufferCreate(DisplayBuffer *buffer,
                   		 size_t logical_size);

void DisplayBufferDispose(DisplayBuffer *buffer);

void DisplayBufferWrite(DisplayBuffer *buffer,
                  		Character str[],
                  		Dot dot[],
                  		size_t logical_size);

void DisplayBufferWriteDots(DisplayBuffer *buffer,
							Dot dot[],
							size_t logical_size);

void DisplayBufferWriteCharacters(DisplayBuffer *buffer,
							 	  Character str[],
							 	  size_t logical_size);

#endif /* INC_DISPLAY_BUFFER_H_ */
