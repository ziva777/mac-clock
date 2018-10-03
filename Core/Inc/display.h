/*
 * display.h
 *
 *  Created on: 27 сент. 2018 г.
 *      Author: ziva
 */

#ifndef INC_DISPLAY_H_
#define INC_DISPLAY_H_

#include "display_buffer.h"

#include <stdlib.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"
#include "spi.h"

/*
 * Display
 * Module to display 18 segment LED
 */
typedef struct {
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *port;
	uint16_t pin;
	uint32_t transmit_timeout;

	DisplayBuffer display_buffer;
} Display;


void DisplayCreate(Display *display,
				   size_t n_places,
				   SPI_HandleTypeDef *hspi,
				   GPIO_TypeDef *port,
				   uint16_t pin);

void DisplayDispose(Display *display);

void DisplayWrite(Display *display,
				  Character str[],
                  Dot dot[],
                  size_t n);

void DisplayWriteCharacters(Display *display,
							Character str[],
							size_t n);

void DisplayWriteDots(Display *display,
					  Dot dot[],
					  size_t n);

void DisplayWriteStr(Display *display,
					 const char *str,
					 size_t n);

void DisplayWriteUint(Display *display,
					  uint32_t value);

void DisplaySync(Display *display);

#endif /* INC_DISPLAY_H_ */
