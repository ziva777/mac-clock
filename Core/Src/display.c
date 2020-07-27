/*
 * display.c
 *
 *  Created on: 27 сент. 2018 г.
 *      Author: ziva
 */

#include "display.h"

#include <string.h>

HAL_StatusTypeDef DisplaySend(Display *display);

void DisplayCreate(Display *display,
				   size_t n_places,
				   SPI_HandleTypeDef *hspi,
				   GPIO_TypeDef *port,
				   uint16_t pin)
{
	display->hspi = hspi;
	display->port = port;
	display->pin = pin;
	display->transmit_timeout = 1000;

	DisplayBufferCreate(&(display->display_buffer), n_places);
	DisplaySend(display);
}

void DisplayDispose(Display *display)
{
	DisplayBufferDispose(&display->display_buffer);
}

HAL_StatusTypeDef DisplaySend(Display *display)
{
	HAL_StatusTypeDef ret;

	/* Sending */
	ret = HAL_SPI_Transmit(display->hspi,
						   display->display_buffer.buffer,
						   display->display_buffer.buffer_size / sizeof(uint16_t),
						   display->transmit_timeout);
	HAL_GPIO_WritePin(display->port, display->pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(display->port, display->pin, GPIO_PIN_RESET);

//		HAL_SPI_Transmit(&hspi1, display->display_buffer.display_buffer, 7, 1000);
//		HAL_GPIO_TogglePin(LED_DATA_LATCH_GPIO_Port, LED_DATA_LATCH_Pin);
//		HAL_Delay(1);
//		HAL_GPIO_TogglePin(LED_DATA_LATCH_GPIO_Port, LED_DATA_LATCH_Pin);

	return ret;
}

void DisplayWrite(Display *display,
				  Character str[],
                  Dot dot[],
                  size_t n)
{
	DisplayBufferWrite(&display->display_buffer, str, dot, n);
}

void DisplayWriteCharacters(Display *display,
							 Character str[],
							 size_t n)
{
	DisplayBufferWriteCharacters(&display->display_buffer, str, n);
}

void DisplayWriteDots(Display *display,
					  Dot dot[],
					  size_t n)
{
	DisplayBufferWriteDots(&display->display_buffer, dot, n);
}

void DisplayWriteStr(Display *display,
					 const char *str,
					 size_t n)
{
	Character c[n];

	for (size_t i = 0, j = n-1; i != n; ++i, --j) {
		c[j] = AsciiToCharacter(str[i]);
	}

	DisplayBufferWriteCharacters(&display->display_buffer, c, n);
}

void DisplayWriteUint(Display *display,
					  uint32_t value)
{
	Character c[display->display_buffer.logical_size];

	memset(c, CH_BLANK, sizeof(c));

	if (value) {
		uint8_t d;
		size_t i = 0;

		while (value) {
			d = value % 10;
			c[i] = AsciiToCharacter(d + '0');
			value /= 10;
			++i;

		}
	} else {
		c[0] = CH_0;
	}

	DisplayBufferWriteCharacters(&display->display_buffer, c, display->display_buffer.logical_size);
}

void DisplaySync(Display *display)
{
	DisplaySend(display);
}
