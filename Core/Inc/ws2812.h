/*
 * ws2812.h
 *
 *  Created on: 3 сент. 2018 г.
 *      Author: bunin
 */

#ifndef INC_WS2812_H_
#define INC_WS2812_H_

#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#define DELAY_LEN 48
#define LED_COUNT 2

#define ARRAY_LEN DELAY_LEN + LED_COUNT * 24

//#define HIGH 76
#define HIGH 58
#define LOW 29

//--------------------------------------------------
#define BitIsSet(reg, bit) ((reg & (1 << bit)) != 0)

//--------------------------------------------------
extern uint32_t BUF_DMA[ARRAY_LEN];

//--------------------------------------------------
void ws2812_init(void);
void ws2812_pixel_rgb_to_buf_dma(uint8_t Rpixel, uint8_t Gpixel, uint8_t Bpixel,
		uint16_t posX);

#endif /* INC_WS2812_H_ */
