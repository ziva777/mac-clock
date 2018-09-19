/*
 * ws2812.c
 *
 *  Created on: 3 сент. 2018 г.
 *      Author: bunin
 */

#include "ws2812.h"

//------------------------------------------------------------------
uint32_t BUF_DMA[ARRAY_LEN];

//------------------------------------------------------------------
void ws2812_init(void)
{
	int i;

	memset(BUF_DMA, 0x00, sizeof(BUF_DMA));
	for (i = 0; i < LED_COUNT * 24; i++)
		BUF_DMA[i] = LOW;
}

//------------------------------------------------------------------
void ws2812_pixel_rgb_to_buf_dma(uint8_t Rpixel, uint8_t Gpixel, uint8_t Bpixel,
		uint16_t posX)
{
	volatile uint16_t i;

	for (i = 0; i < 8; i++) {

		if (BitIsSet(Rpixel, (7 - i)) == 1) {
			BUF_DMA[0 + posX * 24 + i + 8] = HIGH;
		} else {
			BUF_DMA[0 + posX * 24 + i + 8] = LOW;
		}

		if (BitIsSet(Gpixel, (7 - i)) == 1) {
			BUF_DMA[0 + posX * 24 + i + 0] = HIGH;
		} else {
			BUF_DMA[0 + posX * 24 + i + 0] = LOW;
		}

		if (BitIsSet(Bpixel, (7 - i)) == 1) {
			BUF_DMA[0 + posX * 24 + i + 16] = HIGH;
		} else {
			BUF_DMA[0 + posX * 24 + i + 16] = LOW;
		}

	}
}
