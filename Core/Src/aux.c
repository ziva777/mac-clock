/*
 * aux.c
 *
 *  Created on: 16 окт. 2018 г.
 *      Author: ziva
 */

#include "aux.h"

#include "tim.h"
#include "ws2812.h"
#include "stm32f4xx_hal.h"

static const uint16_t 	LUM_MIN = 0u;      /* Abs luminosity minimum */
static const uint16_t 	LUM_MAX = 4095u;   /* Abs luminosity maximum */

void SetBrightness(uint16_t br)
{
    htim5.Instance->CCR4 = br;
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
}

void SetBrightnessOn()
{
    HAL_TIM_Base_Start(&htim5);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
}

void SetBacklight(uint8_t r,
				  uint8_t g,
				  uint8_t b)
{
    static uint8_t R, G, B;

    if (r != R || g != G || b != B) {
        R = r;
        G = g;
        B = b;

        ws2812_pixel_rgb_to_buf_dma(r, g, b, 0);
        ws2812_pixel_rgb_to_buf_dma(r, g, b, 1);
        HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, BUF_DMA, ARRAY_LEN);
    }
}

void Beep()
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_Delay(10);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
}

float calc_brightness_linear(float l,
                             float b_inf,
                             float b_sup)
{
    float k, b, y;

    k = (b_inf - b_sup);
    k /=  (float)(LUM_MAX - LUM_MIN);
    b = b_sup;
    y = k * l + b;

    return y;
}
