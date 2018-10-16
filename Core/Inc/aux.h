/*
 * aux.h
 *
 *  Created on: 16 окт. 2018 г.
 *      Author: ziva
 */

#ifndef INC_AUX_H_
#define INC_AUX_H_

#include <stdint.h>

void SetBrightness(uint16_t br);
void SetBrightnessOn();
void SetBacklight(uint8_t r,
				  uint8_t g,
				  uint8_t b);
void Beep();

float calc_brightness_linear(float l,
                             float b_inf,
                             float b_sup);

#endif /* INC_AUX_H_ */
