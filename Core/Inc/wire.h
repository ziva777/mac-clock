/*
 * wire.h
 *
 *  Created on: 28 сент. 2018 г.
 *      Author: ziva
 */

#ifndef INC_WIRE_H_
#define INC_WIRE_H_

#include <stdint.h>

#include "stm32f4xx_hal.h"

void Wire_SetI2C(I2C_HandleTypeDef *i2c);

void Wire_beginTransmission(uint8_t address);
uint8_t Wire_endTransmission(void);

uint8_t Wire_requestFrom(uint8_t address,
                         uint8_t quantity);

size_t Wire_write(uint8_t data);
int Wire_read(void);

#endif /* INC_WIRE_H_ */
