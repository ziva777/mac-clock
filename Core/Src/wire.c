/*
 * wire.c
 *
 *  Created on: 28 сент. 2018 г.
 *      Author: ziva
 */

#include "wire.h"

#include <string.h>

#include "i2c.h"

static uint16_t wire_dev_address;
static uint8_t wire_buff_idx;
static uint8_t wire_buff[256];
static const uint32_t wire_timeout = 100u;
static I2C_HandleTypeDef *wire_i2c;

void Wire_SetI2C(I2C_HandleTypeDef *i2c)
{
    wire_i2c = i2c;
}

void Wire_beginTransmission(uint8_t address)
{
    wire_dev_address = address;

    memset(wire_buff, 0, sizeof(wire_buff));
    wire_buff_idx = 0;
}

uint8_t Wire_endTransmission(void)
{
    HAL_I2C_Master_Transmit(wire_i2c,
                            wire_dev_address,
                            wire_buff,
                            wire_buff_idx,
                            wire_timeout);
    return wire_buff_idx;
}

uint8_t Wire_requestFrom(uint8_t address,
                         uint8_t quantity)
{
    memset(wire_buff, 0, sizeof(wire_buff));
    wire_buff_idx = 0;

    HAL_I2C_Master_Receive(wire_i2c,
                           wire_dev_address,
                           wire_buff,
                           quantity,
                           wire_timeout);

    return quantity;
}

size_t Wire_write(uint8_t data)
{
    wire_buff[wire_buff_idx++] = data;
    return 1;
}

int Wire_read(void)
{
    return wire_buff[wire_buff_idx++];
}
