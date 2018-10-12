/*
 * luminosity_sensor.c
 *
 *  Created on: 2 окт. 2018 г.
 *      Author: ziva
 */

#include "luminosity_sensor.h"

#include "adc.h"

LuminositySensor luminosity_sensor;

void LuminositySensorCreate(LuminositySensor *sensor)
{
	sensor->value[0] = 0;
	sensor->value_ready = ATOMIC_VAR_INIT(0);
}

void LuminositySensorCommit(LuminositySensor *sensor)
{
    atomic_fetch_add(&sensor->value_ready, 1);
}

int LuminositySensorIsReady(LuminositySensor *sensor)
{
    int tmp = atomic_load(&sensor->value_ready);
    return tmp;
}

void LuminositySensorBegin(LuminositySensor *sensor)
{
	sensor->value_ready = ATOMIC_VAR_INIT(0);
	HAL_ADC_Start_DMA(&hadc1, sensor->value, 1);
}
