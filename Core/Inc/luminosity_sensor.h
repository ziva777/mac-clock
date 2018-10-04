/*
 * luminosity_sensor.h
 *
 *  Created on: 2 окт. 2018 г.
 *      Author: ziva
 */

#ifndef INC_LUMINOSITY_SENSOR_H_
#define INC_LUMINOSITY_SENSOR_H_

#include <stdatomic.h>
#include <stdint.h>

typedef struct {
	uint32_t value[1];
	atomic_int value_ready;
} LuminositySensor;

extern LuminositySensor luminosity_sensor;

void 	LuminositySensorCreate(LuminositySensor *sensor);
void 	LuminositySensorCommit(LuminositySensor *sensor);
int 	LuminositySensorIsReady(LuminositySensor *sensor);
void 	LuminositySensorBegin(LuminositySensor *sensor);

#endif /* INC_LUMINOSITY_SENSOR_H_ */
