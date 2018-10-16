/*
 * fsm_luminosity.c
 *
 *  Created on: 16 окт. 2018 г.
 *      Author: ziva
 */

#include "fsm_luminosity.h"

#include <stdint.h>

#include "aux.h"
#include "luminosity_sensor.h"

static const uint8_t 	BR_MIN = 1u;        /* Abs brightness minimum */
static const uint8_t 	BR_MAX = 254u;      /* Abs brightness maximum */

void FsmLuminosityCreate(FsmLuminosity *fsm)
{
	/* Yep, these params are magic! */
	KalmanFilterInit(&fsm->luminosity_filter,
					 4000.0f,
					 4000.0f,
					 0.125f);
	LuminositySensorCreate(&luminosity_sensor);
	LuminositySensorBegin(&luminosity_sensor);
}

void FsmLuminosityDoCalc(FsmLuminosity *fsm)
{
	if (LuminositySensorIsReady(&luminosity_sensor)) {
		fsm->lum = luminosity_sensor.value[0];
		fsm->lum = KalmanFilterUpdate(&fsm->luminosity_filter, fsm->lum);
		fsm->br = calc_brightness_linear(fsm->lum, BR_MIN, BR_MAX);
		SetBrightness(fsm->br);

		LuminositySensorBegin(&luminosity_sensor);
	}
}

