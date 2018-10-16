/*
 * fsm_luminosity.h
 *
 *  Created on: 16 окт. 2018 г.
 *      Author: ziva
 */

#ifndef INC_FSM_LUMINOSITY_H_
#define INC_FSM_LUMINOSITY_H_

#include "kalman.h"

/*
 * FSM luminosity
 */
typedef struct {
	KalmanFilter luminosity_filter;
    float lum;
    float br;
} FsmLuminosity;

void FsmLuminosityCreate(FsmLuminosity *fsm);
void FsmLuminosityDoCalc(FsmLuminosity *fsm);

#endif /* INC_FSM_LUMINOSITY_H_ */
