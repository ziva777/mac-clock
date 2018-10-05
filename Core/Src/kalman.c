/*
 * kalman.c
 *
 *  Created on: 24 сент. 2018 г.
 *      Author: ziva
 */

#ifndef SRC_KALMAN_C_
#define SRC_KALMAN_C_

#include "kalman.h"

#include <math.h>

void KalmanFilterInit(KalmanFilter *filter,
                      float measure_e,
                      float estimate_e,
                      float q)
{
    filter->error_measure = measure_e;
    filter->error_estimate = estimate_e;
    filter->q = q;
}

float KalmanFilterUpdate(KalmanFilter *filter,
                         float measure)
{
    filter->kalman_gain = filter->error_estimate
            / (filter->error_estimate + filter->error_measure);
    filter->current_estimate = filter->last_estimate
            + filter->kalman_gain * (measure - filter->last_estimate);

    filter->error_estimate = (1.0 - filter->kalman_gain)
            * filter->error_estimate
            + fabs(filter->last_estimate - filter->current_estimate)
                    * filter->q;

    filter->last_estimate = filter->current_estimate;

    return filter->current_estimate;
}

#endif /* SRC_KALMAN_C_ */
