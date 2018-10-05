/*
 * kalman.h
 *
 *  Created on: 24 сент. 2018 г.
 *      Author: ziva
 */

#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

typedef struct {
    float error_measure;
    float error_estimate;
    float q;

    float current_estimate;
    float last_estimate;
    float kalman_gain;
} KalmanFilter;

void KalmanFilterInit(KalmanFilter *filter,
                      float measure_e,
                      float estimate_e,
                      float q);

float KalmanFilterUpdate(KalmanFilter *filter,
                         float measure);

#endif /* INC_KALMAN_H_ */
