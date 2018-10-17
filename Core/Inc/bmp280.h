/*
 * bmp280.h
 *
 *  Created on: 16 окт. 2018 г.
 *      Author: ziva
 */

#ifndef INC_BMP280_H_
#define INC_BMP280_H_

#include <stdint.h>

#include "kalman.h"
#include "stm32f4xx_hal.h"

#define CS_SET()        HAL_GPIO_WritePin(BMP280_NSS_GPIO_Port, BMP280_NSS_Pin, GPIO_PIN_RESET)
#define CS_RESET()      HAL_GPIO_WritePin(BMP280_NSS_GPIO_Port, BMP280_NSS_Pin, GPIO_PIN_SET)

#define BMP280_REGISTER_DIG_T1 0x88
#define BMP280_REGISTER_DIG_T2 0x8A
#define BMP280_REGISTER_DIG_T3 0x8C
#define BMP280_REGISTER_DIG_P1 0x8E
#define BMP280_REGISTER_DIG_P2 0x90
#define BMP280_REGISTER_DIG_P3 0x92
#define BMP280_REGISTER_DIG_P4 0x94
#define BMP280_REGISTER_DIG_P5 0x96
#define BMP280_REGISTER_DIG_P6 0x98
#define BMP280_REGISTER_DIG_P7 0x9A
#define BMP280_REGISTER_DIG_P8 0x9C
#define BMP280_REGISTER_DIG_P9 0x9E

typedef int32_t BMP280_S32_t;
typedef uint32_t BMP280_U32_t;

/*
 * BMP280 sensor
 */
typedef struct {
	SPI_HandleTypeDef *hspi;

	KalmanFilter pressure_filter; /* Atmospheric pressure filter */
	KalmanFilter temp_filter; /* Temperature filter */

	BMP280_S32_t t_fine;

	uint16_t dig_T1;
	int16_t  dig_T2;
	int16_t  dig_T3;

	uint16_t dig_P1;
	int16_t  dig_P2;
	int16_t  dig_P3;
	int16_t  dig_P4;
	int16_t  dig_P5;
	int16_t  dig_P6;
	int16_t  dig_P7;
	int16_t  dig_P8;
	int16_t  dig_P9;
} Bmp280;

void Bmp280Create(Bmp280 *bmp,
				  SPI_HandleTypeDef *hspi);

void Bmp280GetValues(Bmp280 *bmp,
					 double *t,
					 double *p);

#endif /* INC_BMP280_H_ */
