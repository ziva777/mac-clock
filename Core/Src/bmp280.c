/*
 * bmp280.c
 *
 *  Created on: 16 окт. 2018 г.
 *      Author: ziva
 */

#include "bmp280.h"

static const uint32_t 	BMP280_READ_DELAY = 5000u;

void ReadCoefficients(Bmp280 *bmp)
{
    uint8_t data[6];
    uint8_t address[1];

    CS_SET();
    data[0] = 0xF4 & ~0x80;
    data[1] = 0b00100111;
    HAL_SPI_Transmit(bmp->hspi, data, 2, BMP280_READ_DELAY);
    CS_RESET();

    // cooeff's
    // T1
    CS_SET();
    address[0] = BMP280_REGISTER_DIG_T1 | 0x80;
    HAL_SPI_Transmit(bmp->hspi, address, 1, BMP280_READ_DELAY);
    HAL_SPI_Receive(bmp->hspi, (uint8_t *)&bmp->dig_T1, 2, BMP280_READ_DELAY);
    CS_RESET();

    // T2
    CS_SET();
    address[0] = BMP280_REGISTER_DIG_T2 | 0x80;
    HAL_SPI_Transmit(bmp->hspi, address, 1, BMP280_READ_DELAY);
    HAL_SPI_Receive(bmp->hspi, (uint8_t *)&bmp->dig_T2, 2, BMP280_READ_DELAY);
    CS_RESET();

    // T3
    CS_SET();
    address[0] = BMP280_REGISTER_DIG_T3 | 0x80;
    HAL_SPI_Transmit(bmp->hspi, address, 1, BMP280_READ_DELAY);
    HAL_SPI_Receive(bmp->hspi, (uint8_t *)&bmp->dig_T3, 2, BMP280_READ_DELAY);
    CS_RESET();

    // P1
    CS_SET();
    address[0] = BMP280_REGISTER_DIG_P1 | 0x80;
    HAL_SPI_Transmit(bmp->hspi, address, 1, BMP280_READ_DELAY);
    HAL_SPI_Receive(bmp->hspi, (uint8_t *)&bmp->dig_P1, 2, BMP280_READ_DELAY);
    CS_RESET();

    // P2
    CS_SET();
    address[0] = BMP280_REGISTER_DIG_P2 | 0x80;
    HAL_SPI_Transmit(bmp->hspi, address, 1, BMP280_READ_DELAY);
    HAL_SPI_Receive(bmp->hspi, (uint8_t *)&bmp->dig_P2, 2, BMP280_READ_DELAY);
    CS_RESET();

    // P3
    CS_SET();
    address[0] = BMP280_REGISTER_DIG_P3 | 0x80;
    HAL_SPI_Transmit(bmp->hspi, address, 1, BMP280_READ_DELAY);
    HAL_SPI_Receive(bmp->hspi, (uint8_t *)&bmp->dig_P3, 2, BMP280_READ_DELAY);
    CS_RESET();

    // P4
    CS_SET();
    address[0] = BMP280_REGISTER_DIG_P4 | 0x80;
    HAL_SPI_Transmit(bmp->hspi, address, 1, BMP280_READ_DELAY);
    HAL_SPI_Receive(bmp->hspi, (uint8_t *)&bmp->dig_P4, 2, BMP280_READ_DELAY);
    CS_RESET();

    // P5
    CS_SET();
    address[0] = BMP280_REGISTER_DIG_P5 | 0x80;
    HAL_SPI_Transmit(bmp->hspi, address, 1, BMP280_READ_DELAY);
    HAL_SPI_Receive(bmp->hspi, (uint8_t *)&bmp->dig_P5, 2, BMP280_READ_DELAY);
    CS_SET();
    CS_RESET();

    // P6
    CS_SET();
    address[0] = BMP280_REGISTER_DIG_P6 | 0x80;
    HAL_SPI_Transmit(bmp->hspi, address, 1, BMP280_READ_DELAY);
    HAL_SPI_Receive(bmp->hspi, (uint8_t *)&bmp->dig_P6, 2, BMP280_READ_DELAY);
    CS_RESET();

    // P7
    CS_SET();
    address[0] = BMP280_REGISTER_DIG_P7 | 0x80;
    HAL_SPI_Transmit(bmp->hspi, address, 1, BMP280_READ_DELAY);
    HAL_SPI_Receive(bmp->hspi, (uint8_t *)&bmp->dig_P7, 2, BMP280_READ_DELAY);
    CS_RESET();

    // P8
    CS_SET();
    address[0] = BMP280_REGISTER_DIG_P8 | 0x80;
    HAL_SPI_Transmit(bmp->hspi, address, 1, BMP280_READ_DELAY);
    HAL_SPI_Receive(bmp->hspi, (uint8_t *)&bmp->dig_P8, 2, BMP280_READ_DELAY);
    CS_RESET();

    // P9
    CS_SET();
    address[0] = BMP280_REGISTER_DIG_P9 | 0x80;
    HAL_SPI_Transmit(bmp->hspi, address, 1, BMP280_READ_DELAY);
    HAL_SPI_Receive(bmp->hspi, (uint8_t *)&bmp->dig_P9, 2, BMP280_READ_DELAY);
    CS_RESET();
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value
static
BMP280_S32_t bmp280_compensate_T_int32(Bmp280 *bmp, BMP280_S32_t adc_T)
{
    BMP280_S32_t var1, var2, T;

    var1 = ((((adc_T>>3) - ((BMP280_S32_t)bmp->dig_T1<<1))) * ((BMP280_S32_t)bmp->dig_T2)) >> 11;
    var2 = (((((adc_T>>4) - ((BMP280_S32_t)bmp->dig_T1)) * ((adc_T>>4) - ((BMP280_S32_t)bmp->dig_T1))) >> 12) * ((BMP280_S32_t)bmp->dig_T3)) >> 14;
    bmp->t_fine = var1 + var2;
    T = (bmp->t_fine * 5 + 128) >> 8;
    return T;
}

// Returns pressure in Pa as unsigned 32 bit integer. Output value of “96386” equals 96386 Pa = 963.86 hPa
static
BMP280_U32_t bmp280_compensate_P_int32(Bmp280 *bmp, BMP280_S32_t adc_P)
{
    BMP280_S32_t var1, var2;
    BMP280_U32_t p;

    var1 = (((BMP280_S32_t)bmp->t_fine)>>1) - (BMP280_S32_t)64000;
    var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((BMP280_S32_t)bmp->dig_P6);
    var2 = var2 + ((var1*((BMP280_S32_t)bmp->dig_P5))<<1);
    var2 = (var2>>2)+(((BMP280_S32_t)bmp->dig_P4)<<16);
    var1 = (((bmp->dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((BMP280_S32_t)bmp->dig_P2) * var1)>>1))>>18;
    var1 =((((32768+var1))*((BMP280_S32_t)bmp->dig_P1))>>15);

    if (var1 == 0) {
        return 0; // avoid exception caused by division by zero
    }

    p = (((BMP280_U32_t)(((BMP280_S32_t)1048576)-adc_P)-(var2>>12)))*3125;

    if (p < 0x80000000) {
        p = (p << 1) / ((BMP280_U32_t)var1);
    } else {
        p = (p / (BMP280_U32_t)var1) * 2;
    }

    var1 = (((BMP280_S32_t)bmp->dig_P9) * ((BMP280_S32_t)(((p>>3) * (p>>3))>>13)))>>12;
    var2 = (((BMP280_S32_t)(p>>2)) * ((BMP280_S32_t)bmp->dig_P8))>>13;
    p = (BMP280_U32_t)((BMP280_S32_t)p + ((var1 + var2 + bmp->dig_P7) >> 4));
    return p;
}

void Bmp280Create(Bmp280 *bmp,
				  SPI_HandleTypeDef *hspi)
{
	bmp->hspi = hspi;

	ReadCoefficients(bmp);

	KalmanFilterInit(&bmp->pressure_filter, 4.0f, 4.0f, 0.0250f);
	KalmanFilterInit(&bmp->temp_filter, 4.0f, 4.0f, 0.0250f);
}

void Bmp280GetValues(Bmp280 *bmp,
	      			 double *t,
					 double *p)
{
	const static double to_mmHg = 0.00750062;

	uint8_t data[6];
	uint8_t address[1];

	// adc values
	CS_SET();
	address[0] = 0xF7 | 0x80;
	HAL_SPI_Transmit(bmp->hspi, address, 1, BMP280_READ_DELAY);
	HAL_SPI_Receive(bmp->hspi, data, 6, BMP280_READ_DELAY);
	CS_RESET();

	uint8_t adc_P_msb = data[0];
	uint8_t adc_P_lsb = data[1];
	uint8_t adc_P_xlsb = data[2];

	uint8_t adc_T_msb = data[3];
	uint8_t adc_T_lsb = data[4];
	uint8_t adc_T_xlsb = data[5];

	BMP280_S32_t adc_T = ((adc_T_msb << 16) | (adc_T_lsb << 8) | adc_T_xlsb) >> 4;
	BMP280_S32_t adc_P = ((adc_P_msb << 16) | (adc_P_lsb << 8) | adc_P_xlsb) >> 4;

	BMP280_S32_t T, P;

	T = bmp280_compensate_T_int32(bmp, adc_T);
	P = bmp280_compensate_P_int32(bmp, adc_P);

	*t = KalmanFilterUpdate(&bmp->temp_filter, (float)T);
	*p = KalmanFilterUpdate(&bmp->pressure_filter, (float)P);

	*p *= to_mmHg;
}
