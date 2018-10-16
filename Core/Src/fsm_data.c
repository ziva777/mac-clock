/*
 * fsm_data.c
 *
 *  Created on: 16 окт. 2018 г.
 *      Author: ziva
 */

#include "fsm_data.h"

#include <string.h>

#include "i2c.h"

#include "aux.h"
#include "wire.h"
#include "flash.h"
#include "ws2812.h"
#include "display.h"
#include "display_aux.h"

typedef int32_t BMP280_S32_t;
typedef uint32_t BMP280_U32_t;

BMP280_S32_t t_fine;
uint16_t dig_T1 = 27936;
int16_t  dig_T2 = 26726;
int16_t  dig_T3 = 50;

uint16_t dig_P1 = 38285;
int16_t  dig_P2 = -10543;
int16_t  dig_P3 = 3024;
int16_t  dig_P4 = 8470;
int16_t  dig_P5 = -110;
int16_t  dig_P6 = -7;
int16_t  dig_P7 = 15500;
int16_t  dig_P8 = -14600;
int16_t  dig_P9 = 6000;

static const uint32_t 	CYCLE_DELAY = 10u;
static const uint32_t 	BTN_REPEAT_DELAY = 1000u;
static const uint32_t 	BTN_FAST_REPEAT_DELAY = 850u;
static const uint32_t 	BMP280_READ_DELAY = 5000u;

extern Display display;

typedef
FSM_DATA_MODES Transition(FsmData *fsm);

/* Transition table */
Transition * const STATES_TABLE[FSM_DATA_MODES_NUM] = {
    FsmData_Do_MODE_A1_1,
    FsmData_Do_MODE_A1_2,
    FsmData_Do_MODE_A1_3,
    FsmData_Do_MODE_S1_1,
    FsmData_Do_MODE_S1_2,
    FsmData_Do_MODE_P1_1,
    FsmData_Do_MODE_EDIT_TIME_1,
    FsmData_Do_MODE_EDIT_TIME_2,
    FsmData_Do_MODE_EDIT_AGING,
    FsmData_Do_MODE_EDIT_LATITUDE,
    FsmData_Do_MODE_EDIT_LONGITUDE,
    FsmData_Do_MODE_EDIT_TIMEZONE,
    FsmData_Do_MODE_EDIT_DATE,
};

FSM_DATA_MODES process_state(FSM_DATA_MODES curr_state, FsmData *state_data) {
    return STATES_TABLE[curr_state](state_data);
}

/* Private */

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value
BMP280_S32_t bmp280_compensate_T_int32(BMP280_S32_t adc_T)
{
    BMP280_S32_t var1, var2, T;

    var1 = ((((adc_T>>3) - ((BMP280_S32_t)dig_T1<<1))) * ((BMP280_S32_t)dig_T2)) >> 11;
    var2 = (((((adc_T>>4) - ((BMP280_S32_t)dig_T1)) * ((adc_T>>4) - ((BMP280_S32_t)dig_T1))) >> 12) * ((BMP280_S32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

// Returns pressure in Pa as unsigned 32 bit integer. Output value of “96386” equals 96386 Pa = 963.86 hPa
BMP280_U32_t bmp280_compensate_P_int32(BMP280_S32_t adc_P)
{
    BMP280_S32_t var1, var2;
    BMP280_U32_t p;

    var1 = (((BMP280_S32_t)t_fine)>>1) - (BMP280_S32_t)64000;
    var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((BMP280_S32_t)dig_P6);
    var2 = var2 + ((var1*((BMP280_S32_t)dig_P5))<<1);
    var2 = (var2>>2)+(((BMP280_S32_t)dig_P4)<<16);
    var1 = (((dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((BMP280_S32_t)dig_P2) * var1)>>1))>>18;
    var1 =((((32768+var1))*((BMP280_S32_t)dig_P1))>>15);

    if (var1 == 0) {
        return 0; // avoid exception caused by division by zero
    }

    p = (((BMP280_U32_t)(((BMP280_S32_t)1048576)-adc_P)-(var2>>12)))*3125;

    if (p < 0x80000000) {
        p = (p << 1) / ((BMP280_U32_t)var1);
    } else {
        p = (p / (BMP280_U32_t)var1) * 2;
    }

    var1 = (((BMP280_S32_t)dig_P9) * ((BMP280_S32_t)(((p>>3) * (p>>3))>>13)))>>12;
    var2 = (((BMP280_S32_t)(p>>2)) * ((BMP280_S32_t)dig_P8))>>13;
    p = (BMP280_U32_t)((BMP280_S32_t)p + ((var1 + var2 + dig_P7) >> 4));
    return p;
}

// Returns temperature in DegC, double precision. Output value of “51.23” equals 51.23 DegC.
// t_fine carries fine temperature as global value
double bmp280_compensate_T_double(BMP280_S32_t adc_T)
{
	double var1, var2, T;
	var1 = (((double) adc_T) / 16384.0 - ((double)dig_T1)/1024.0) * ((double)dig_T2);
	var2 = ((((double) adc_T) / 131072.0 - ((double)dig_T1)/8192.0) *
	(((double)adc_T)/131072.0 - ((double) dig_T1)/8192.0)) * ((double)dig_T3);
	t_fine = (BMP280_S32_t) (var1 + var2);
	T = (var1 + var2) / 5120.0;
	return T;
}

// Returns pressure in Pa as double. Output value of “96386.2” equals 96386.2 Pa = 963.862 hPa
double bmp280_compensate_P_double(BMP280_S32_t adc_P)
{
	double var1, var2, p;
	var1 = ((double) t_fine / 2.0) - 64000.0;
	var2 = var1 * var1 * ((double) dig_P6) / 32768.0;
	var2 = var2 + var1 * ((double) dig_P5) * 2.0;
	var2 = (var2 / 4.0) + (((double) dig_P4) * 65536.0);
	var1 = (((double) dig_P3) * var1 * var1 / 524288.0
			+ ((double) dig_P2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0) * ((double) dig_P1);
	if (var1 == 0.0) {
		return 0; // avoid exception caused by division by zero
	}
	p = 1048576.0 - (double)adc_P;
	p = (p - (var2 / 4096.0)) * 6250.0 / var1;
	var1 = ((double) dig_P9) * p * p / 2147483648.0;
	var2 = p * ((double) dig_P8) / 32768.0;
	p = p + (var1 + var2 + ((double) dig_P7)) / 16.0;
	return p;
}

void FsmData_ReadBmp280Coefficients()
{
	uint8_t data[6];
	uint8_t address[1];

	CS_SET();
	data[0] = 0xF4 & ~0x80;
	data[1] = 0b00100111;
	HAL_SPI_Transmit(&hspi3, data, 2, BMP280_READ_DELAY);
	CS_RESET();

	// cooeff's
	// T1
	CS_SET();
	address[0] = BMP280_REGISTER_DIG_T1 | 0x80;
	HAL_SPI_Transmit(&hspi3, address, 1, BMP280_READ_DELAY);
	HAL_SPI_Receive(&hspi3, (uint8_t *)&dig_T1, 2, BMP280_READ_DELAY);
	CS_RESET();

	// T2
	CS_SET();
	address[0] = BMP280_REGISTER_DIG_T2 | 0x80;
	HAL_SPI_Transmit(&hspi3, address, 1, BMP280_READ_DELAY);
	HAL_SPI_Receive(&hspi3, (uint8_t *)&dig_T2, 2, BMP280_READ_DELAY);
	CS_RESET();

	// T3
	CS_SET();
	address[0] = BMP280_REGISTER_DIG_T3 | 0x80;
	HAL_SPI_Transmit(&hspi3, address, 1, BMP280_READ_DELAY);
	HAL_SPI_Receive(&hspi3, (uint8_t *)&dig_T3, 2, BMP280_READ_DELAY);
	CS_RESET();

	// P1
	CS_SET();
	address[0] = BMP280_REGISTER_DIG_P1 | 0x80;
	HAL_SPI_Transmit(&hspi3, address, 1, BMP280_READ_DELAY);
	HAL_SPI_Receive(&hspi3, (uint8_t *)&dig_P1, 2, BMP280_READ_DELAY);
	CS_RESET();

	// P2
	CS_SET();
	address[0] = BMP280_REGISTER_DIG_P2 | 0x80;
	HAL_SPI_Transmit(&hspi3, address, 1, BMP280_READ_DELAY);
	HAL_SPI_Receive(&hspi3, (uint8_t *)&dig_P2, 2, BMP280_READ_DELAY);
	CS_RESET();

	// P3
	CS_SET();
	address[0] = BMP280_REGISTER_DIG_P3 | 0x80;
	HAL_SPI_Transmit(&hspi3, address, 1, BMP280_READ_DELAY);
	HAL_SPI_Receive(&hspi3, (uint8_t *)&dig_P3, 2, BMP280_READ_DELAY);
	CS_RESET();

	// P4
	CS_SET();
	address[0] = BMP280_REGISTER_DIG_P4 | 0x80;
	HAL_SPI_Transmit(&hspi3, address, 1, BMP280_READ_DELAY);
	HAL_SPI_Receive(&hspi3, (uint8_t *)&dig_P4, 2, BMP280_READ_DELAY);
	CS_RESET();

	// P5
	CS_SET();
	address[0] = BMP280_REGISTER_DIG_P5 | 0x80;
	HAL_SPI_Transmit(&hspi3, address, 1, BMP280_READ_DELAY);
	HAL_SPI_Receive(&hspi3, (uint8_t *)&dig_P5, 2, BMP280_READ_DELAY);
	CS_SET();
	CS_RESET();

	// P6
	CS_SET();
	address[0] = BMP280_REGISTER_DIG_P6 | 0x80;
	HAL_SPI_Transmit(&hspi3, address, 1, BMP280_READ_DELAY);
	HAL_SPI_Receive(&hspi3, (uint8_t *)&dig_P6, 2, BMP280_READ_DELAY);
	CS_RESET();

	// P7
	CS_SET();
	address[0] = BMP280_REGISTER_DIG_P7 | 0x80;
	HAL_SPI_Transmit(&hspi3, address, 1, BMP280_READ_DELAY);
	HAL_SPI_Receive(&hspi3, (uint8_t *)&dig_P7, 2, BMP280_READ_DELAY);
	CS_RESET();

	// P8
	CS_SET();
	address[0] = BMP280_REGISTER_DIG_P8 | 0x80;
	HAL_SPI_Transmit(&hspi3, address, 1, BMP280_READ_DELAY);
	HAL_SPI_Receive(&hspi3, (uint8_t *)&dig_P8, 2, BMP280_READ_DELAY);
	CS_RESET();

	// P9
	CS_SET();
	address[0] = BMP280_REGISTER_DIG_P9 | 0x80;
	HAL_SPI_Transmit(&hspi3, address, 1, BMP280_READ_DELAY);
	HAL_SPI_Receive(&hspi3, (uint8_t *)&dig_P9, 2, BMP280_READ_DELAY);
	CS_RESET();
}

void FsmDataValidateTs(FsmData *fsm)
{
    if (fsm->ts.mday == 0 || fsm->ts.mon == 0 || fsm->ts.year < 2015) {
        /* ... */
        fsm->ts.mday = 1;
        fsm->ts.mon = 1;
        fsm->ts.year = 2015;

        fsm->ts.hour = 0;
        fsm->ts.min = 0;
        fsm->ts.sec = 0;
        Rtc_DS3231_set(fsm->ts);
    }
}

/* Public */

void FsmDataCreate(FsmData *fsm)
{
    memset(&fsm->ts, 0, sizeof(fsm->ts));
    memset(&fsm->sun_rise1, 0, sizeof(fsm->sun_rise1));
    memset(&fsm->sun_set1, 0, sizeof(fsm->sun_set1));
    memset(&fsm->sun_rise2, 0, sizeof(fsm->sun_rise2));
    memset(&fsm->sun_set2, 0, sizeof(fsm->sun_set2));

    fsm->update_time = 0;
    fsm->update_aging = 0;
    fsm->update_latitude = 0;
    fsm->update_longitude = 0;
    fsm->update_timezone = 0;
    fsm->update_celestial = 0;

    fsm->btn1_state_prev = fsm->btn1_state_curr = BTN1_STATE();
    fsm->btn2_state_prev = fsm->btn2_state_curr = BTN2_STATE();
    fsm->btn3_state_prev = fsm->btn3_state_curr = BTN3_STATE();
    fsm->btn4_state_prev = fsm->btn4_state_curr = BTN4_STATE();
    fsm->btn5_state_prev = fsm->btn5_state_curr = BTN5_STATE();
    fsm->btn6_state_prev = fsm->btn6_state_curr = BTN6_STATE();
    fsm->btn7_state_prev = fsm->btn7_state_curr = BTN7_STATE();
    fsm->btn8_state_prev = fsm->btn8_state_curr = BTN8_STATE();

    fsm->mode_prev =
        fsm->mode_curr =
            FSM_DATA_MODE_A1_1;

    fsm->btn6_pressed_counter = 0;
    fsm->btn7_pressed_counter = 0;
    fsm->btn8_pressed_counter = 0;

    fsm->btn7_fast_repeat_delay_accelerated = BTN_FAST_REPEAT_DELAY;
    fsm->btn8_fast_repeat_delay_accelerated = BTN_FAST_REPEAT_DELAY;

    {
        /* Current timestamp */
        Wire_SetI2C(&hi2c1);
        Rtc_DS3231_init(DS3231_INTCN);
        Rtc_DS3231_get(&fsm->ts);
        FsmDataValidateTs(fsm);

        fsm->aging = Rtc_DS3231_get_aging();
    }

    {
        /* Observation point */
        uint32_t idx = FlashReadTZ();
        double latitude_deg2 = FlashReadLatitude();
        double longitude_deg2 = FlashReadLongitude();

		if (idx >= 0 && idx < N_TIMEZONES) {
			fsm->tz_idx = (uint8_t)idx;
			fsm->tz = TIMEZONES[fsm->tz_idx];
		} else {
			fsm->tz_idx = MSK_ZONE;
			fsm->tz = TIMEZONES[fsm->tz_idx];
		}

		if ((latitude_deg2 > -90.0 && latitude_deg2 < +90.0) &&
				(latitude_deg2 > -180.0 && latitude_deg2 < +180.0))
		{
			fsm->latitude_deg = latitude_deg2;
			fsm->longitude_deg = longitude_deg2;
		} else {
			fsm->latitude_deg = DEFAULT_LATITUDE;
			fsm->longitude_deg = DEFAULT_LONGITUDE;
		}

        fsm->latitude_rad = fsm->latitude_deg * snm_DEG_TO_RAD;
        fsm->longitude_rad = fsm->longitude_deg * snm_DEG_TO_RAD;
    }

    {
        /* Sun & moon calculator */
        snm_CalculatorCreate(&fsm->calc);
        snm_CalculatorSetPoint(&fsm->calc,
                               fsm->latitude_deg,
                               fsm->longitude_rad);
    }

    FsmData_ReadBmp280Coefficients();
    KalmanFilterInit(&fsm->pressure_filter,
    					 4.0f,
    					 4.0f,
    					 0.0250f);
}

void FsmDataCalcCelestial(FsmData *fsm)
{
    if (fsm->sun_rise1.mday != fsm->ts.mday || fsm->update_celestial) {
        struct tm rise, set;

        snm_CalculatorSetTime(&fsm->calc,
                fsm->ts.mday, fsm->ts.mon, fsm->ts.year);
        snm_CalculatorSetDate(&fsm->calc,
                fsm->ts.hour, fsm->ts.min, fsm->ts.sec);
        snm_CalculatorSetPoint(&fsm->calc,
                fsm->latitude_rad, fsm->longitude_rad);

        snm_CalculatorSetTwilight(&fsm->calc, HORIZON_34arcmin);
        snm_CalculatorCalc(&fsm->calc);
        rise = snm_CalculatorGetDateAsTm(fsm->calc.sun_rise, fsm->tz);
        set = snm_CalculatorGetDateAsTm(fsm->calc.sun_set, fsm->tz);

        fsm->sun_rise1 = GetTimestampFromTm(rise);
        fsm->sun_set1 = GetTimestampFromTm(set);

        snm_CalculatorSetTwilight(&fsm->calc, TWILIGHT_CIVIL);
        snm_CalculatorCalc(&fsm->calc);
        rise = snm_CalculatorGetDateAsTm(fsm->calc.sun_rise, fsm->tz);
        set = snm_CalculatorGetDateAsTm(fsm->calc.sun_set, fsm->tz);

        fsm->sun_rise2 = GetTimestampFromTm(rise);
        fsm->sun_set2 = GetTimestampFromTm(set);
        fsm->update_celestial = 0;
    }
}

void FsmDataSetBacklight(FsmData *fsm)
{
    if ((fsm->ts.hour == 0 && fsm->ts.min == 0 && fsm->ts.sec == 0)
            || (fsm->ts.hour == fsm->sun_rise2.hour &&
                fsm->ts.min < fsm->sun_rise2.min))
    {
        SetBacklight(255, 0, 0); // ночь
    } else if ((fsm->ts.hour > fsm->sun_set2.hour)
            || (fsm->ts.hour == fsm->sun_set2.hour &&
                fsm->ts.min > fsm->sun_set2.min))
    {
        SetBacklight(255, 0, 0); // ночь
    } else {
        if ((fsm->ts.hour < fsm->sun_rise1.hour)
                || (fsm->ts.hour == fsm->sun_rise1.hour &&
                    fsm->ts.min < fsm->sun_rise1.min))
        {
            SetBacklight(255, 127, 0); // сумерки
        } else if ((fsm->ts.hour > fsm->sun_set1.hour)
                || (fsm->ts.hour == fsm->sun_set1.hour &&
                    fsm->ts.min > fsm->sun_set1.min))
        {
            SetBacklight(255, 127, 0); // сумерки
        } else {
            SetBacklight(255, 255, 255); // день
        }
    }
}

void FsmDataButtonsUpdateState(FsmData *fsm)
{
    fsm->btn1_state_curr = BTN1_STATE();
    fsm->btn2_state_curr = BTN2_STATE();
    fsm->btn3_state_curr = BTN3_STATE();
    fsm->btn4_state_curr = BTN4_STATE();
    fsm->btn5_state_curr = BTN5_STATE();
    fsm->btn6_state_curr = BTN6_STATE();
    fsm->btn7_state_curr = BTN7_STATE();
    fsm->btn8_state_curr = BTN8_STATE();
}

FSM_DATA_MODES FsmData_Do_MODE_A1_1(FsmData *fsm)
{
    Rtc_DS3231_get(&fsm->ts);
    Display_A1_1(&display, &fsm->ts);

    if (fsm->btn1_state_prev != fsm->btn1_state_curr) {
        fsm->btn1_state_prev = fsm->btn1_state_curr;
        fsm->mode_prev = fsm->mode_curr;
        fsm->mode_curr = FSM_DATA_MODE_EDIT_TIME_1;
        Beep();
    } else if (fsm->btn2_state_prev != fsm->btn2_state_curr) {
        fsm->btn2_state_prev = fsm->btn2_state_curr;

        if (fsm->btn2_state_curr == BTN_DOWN) {
            fsm->mode_prev = fsm->mode_curr;
            fsm->mode_curr = FSM_DATA_MODE_A1_2;
            Beep();
        }
    } else if (fsm->btn8_state_prev != fsm->btn8_state_curr || fsm->btn8_pressed_counter > BTN_REPEAT_DELAY) {
        fsm->btn8_pressed_counter = BTN_FAST_REPEAT_DELAY;
        fsm->btn8_state_prev = fsm->btn8_state_curr;

        if (fsm->btn8_state_curr == BTN_DOWN) {
        }
    } else if (fsm->btn7_state_prev != fsm->btn7_state_curr || fsm->btn7_pressed_counter > BTN_REPEAT_DELAY) {
        fsm->btn7_pressed_counter = BTN_FAST_REPEAT_DELAY;
        fsm->btn7_state_prev = fsm->btn7_state_curr;

        if (fsm->btn7_state_curr == BTN_DOWN) {
        }
    } else if (fsm->btn3_state_prev != fsm->btn3_state_curr) {
        fsm->btn3_state_prev = fsm->btn3_state_curr;

        if (fsm->btn3_state_curr == BTN_DOWN) {
            fsm->mode_prev = fsm->mode_curr;
            fsm->mode_curr = FSM_DATA_MODE_S1_1;
            Beep();
        }
    } else if (fsm->btn4_state_prev != fsm->btn4_state_curr) {
        if (fsm->btn4_state_curr == BTN_DOWN) {
            fsm->mode_prev = fsm->mode_curr;
            fsm->mode_curr = FSM_DATA_MODE_P1_1;
            Beep();
        }
    }

    return fsm->mode_curr;
}

FSM_DATA_MODES FsmData_Do_MODE_A1_2(FsmData *fsm)
{
    Rtc_DS3231_get(&fsm->ts);
    Display_A1_2(&display, &fsm->ts);

    {
        if (fsm->btn1_state_prev != fsm->btn1_state_curr) {
            fsm->btn1_state_prev = fsm->btn1_state_curr;
            fsm->mode_prev = fsm->mode_curr;
            fsm->mode_curr = FSM_DATA_MODE_EDIT_TIME_2;
            Beep();
        } else if (fsm->btn2_state_prev != fsm->btn2_state_curr) {
            fsm->btn2_state_prev = fsm->btn2_state_curr;

            if (fsm->btn2_state_curr == BTN_DOWN) {
                fsm->mode_prev = fsm->mode_curr;
                fsm->mode_curr = FSM_DATA_MODE_A1_3;
                Beep();
            }
        }
    }

    return fsm->mode_curr;
}

FSM_DATA_MODES FsmData_Do_MODE_A1_3(FsmData *fsm)
{
    Rtc_DS3231_get(&fsm->ts);
    Display_A1_3(&display, &fsm->ts);

    {
        if (fsm->btn1_state_prev != fsm->btn1_state_curr) {
            fsm->btn1_state_prev = fsm->btn1_state_curr;
            fsm->mode_prev = fsm->mode_curr;
            fsm->mode_curr = FSM_DATA_MODE_EDIT_DATE;
            Beep();
        } else if (fsm->btn2_state_prev != fsm->btn2_state_curr) {
            fsm->btn2_state_prev = fsm->btn2_state_curr;

            if (fsm->btn2_state_curr == BTN_DOWN) {
                fsm->mode_prev = fsm->mode_curr;
                fsm->mode_curr = FSM_DATA_MODE_A1_1;
                Beep();
            }
        }
    }
    return fsm->mode_curr;
}

FSM_DATA_MODES FsmData_Do_MODE_S1_1(FsmData *fsm)
{
    static uint32_t counter;
    static uint8_t flip_flop;
    uint32_t counter_curr;

    if (counter == 0) {
        counter = HAL_GetTick();
    }

    counter_curr = HAL_GetTick();

    if (counter_curr - counter > 1000u) {
        flip_flop = 1;
        counter = 0;
    }

    if (flip_flop == 0) {
        Display_S1_1_Msg(&display);
    } else {
        Display_S1_1(&display, &fsm->sun_rise1);
    }

    if (fsm->btn3_state_prev != fsm->btn3_state_curr) {
        if (fsm->btn3_state_curr == BTN_DOWN) {
            counter = 0;
            flip_flop = 0;
            fsm->mode_curr = FSM_DATA_MODE_S1_2;
        }

        fsm->btn3_state_prev = fsm->btn3_state_curr;
    }

    if (fsm->btn1_state_prev != fsm->btn1_state_curr
        || fsm->btn2_state_prev != fsm->btn2_state_curr
        || fsm->btn4_state_prev != fsm->btn4_state_curr
        || fsm->btn5_state_prev != fsm->btn5_state_curr
        || fsm->btn6_state_prev != fsm->btn6_state_curr
        || fsm->btn7_state_prev != fsm->btn7_state_curr
        || fsm->btn8_state_prev != fsm->btn8_state_curr)
    {
        counter = 0;
        flip_flop = 0;
        fsm->mode_curr = fsm->mode_prev;
        fsm->btn2_state_prev = fsm->btn2_state_curr;
    }

    return fsm->mode_curr;
}

FSM_DATA_MODES FsmData_Do_MODE_S1_2(FsmData *fsm)
{
    static uint32_t counter;
    static uint8_t flip_flop;
    uint32_t counter_curr;

    if (counter == 0) {
        counter = HAL_GetTick();
    }

    counter_curr = HAL_GetTick();

    if (counter_curr - counter > 1000u) {
        flip_flop = 1;
        counter = 0;
    }

    if (flip_flop == 0) {
        Display_S2_1_Msg(&display);
    } else {
        Display_S1_1(&display, &fsm->sun_set1);
    }

    if (fsm->btn3_state_prev != fsm->btn3_state_curr) {
        if (fsm->btn3_state_curr == BTN_DOWN) {
            counter = 0;
            flip_flop = 0;
            fsm->mode_curr = FSM_DATA_MODE_S1_1;
        }

        fsm->btn3_state_prev = fsm->btn3_state_curr;
    }

    if (fsm->btn1_state_prev != fsm->btn1_state_curr
        || fsm->btn2_state_prev != fsm->btn2_state_curr
        || fsm->btn4_state_prev != fsm->btn4_state_curr
        || fsm->btn5_state_prev != fsm->btn5_state_curr
        || fsm->btn6_state_prev != fsm->btn6_state_curr
        || fsm->btn7_state_prev != fsm->btn7_state_curr
        || fsm->btn8_state_prev != fsm->btn8_state_curr)
    {
        counter = 0;
        flip_flop = 0;
        fsm->mode_curr = fsm->mode_prev;
        fsm->btn2_state_prev = fsm->btn2_state_curr;
    }

    return fsm->mode_curr;
}

FSM_DATA_MODES FsmData_Do_MODE_P1_1(FsmData *fsm)
{
	const static double to_mmHg = 0.00750062;
	const static uint32_t counter_bound = 100u;
	static uint32_t counter = counter_bound;

//    if (--counter == 0) {
	{
    	uint16_t pressure = 0;
        uint8_t data[6];
        uint8_t address[1];

        // adc values
        CS_SET();
        address[0] = 0xF7 | 0x80;
        HAL_SPI_Transmit(&hspi3, address, 1, BMP280_READ_DELAY);
        HAL_SPI_Receive(&hspi3, data, 6, BMP280_READ_DELAY);
        CS_RESET();

        uint8_t adc_P_msb = data[0];
        uint8_t adc_P_lsb = data[1];
        uint8_t adc_P_xlsb = data[2];

        uint8_t adc_T_msb = data[3];
        uint8_t adc_T_lsb = data[4];
        uint8_t adc_T_xlsb = data[5];

        BMP280_S32_t adc_T = ((adc_T_msb << 16) | (adc_T_lsb << 8) | adc_T_xlsb) >> 4;
        BMP280_S32_t adc_P = ((adc_P_msb << 16) | (adc_P_lsb << 8) | adc_P_xlsb) >> 4;

        BMP280_S32_t t, p;

        t = bmp280_compensate_T_int32(adc_T);
        p = bmp280_compensate_P_int32(adc_P);

        (void)t;
		p = KalmanFilterUpdate(&fsm->pressure_filter, (float)p);

        double tmp = p;
        tmp *= to_mmHg;
        tmp *= 10.0;

        pressure = tmp;
        counter = counter_bound;
        Display_P1(&display, pressure);
    }

    if (fsm->btn1_state_prev != fsm->btn1_state_curr
        || fsm->btn2_state_prev != fsm->btn2_state_curr
        || fsm->btn3_state_prev != fsm->btn3_state_curr
        || fsm->btn5_state_prev != fsm->btn5_state_curr
        || fsm->btn6_state_prev != fsm->btn6_state_curr
        || fsm->btn7_state_prev != fsm->btn7_state_curr
        || fsm->btn8_state_prev != fsm->btn8_state_curr)
    {
        fsm->mode_curr = fsm->mode_prev;
        fsm->btn2_state_prev = fsm->btn2_state_curr;
    }

    return fsm->mode_curr;
}

FSM_DATA_MODES FsmData_Do_MODE_EDIT_TIME_1(FsmData *fsm)
{
    Display_EditTime1(&display, &fsm->ts);

    if (fsm->btn1_state_prev != fsm->btn1_state_curr) {
        fsm->btn1_state_prev = fsm->btn1_state_curr;

        if (fsm->btn1_state_curr == BTN_UP) {
            if (fsm->update_time) {
                Rtc_DS3231_set(fsm->ts);
                fsm->update_time = 0;
                fsm->update_celestial = 1;
            }

            fsm->mode_curr = fsm->mode_prev;
        }
    } else if (fsm->btn8_state_prev != fsm->btn8_state_curr || fsm->btn8_pressed_counter > BTN_REPEAT_DELAY) {
        if (fsm->btn8_state_curr == BTN_DOWN) {
            fsm->ts.sec += 1;
            fsm->ts.sec %= 60;
            fsm->update_time = 1;
        }

        fsm->btn8_pressed_counter = BTN_FAST_REPEAT_DELAY;
        fsm->btn8_state_prev = fsm->btn8_state_curr;
    } else if (fsm->btn7_state_prev != fsm->btn7_state_curr || fsm->btn7_pressed_counter > BTN_REPEAT_DELAY) {
        if (fsm->btn7_state_curr == BTN_DOWN) {
            fsm->ts.min += 1;
            fsm->ts.min %= 60;
            fsm->update_time = 1;
        }

        fsm->btn7_pressed_counter = BTN_FAST_REPEAT_DELAY;
        fsm->btn7_state_prev = fsm->btn7_state_curr;
    } else if (fsm->btn6_state_prev != fsm->btn6_state_curr || fsm->btn6_pressed_counter > BTN_REPEAT_DELAY) {
        if (fsm->btn6_state_curr == BTN_DOWN) {
            fsm->ts.hour += 1;
            fsm->ts.hour %= 24;
            fsm->update_time = 1;
        }

        fsm->btn6_pressed_counter = BTN_FAST_REPEAT_DELAY;
        fsm->btn6_state_prev = fsm->btn6_state_curr;
    } else if (fsm->btn2_state_prev != fsm->btn2_state_curr) {
        if (fsm->btn2_state_curr == BTN_DOWN) {
            fsm->mode_curr = FSM_DATA_MODE_EDIT_AGING;
        }

        fsm->btn2_state_prev = fsm->btn2_state_curr;
    }

    return fsm->mode_curr;
}

FSM_DATA_MODES FsmData_Do_MODE_EDIT_TIME_2(FsmData *fsm)
{
    Display_EditTime2(&display, &fsm->ts);

    if (fsm->btn1_state_prev != fsm->btn1_state_curr) {
        fsm->btn1_state_prev = fsm->btn1_state_curr;

        if (fsm->btn1_state_curr == BTN_UP) {
            if (fsm->update_time) {
                Rtc_DS3231_set(fsm->ts);
                fsm->update_time = 0;
                fsm->update_celestial = 1;
            }

            fsm->mode_curr = fsm->mode_prev;
        }
    } else if (fsm->btn7_state_prev != fsm->btn7_state_curr || fsm->btn7_pressed_counter > BTN_REPEAT_DELAY) {
        if (fsm->btn7_state_curr == BTN_DOWN) {
            fsm->ts.sec = 0;
            fsm->ts.min += 1;
            fsm->ts.min %= 60;
            fsm->update_time = 1;
        }

        fsm->btn7_pressed_counter = BTN_FAST_REPEAT_DELAY;
        fsm->btn7_state_prev = fsm->btn7_state_curr;
    } else if (fsm->btn6_state_prev != fsm->btn6_state_curr || fsm->btn6_pressed_counter > BTN_REPEAT_DELAY) {
        if (fsm->btn6_state_curr == BTN_DOWN) {
            fsm->ts.sec = 0;
            fsm->ts.hour += 1;
            fsm->ts.hour %= 23;
            fsm->update_time = 1;
        }

        fsm->btn6_pressed_counter = BTN_FAST_REPEAT_DELAY;
        fsm->btn6_state_prev = fsm->btn6_state_curr;
    }

    return fsm->mode_curr;
}

FSM_DATA_MODES FsmData_Do_MODE_EDIT_AGING(FsmData *fsm)
{
    Display_EditAging(&display, fsm->aging);

    if (fsm->btn1_state_prev != fsm->btn1_state_curr) {
        fsm->btn1_state_prev = fsm->btn1_state_curr;

        if (fsm->btn1_state_curr == BTN_UP) {
            if (fsm->update_aging) {
                Rtc_DS3231_set_aging(fsm->aging);
                fsm->update_aging = 0;
            }
        }

        fsm->mode_curr = fsm->mode_prev;
    } else {
        if (fsm->btn8_state_prev != fsm->btn8_state_curr) {
            if (fsm->btn8_state_curr == BTN_DOWN) {
                --fsm->aging;
                fsm->update_aging = 1;
            }

            fsm->btn8_state_prev = fsm->btn8_state_curr;
        } else if (fsm->btn7_state_prev != fsm->btn7_state_curr) {
            if (fsm->btn7_state_curr == BTN_DOWN) {
                ++fsm->aging;
                fsm->update_aging = 1;
            }

            fsm->btn7_state_prev = fsm->btn7_state_curr;
        } else if (fsm->btn2_state_prev != fsm->btn2_state_curr) {
            if (fsm->btn2_state_curr == BTN_DOWN) {
                fsm->mode_curr = FSM_DATA_MODE_EDIT_LATITUDE;
            }

            fsm->btn2_state_prev = fsm->btn2_state_curr;
        }
    }

    return fsm->mode_curr;
}

FSM_DATA_MODES FsmData_Do_MODE_EDIT_LATITUDE(FsmData *fsm)
{
    Display_EditLatitude(&display, fsm->latitude_deg);

    if (fsm->btn1_state_prev != fsm->btn1_state_curr) {
        fsm->btn1_state_prev = fsm->btn1_state_curr;

        if (fsm->btn1_state_curr == BTN_UP) {
            if (fsm->update_latitude) {
                /* save latitude to flash */
            	FlashWrite(fsm->tz_idx, fsm->latitude_deg, fsm->longitude_deg);

                fsm->update_latitude = 0;
                fsm->update_celestial = 1;
            }
        }

        fsm->mode_curr = fsm->mode_prev;
    } else {
        if (fsm->btn8_state_prev != fsm->btn8_state_curr || fsm->btn8_pressed_counter > BTN_REPEAT_DELAY) {
            if (fsm->btn8_state_curr == BTN_DOWN) {
                if (fsm->latitude_deg > -90.00)
                    fsm->latitude_deg -= 0.01;

                fsm->latitude_rad = fsm->latitude_deg * snm_DEG_TO_RAD;
                fsm->update_latitude = 1;
            } else {
                fsm->btn8_fast_repeat_delay_accelerated = BTN_FAST_REPEAT_DELAY;
            }

            fsm->btn8_pressed_counter = fsm->btn8_fast_repeat_delay_accelerated;
            fsm->btn8_fast_repeat_delay_accelerated += 10;
            fsm->btn8_state_prev = fsm->btn8_state_curr;
        } else if (fsm->btn7_state_prev != fsm->btn7_state_curr || fsm->btn7_pressed_counter > BTN_REPEAT_DELAY) {
            if (fsm->btn7_state_curr == BTN_DOWN) {
                if (fsm->latitude_deg < +90.00)
                    fsm->latitude_deg += 0.01;

                fsm->latitude_rad = fsm->latitude_deg * snm_DEG_TO_RAD;
                fsm->update_latitude = 1;
            } else {
                fsm->btn7_fast_repeat_delay_accelerated = BTN_FAST_REPEAT_DELAY;
            }

            fsm->btn7_pressed_counter = fsm->btn7_fast_repeat_delay_accelerated;
            fsm->btn7_fast_repeat_delay_accelerated += 10;
            fsm->btn7_state_prev = fsm->btn7_state_curr;
        } else if (fsm->btn2_state_prev != fsm->btn2_state_curr) {
            if (fsm->btn2_state_curr == BTN_DOWN) {
                fsm->mode_curr = FSM_DATA_MODE_EDIT_LONGITUDE;
            }

            fsm->btn2_state_prev = fsm->btn2_state_curr;
        }
    }

    return fsm->mode_curr;
}

FSM_DATA_MODES FsmData_Do_MODE_EDIT_LONGITUDE(FsmData *fsm)
{
    Display_EditLongitude(&display, fsm->longitude_deg);

    if (fsm->btn1_state_prev != fsm->btn1_state_curr) {
        fsm->btn1_state_prev = fsm->btn1_state_curr;

        if (fsm->btn1_state_curr == BTN_UP) {
            if (fsm->update_longitude) {
                /* save longitude to flash */
            	FlashWrite(fsm->tz_idx, fsm->latitude_deg, fsm->longitude_deg);

                fsm->update_longitude = 0;
                fsm->update_celestial = 1;
            }
        }

        fsm->mode_curr = fsm->mode_prev;
    } else {
        if (fsm->btn8_state_prev != fsm->btn8_state_curr || fsm->btn8_pressed_counter > BTN_REPEAT_DELAY) {
            if (fsm->btn8_state_curr == BTN_DOWN) {
                if (fsm->longitude_deg > -180.00)
                    fsm->longitude_deg -= 0.01;

                fsm->longitude_rad = fsm->longitude_deg * snm_DEG_TO_RAD;
                fsm->update_longitude = 1;
            } else {
                fsm->btn8_fast_repeat_delay_accelerated = BTN_FAST_REPEAT_DELAY;
            }

            fsm->btn8_pressed_counter = fsm->btn8_fast_repeat_delay_accelerated;
            fsm->btn8_fast_repeat_delay_accelerated += 10;
            fsm->btn8_state_prev = fsm->btn8_state_curr;
        } else if (fsm->btn7_state_prev != fsm->btn7_state_curr || fsm->btn7_pressed_counter > BTN_REPEAT_DELAY) {
            if (fsm->btn7_state_curr == BTN_DOWN) {
                if (fsm->longitude_deg < +180.00)
                    fsm->longitude_deg += 0.01;

                fsm->longitude_rad = fsm->longitude_deg * snm_DEG_TO_RAD;
                fsm->update_longitude = 1;
            } else {
                fsm->btn7_fast_repeat_delay_accelerated = BTN_FAST_REPEAT_DELAY;
            }

            fsm->btn7_pressed_counter = fsm->btn7_fast_repeat_delay_accelerated;
            fsm->btn7_fast_repeat_delay_accelerated += 10;
            fsm->btn7_state_prev = fsm->btn7_state_curr;
        } else if (fsm->btn2_state_prev != fsm->btn2_state_curr) {
            if (fsm->btn2_state_curr == BTN_DOWN) {
                fsm->mode_curr = FSM_DATA_MODE_EDIT_TIMEZONE;
            }

            fsm->btn2_state_prev = fsm->btn2_state_curr;
        }
    }

    return fsm->mode_curr;
}

FSM_DATA_MODES FsmData_Do_MODE_EDIT_TIMEZONE(FsmData *fsm)
{
    Display_EditTimezone(&display, fsm->tz);

    if (fsm->btn1_state_prev != fsm->btn1_state_curr) {
        fsm->btn1_state_prev = fsm->btn1_state_curr;

        if (fsm->btn1_state_curr == BTN_UP) {
            if (fsm->update_timezone) {
                /* save fsm->tz_idx to flash */
            	FlashWrite(fsm->tz_idx, fsm->latitude_deg, fsm->longitude_deg);

                fsm->update_timezone = 0;
                fsm->update_celestial = 1;
            }
        }

        fsm->mode_curr = fsm->mode_prev;
    } else {
        if (fsm->btn8_state_prev != fsm->btn8_state_curr) {
            if (fsm->btn8_state_curr == BTN_DOWN) {
                fsm->tz_idx = (fsm->tz_idx-1) % N_TIMEZONES;
                fsm->tz = TIMEZONES[fsm->tz_idx];
                fsm->update_timezone = 1;
            }

            fsm->btn8_state_prev = fsm->btn8_state_curr;
        } else if (fsm->btn7_state_prev != fsm->btn7_state_curr) {
            if (fsm->btn7_state_curr == BTN_DOWN) {
                fsm->tz_idx = (fsm->tz_idx+1) % N_TIMEZONES;
                fsm->tz = TIMEZONES[fsm->tz_idx];
                fsm->update_timezone = 1;
            }

            fsm->btn7_state_prev = fsm->btn7_state_curr;
        } else if (fsm->btn2_state_prev != fsm->btn2_state_curr) {
            if (fsm->btn2_state_curr == BTN_DOWN) {
                fsm->mode_curr = FSM_DATA_MODE_EDIT_AGING;
            }

            fsm->btn2_state_prev = fsm->btn2_state_curr;
        }
    }

    return fsm->mode_curr;
}

FSM_DATA_MODES FsmData_Do_MODE_EDIT_DATE(FsmData *fsm)
{
    Display_EditDate(&display, &fsm->ts);

    if (fsm->btn1_state_prev != fsm->btn1_state_curr) {
        fsm->btn1_state_prev = fsm->btn1_state_curr;

        if (fsm->btn1_state_curr == BTN_UP) {
            if (fsm->update_time) {
                Rtc_DS3231_set(fsm->ts);
                fsm->update_time = 0;
            }

            fsm->mode_curr = fsm->mode_prev;
        }
    } else if (fsm->btn8_state_prev != fsm->btn8_state_curr || fsm->btn8_pressed_counter > BTN_REPEAT_DELAY) {
        if (fsm->btn8_state_curr == BTN_DOWN) {
            if (fsm->ts.year < 2015)
                fsm->ts.year = 2015;

            fsm->ts.year -= 2000;
            fsm->ts.year += 1;
            fsm->ts.year %= 100;
            fsm->ts.year += 2000;
            fsm->update_time = 1;
        }

        fsm->btn8_pressed_counter = BTN_FAST_REPEAT_DELAY;
        fsm->btn8_state_prev = fsm->btn8_state_curr;
    } else if (fsm->btn7_state_prev != fsm->btn7_state_curr || fsm->btn7_pressed_counter > BTN_REPEAT_DELAY) {
        if (fsm->btn7_state_curr == BTN_DOWN) {
            fsm->ts.mon = (fsm->ts.mon % 12) + 1;
            fsm->update_time = 1;
        }

        fsm->btn7_pressed_counter = BTN_FAST_REPEAT_DELAY;
        fsm->btn7_state_prev = fsm->btn7_state_curr;
    } else if (fsm->btn6_state_prev != fsm->btn6_state_curr || fsm->btn6_pressed_counter > BTN_REPEAT_DELAY) {
        if (fsm->btn6_state_curr == BTN_DOWN) {
            fsm->ts.mday = (fsm->ts.mday % 31) + 1;
            fsm->update_time = 1;
        }

        fsm->btn6_pressed_counter = BTN_FAST_REPEAT_DELAY;
        fsm->btn6_state_prev = fsm->btn6_state_curr;
    }

    return fsm->mode_curr;
}

void FsmDataProcess(FsmData *fsm)
{
    FsmDataButtonsUpdateState(fsm);

    fsm->mode_curr = process_state(fsm->mode_curr, fsm);

    if (fsm->btn6_state_curr == BTN_DOWN)
            fsm->btn6_pressed_counter += CYCLE_DELAY;

    if (fsm->btn7_state_curr == BTN_DOWN)
        fsm->btn7_pressed_counter += CYCLE_DELAY;

    if (fsm->btn8_state_curr == BTN_DOWN)
        fsm->btn8_pressed_counter += CYCLE_DELAY;
}
