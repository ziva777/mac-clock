/*
 * fsm_data.c
 *
 *  Created on: 16 окт. 2018 г.
 *      Author: ziva
 */

#include <aux2.h>
#include "fsm_data.h"

#include <string.h>

#include "i2c.h"

#include "wire.h"
#include "flash.h"
#include "ws2812.h"
#include "display.h"
#include "display_aux.h"

static const uint32_t 	CYCLE_DELAY = 10u;
static const uint32_t 	BTN_REPEAT_DELAY = 1000u;
static const uint32_t 	BTN_FAST_REPEAT_DELAY = 850u;

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
	FsmData_Do_MODE_P1_2,
	FsmData_Do_MODE_P1_3,
    FsmData_Do_MODE_EDIT_TIME_1,
    FsmData_Do_MODE_EDIT_TIME_2,
    FsmData_Do_MODE_EDIT_AGING,
    FsmData_Do_MODE_EDIT_LATITUDE,
    FsmData_Do_MODE_EDIT_LONGITUDE,
    FsmData_Do_MODE_EDIT_TIMEZONE,
	FsmData_Do_MODE_EDIT_P_CORRECTION,
    FsmData_Do_MODE_EDIT_DATE,
};

FSM_DATA_MODES process_state(FSM_DATA_MODES curr_state, FsmData *state_data) {
    return STATES_TABLE[curr_state](state_data);
}

/* Private */

void FsmDataValidateTs(FsmData *fsm)
{
    if (fsm->ts.mday == 0 || fsm->ts.mon == 0 || fsm->ts.year < 2015) {
        /* Set to default */
        fsm->ts.mday = DT_MIN_DAY;
        fsm->ts.mon = DT_MIN_MON;
        fsm->ts.year = DT_MIN_YEAR;

        fsm->ts.hour = DT_MIN_HOUR;
        fsm->ts.min = DT_MIN_MIN;
        fsm->ts.sec = DT_MIN_SEC;
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
    fsm->update_p_correction = 0;
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

    {
    	/* Pressure calculation */
    	fsm->p_correction = FlashReadPCorrection();
    	fsm->p_base = SEALEVEL_PRESSURE;
		Bmp280Create(&fsm->bmp280, &hspi3);
    }

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
		SetBacklight(NIGHT_BG_COLOR_R,
					 NIGHT_BG_COLOR_G,
					 NIGHT_BG_COLOR_B); // ночь
    } else if ((fsm->ts.hour > fsm->sun_set2.hour)
            || (fsm->ts.hour == fsm->sun_set2.hour &&
                fsm->ts.min > fsm->sun_set2.min))
    {
    	SetBacklight(NIGHT_BG_COLOR_R,
    				 NIGHT_BG_COLOR_G,
					 NIGHT_BG_COLOR_B); // ночь
    } else {
        if ((fsm->ts.hour < fsm->sun_rise1.hour)
                || (fsm->ts.hour == fsm->sun_rise1.hour &&
                    fsm->ts.min < fsm->sun_rise1.min))
        {
            SetBacklight(TWILIGHT_BG_COLOR_R,
            			 TWILIGHT_BG_COLOR_G,
						 TWILIGHT_BG_COLOR_B); // сумерки
        } else if ((fsm->ts.hour > fsm->sun_set1.hour)
                || (fsm->ts.hour == fsm->sun_set1.hour &&
                    fsm->ts.min > fsm->sun_set1.min))
        {
        	SetBacklight(TWILIGHT_BG_COLOR_R,
						 TWILIGHT_BG_COLOR_G,
						 TWILIGHT_BG_COLOR_B); // сумерки
        } else {
			SetBacklight(DAYLIGHT_BG_COLOR_R,
						 DAYLIGHT_BG_COLOR_G,
						 DAYLIGHT_BG_COLOR_B); // день
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
    	fsm->btn4_state_prev = fsm->btn4_state_curr;

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
	{
		double t, p;
		Bmp280GetValues(&fsm->bmp280, &t, &p);
		p += fsm->p_correction;

        Display_P1(&display, p);

//		double a = Bmp280GetAltitude(&fsm->bmp280, fsm->p_base);
//		Display_P1(&display, a);
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
    } else if (fsm->btn4_state_prev != fsm->btn4_state_curr) {
    	fsm->btn4_state_prev = fsm->btn4_state_curr;

    	if (fsm->btn4_state_curr == BTN_DOWN) {
    		fsm->mode_curr = FSM_DATA_MODE_P1_2;
    	}
    }

    return fsm->mode_curr;
}

FSM_DATA_MODES FsmData_Do_MODE_P1_2(FsmData *fsm)
{
	{
		double t, p;
		Bmp280GetValues(&fsm->bmp280, &t, &p);
		p += fsm->p_correction;

        Display_P2(&display, t);
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
    } else if (fsm->btn4_state_prev != fsm->btn4_state_curr) {
    	fsm->btn4_state_prev = fsm->btn4_state_curr;

    	if (fsm->btn4_state_curr == BTN_DOWN) {
    		fsm->mode_curr = FSM_DATA_MODE_P1_3;
    	}
    }

    return fsm->mode_curr;
}

FSM_DATA_MODES FsmData_Do_MODE_P1_3(FsmData *fsm)
{
	{
		double a;

		a = Bmp280GetAltitude(&fsm->bmp280, fsm->p_base);
		Display_P3(&display, a);
    }

    if (fsm->btn1_state_prev != fsm->btn1_state_curr
        || fsm->btn2_state_prev != fsm->btn2_state_curr
        || fsm->btn3_state_prev != fsm->btn3_state_curr
        || fsm->btn5_state_prev != fsm->btn5_state_curr
        || fsm->btn6_state_prev != fsm->btn6_state_curr
        || fsm->btn7_state_prev != fsm->btn7_state_curr
        /*|| fsm->btn8_state_prev != fsm->btn8_state_curr*/)
    {
        fsm->mode_curr = fsm->mode_prev;
        fsm->btn2_state_prev = fsm->btn2_state_curr;
    } else if (fsm->btn4_state_prev != fsm->btn4_state_curr) {
    	fsm->btn4_state_prev = fsm->btn4_state_curr;

    	if (fsm->btn4_state_curr == BTN_DOWN) {
    		fsm->mode_curr = FSM_DATA_MODE_P1_1;
    	}
    } else if (fsm->btn8_state_prev != fsm->btn8_state_curr) {
    	fsm->btn8_state_prev = fsm->btn8_state_curr;

    	if (fsm->btn8_state_curr == BTN_DOWN) {
    		double t, p;
    		Bmp280GetValuesInSi(&fsm->bmp280, &t, &p);
    		p += fsm->p_correction;
    		p /= 100.0;
			fsm->p_base = p;
//			fsm->p_base = 1006;
			Beep();
		}
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
            	FlashWrite(fsm->tz_idx,
            			   fsm->latitude_deg,
						   fsm->longitude_deg,
						   fsm->p_correction);

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
            	FlashWrite(fsm->tz_idx,
						   fsm->latitude_deg,
						   fsm->longitude_deg,
						   fsm->p_correction);

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
            	FlashWrite(fsm->tz_idx,
						   fsm->latitude_deg,
						   fsm->longitude_deg,
						   fsm->p_correction);

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
            	fsm->mode_curr = FSM_DATA_MODE_EDIT_P_CORRECTION;
            }

            fsm->btn2_state_prev = fsm->btn2_state_curr;
        }
    }

    return fsm->mode_curr;
}

FSM_DATA_MODES FsmData_Do_MODE_EDIT_P_CORRECTION(FsmData *fsm)
{
    Display_EditPCorrection(&display, fsm->p_correction);

    if (fsm->btn1_state_prev != fsm->btn1_state_curr) {
        fsm->btn1_state_prev = fsm->btn1_state_curr;

        if (fsm->btn1_state_curr == BTN_UP) {
            if (fsm->update_p_correction) {
                /* save latitude to flash */
            	FlashWrite(fsm->tz_idx,
            			   fsm->latitude_deg,
						   fsm->longitude_deg,
						   fsm->p_correction);

                fsm->update_p_correction = 0;
                fsm->update_celestial = 1;
            }
        }

        fsm->mode_curr = fsm->mode_prev;
    } else {
        if (fsm->btn8_state_prev != fsm->btn8_state_curr || fsm->btn8_pressed_counter > BTN_REPEAT_DELAY) {
            if (fsm->btn8_state_curr == BTN_DOWN) {
                if (fsm->p_correction > -99.99)
                    fsm->p_correction -= 0.1;

                fsm->update_p_correction = 1;
            } else {
                fsm->btn8_fast_repeat_delay_accelerated = BTN_FAST_REPEAT_DELAY;
            }

            fsm->btn8_pressed_counter = fsm->btn8_fast_repeat_delay_accelerated;
            fsm->btn8_fast_repeat_delay_accelerated += 10;
            fsm->btn8_state_prev = fsm->btn8_state_curr;
        } else if (fsm->btn7_state_prev != fsm->btn7_state_curr || fsm->btn7_pressed_counter > BTN_REPEAT_DELAY) {
            if (fsm->btn7_state_curr == BTN_DOWN) {
                if (fsm->p_correction < +99.99)
                    fsm->p_correction += 0.1;

                fsm->update_p_correction = 1;
            } else {
                fsm->btn7_fast_repeat_delay_accelerated = BTN_FAST_REPEAT_DELAY;
            }

            fsm->btn7_pressed_counter = fsm->btn7_fast_repeat_delay_accelerated;
            fsm->btn7_fast_repeat_delay_accelerated += 10;
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
