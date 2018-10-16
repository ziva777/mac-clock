/*
 * fsm_data.h
 *
 *  Created on: 16 окт. 2018 г.
 *      Author: ziva
 */

#ifndef INC_FSM_DATA_H_
#define INC_FSM_DATA_H_

#include <stdint.h>

#include "kalman.h"
#include "rtc_ds3221.h"
#include "sun_n_moon.h"
#include "stm32f4xx_hal.h"

/*
 * Defines
 */
#define DEFAULT_LATITUDE    55.75
#define DEFAULT_LONGITUDE   37.50

#define BTN1_STATE()   (BTN_STATES)HAL_GPIO_ReadPin(BTN0_GPIO_Port, BTN0_Pin)
#define BTN2_STATE()   (BTN_STATES)HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin)
#define BTN3_STATE()   (BTN_STATES)HAL_GPIO_ReadPin(BTN2_GPIO_Port, BTN2_Pin)
#define BTN4_STATE()   (BTN_STATES)HAL_GPIO_ReadPin(BTN3_GPIO_Port, BTN3_Pin)
#define BTN5_STATE()   (BTN_STATES)HAL_GPIO_ReadPin(BTN4_GPIO_Port, BTN4_Pin)
#define BTN6_STATE()   (BTN_STATES)HAL_GPIO_ReadPin(BTN5_GPIO_Port, BTN5_Pin)
#define BTN7_STATE()   (BTN_STATES)HAL_GPIO_ReadPin(BTN6_GPIO_Port, BTN6_Pin)
#define BTN8_STATE()   (BTN_STATES)HAL_GPIO_ReadPin(BTN7_GPIO_Port, BTN7_Pin)

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

/*
 * FSM data states
 */
typedef enum {
    FSM_DATA_MODE_A1_1 = 0,
    FSM_DATA_MODE_A1_2,
    FSM_DATA_MODE_A1_3,

    FSM_DATA_MODE_S1_1,
    FSM_DATA_MODE_S1_2,

    FSM_DATA_MODE_P1_1,

    FSM_DATA_MODE_EDIT_TIME_1,
    FSM_DATA_MODE_EDIT_TIME_2,
    FSM_DATA_MODE_EDIT_AGING,
    FSM_DATA_MODE_EDIT_LATITUDE,
    FSM_DATA_MODE_EDIT_LONGITUDE,
    FSM_DATA_MODE_EDIT_TIMEZONE,
    FSM_DATA_MODE_EDIT_DATE,

    FSM_DATA_MODES_NUM
} FSM_DATA_MODES;

/*
 * Push button states
 */
typedef enum {
    BTN_UP = GPIO_PIN_SET,
    BTN_DOWN = GPIO_PIN_RESET
} BTN_STATES;


/*
 * FSM data
 */
typedef struct {
    /* Modes */
    FSM_DATA_MODES mode_curr, mode_prev;

    /* All about timing */
    Rtc_Timestamp ts;                   /* Current timestamp */
    Rtc_Timestamp sun_rise1, sun_set1;  /* Twilight */
    Rtc_Timestamp sun_rise2, sun_set2;  /* Actual */

    int8_t aging;                       /* Aging of osc */

    /* Observation point */
    uint8_t tz_idx;                     /* In TIMEZONES table */
    double tz;                          /* Actual value */

    double latitude_deg, latitude_rad;
    double longitude_deg, longitude_rad;

    /* Rise & set calculator */
    snm_Calculator calc;

    /* Atmospheric pressure filter */
    KalmanFilter pressure_filter;

    /* Update flags */
    int update_time;
    int update_aging;
    int update_latitude;
    int update_longitude;
    int update_timezone;
    int update_celestial;

    /* Hardware */
    BTN_STATES btn1_state_curr, btn1_state_prev;
    BTN_STATES btn2_state_curr, btn2_state_prev;
    BTN_STATES btn3_state_curr, btn3_state_prev;
    BTN_STATES btn4_state_curr, btn4_state_prev;
    BTN_STATES btn5_state_curr, btn5_state_prev;
    BTN_STATES btn6_state_curr, btn6_state_prev;
    BTN_STATES btn7_state_curr, btn7_state_prev;
    BTN_STATES btn8_state_curr, btn8_state_prev;

    /* Counters */
    uint32_t btn6_pressed_counter;
    uint32_t btn7_pressed_counter;
    uint32_t btn8_pressed_counter;

    uint32_t btn7_fast_repeat_delay_accelerated;
    uint32_t btn8_fast_repeat_delay_accelerated;

} FsmData;

/* Functions */
void FsmDataCreate(FsmData *fsm);
void FsmDataCalcCelestial(FsmData *fsm);
void FsmDataSetBacklight(FsmData *fsm);
void FsmDataButtonsUpdateState(FsmData *fsm);

FSM_DATA_MODES FsmData_Do_MODE_A1_1(FsmData *fsm);
FSM_DATA_MODES FsmData_Do_MODE_A1_2(FsmData *fsm);
FSM_DATA_MODES FsmData_Do_MODE_A1_3(FsmData *fsm);
FSM_DATA_MODES FsmData_Do_MODE_S1_1(FsmData *fsm);
FSM_DATA_MODES FsmData_Do_MODE_S1_2(FsmData *fsm);
FSM_DATA_MODES FsmData_Do_MODE_P1_1(FsmData *fsm);
FSM_DATA_MODES FsmData_Do_MODE_EDIT_TIME_1(FsmData *fsm);
FSM_DATA_MODES FsmData_Do_MODE_EDIT_TIME_2(FsmData *fsm);
FSM_DATA_MODES FsmData_Do_MODE_EDIT_AGING(FsmData *fsm);
FSM_DATA_MODES FsmData_Do_MODE_EDIT_LATITUDE(FsmData *fsm);
FSM_DATA_MODES FsmData_Do_MODE_EDIT_LONGITUDE(FsmData *fsm);
FSM_DATA_MODES FsmData_Do_MODE_EDIT_TIMEZONE(FsmData *fsm);
FSM_DATA_MODES FsmData_Do_MODE_EDIT_DATE(FsmData *fsm);

void FsmDataProcess(FsmData *fsm);

#endif /* INC_FSM_DATA_H_ */
