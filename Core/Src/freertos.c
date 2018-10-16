/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include <stdlib.h>

#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

#include <string.h>
#include <stdlib.h>

#include "mb.h"
#include "mbport.h"
#include "mbutils.h"

#include "wire.h"
#include "ws2812.h"
#include "kalman.h"
#include "display.h"
#include "rtc_ds3221.h"
#include "luminosity_sensor.h"

#include "sun_n_moon.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef int32_t BMP280_S32_t;
typedef uint32_t BMP280_U32_t;

typedef enum {
    BTN_UP = GPIO_PIN_SET,
    BTN_DOWN = GPIO_PIN_RESET
} BTN_STATES;

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
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
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
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static const uint32_t 	CYCLE_DELAY = 10u;
static const uint32_t 	BTN_REPEAT_DELAY = 1000u;
static const uint32_t 	BTN_FAST_REPEAT_DELAY = 850u;
static const uint8_t 	BR_MIN = 1u;        /* Abs brightness minimum */
static const uint8_t 	BR_MAX = 254u;      /* Abs brightness maximum */
static const uint16_t 	LUM_MIN = 0u;      /* Abs luminosity minimum */
static const uint16_t 	LUM_MAX = 4095u;   /* Abs luminosity maximum */

//static const uint32_t tz_idx_addr = 0x080FA000;
//#define TZ_IDX_ADDR 	(0x080FFC00/*+0x80+0x40+0x20+0x0*/)
//#define TZ_IDX_ADDR 	(0x080FFF80/*+0x80+0x40+0x20+0x0*/)
//#define TZ_IDX_ADDR 	(0x080FFFF0-16)
//#define TZ_IDX_ADDR 	(0x080E0000)
#define TZ_IDX_ADDR 	(0x080FFF70)
#define LATITUDE_ADDR 	(0x080FFF80)
#define LONGITUDE_ADDR 	(0x080FFF90)


Display display;
size_t n_places = 6;

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

osThreadId modbus_task_handle;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static
void FlashErase()
{
	HAL_StatusTypeDef flash_ok;

	flash_ok = HAL_ERROR;
	while(flash_ok != HAL_OK) {
		flash_ok = HAL_FLASH_Unlock();
	}

	FLASH_Erase_Sector(FLASH_SECTOR_11, VOLTAGE_RANGE_3);
//	HAL_FLASH_Lock();

	flash_ok = HAL_ERROR;
	while(flash_ok != HAL_OK) {
		flash_ok = HAL_FLASH_Lock();
	}
}

static
void FlashWrite(int tz_idx, double latitude, double longitude)
{
	HAL_StatusTypeDef flash_ok;

	union {
		uint32_t u32[2];
		uint64_t u64;
		double d;
	} cast;

	FlashErase();

	{
		flash_ok = HAL_ERROR;
		while (flash_ok != HAL_OK) {
			flash_ok = HAL_FLASH_Unlock();
		}

		flash_ok = HAL_ERROR;
		while (flash_ok != HAL_OK) {
			flash_ok = HAL_FLASH_Program(TYPEPROGRAM_WORD, TZ_IDX_ADDR, tz_idx);
		}

		/* lat */
		cast.d = latitude;
		flash_ok = HAL_ERROR;
		while (flash_ok != HAL_OK) {
			flash_ok = HAL_FLASH_Program(TYPEPROGRAM_WORD, LATITUDE_ADDR,
					cast.u32[0]);
		}

		flash_ok = HAL_ERROR;
		while (flash_ok != HAL_OK) {
			flash_ok = HAL_FLASH_Program(TYPEPROGRAM_WORD,
					LATITUDE_ADDR + sizeof(int), cast.u32[1]);
		}
		/* ~ lat */

		/* long */
		cast.d = longitude;
		flash_ok = HAL_ERROR;
		while (flash_ok != HAL_OK) {
			flash_ok = HAL_FLASH_Program(TYPEPROGRAM_WORD, LONGITUDE_ADDR,
					cast.u32[0]);
		}

		flash_ok = HAL_ERROR;
		while (flash_ok != HAL_OK) {
			flash_ok = HAL_FLASH_Program(TYPEPROGRAM_WORD,
					LONGITUDE_ADDR + sizeof(int), cast.u32[1]);
		}
		/* ~ long */

		flash_ok = HAL_ERROR;
		while (flash_ok != HAL_OK) {
			flash_ok = HAL_FLASH_Lock();
		}
	}
}

static
int FlashReadTZ()
{
	int32_t idx = *(volatile int *)(TZ_IDX_ADDR);
	return idx;
}

static
double FlashReadLatitude()
{
	double v = *(volatile double *)(LATITUDE_ADDR);
	return v;
}

static
double FlashReadLongitude()
{
	double v = *(volatile double *)(LONGITUDE_ADDR);
	return v;
}

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

/* Установка яркости */
static
void SetBrightness(uint16_t br)
{
    htim5.Instance->CCR4 = br;
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
}

/* Включить PWM для индикатора */
static
void SetBrightnessOn()
{
    HAL_TIM_Base_Start(&htim5);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
}

/* Установка цвета фоновой подстветки */
static
void SetBacklight(uint8_t r, uint8_t g, uint8_t b)
{
    static uint8_t R, G, B;

    if (r != R || g != G || b != B) {
        R = r;
        G = g;
        B = b;

        ws2812_pixel_rgb_to_buf_dma(r, g, b, 0);
        ws2812_pixel_rgb_to_buf_dma(r, g, b, 1);
        HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, BUF_DMA, ARRAY_LEN);
    }
}

/* Бип */
void Beep()
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_Delay(10);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
}


static
float calc_brightness_linear(float l,
                             float b_inf,
                             float b_sup)
{
    float k, b, y;

    k = (b_inf - b_sup);
    k /=  (float)(LUM_MAX - LUM_MIN);
    b = b_sup;
    y = k * l + b;

    return y;
}

/* Отобразить восход солнца чч.мм */
void Display_S1_1(Display *display,
                  Rtc_Timestamp *ts)
{
    const size_t n_places = 6;
    Character c[n_places];
    Dot d[n_places];

    uint8_t h1, h2;
    uint8_t m1, m2;

    h1 = ts->hour / 10;
    h2 = ts->hour % 10;
    h1 += 0x30;
    h2 += 0x30;

    c[4 - 1] = AsciiToCharacter((char) h2);

    if (h1 != 0x30)
        c[5 - 1] = AsciiToCharacter((char) h1);
    else
        c[5 - 1] = CH_BLANK;

    m1 = ts->min / 10;
    m2 = ts->min % 10;
    m1 += 0x30;
    m2 += 0x30;
    c[2 - 1] = AsciiToCharacter((char) m2);
    c[3 - 1] = AsciiToCharacter((char) m1);

    c[0] = AsciiToCharacter((char) ' ');
    c[5] = CH_BLANK;

    d[0] = DOT_OBSCURE;
    d[1] = DOT_OBSCURE;
    d[2] = DOT_OBSCURE;
    d[3] = DOT_HIGHLIGHT;
    d[4] = DOT_OBSCURE;
    d[5] = DOT_OBSCURE;

    DisplayWrite(display, c, d, n_places);
    DisplaySync(display);
}

/* Отобразить сообщение */
void Display_Msg(Display *display,
                 Character msg[],
                 size_t n_places)
{
//  const size_t n_places = 6;
    Dot d[n_places];

    d[0] = DOT_OBSCURE;
    d[1] = DOT_OBSCURE;
    d[2] = DOT_OBSCURE;
    d[3] = DOT_OBSCURE;
    d[4] = DOT_OBSCURE;
    d[5] = DOT_OBSCURE;

    DisplayWrite(display, msg, d, n_places);
    DisplaySync(display);
}

/* Отобразить восход солнца (сообщение) */
void Display_S1_1_Msg(Display *display)
{
    const size_t n_places = 6;
    Character c[n_places];

    c[5] = CH_B;
    c[4] = CH_O;
    c[3] = CH_C;
    c[2] = CH_X;
    c[1] = CH_O;
    c[0] = CH_D_rus;

    Display_Msg(display, c, n_places);
}

/* Отобразить заход солнца (сообщение) */
void Display_S2_1_Msg(Display *display)
{
    const size_t n_places = 6;
    Character c[n_places];

    c[5] = CH_3;
    c[4] = CH_A;
    c[3] = CH_X;
    c[2] = CH_O;
    c[1] = CH_D_rus;
    c[0] = CH_BLANK;

    Display_Msg(display, c, n_places);
}

/* Отобразить давление */
void Display_P1(Display *display,
                uint16_t pressure)
{
    const size_t n_places = 6;
    Character c[n_places];
    Dot d[n_places];

    d[0] = DOT_OBSCURE;
    d[1] = DOT_HIGHLIGHT;
    d[2] = DOT_OBSCURE;
    d[3] = DOT_OBSCURE;
    d[4] = DOT_OBSCURE;
    d[5] = DOT_OBSCURE;

    c[5] = CH_P;
    c[4] = CH_r;
    c[3] = CH_BLANK;

    Character tmp;

    tmp = AsciiToCharacter((int)(pressure / 1000) % 10 + 0x30);
    c[3] = (tmp != CH_0 ? tmp : CH_BLANK);

    tmp = AsciiToCharacter((int)(pressure / 100) % 10 + 0x30);
    c[2] = (tmp != CH_0 ? tmp : CH_BLANK);

    c[1] = AsciiToCharacter((int)(pressure / 10) % 10 + 0x30);
    c[0] = AsciiToCharacter((int)(pressure / 1) % 10 + 0x30);

    DisplayWrite(display, c, d, n_places);
    DisplaySync(display);
}

/* Отобразить чч.мм.сс */
void Display_A1_1(Display *display,
                  Rtc_Timestamp *ts)
{
    const size_t n_places = 6;
    Character c[n_places];
    Dot d[n_places];

    uint8_t h1, h2;
    uint8_t m1, m2;
    uint8_t s1, s2;

    h1 = ts->hour / 10;
    h2 = ts->hour % 10;
    h1 += 0x30;
    h2 += 0x30;
    c[4] = AsciiToCharacter((char) h2);

    if (h1 != 0x30)
        c[5] = AsciiToCharacter((char) h1);
    else
        c[5] = CH_BLANK;

    m1 = ts->min / 10;
    m2 = ts->min % 10;
    m1 += 0x30;
    m2 += 0x30;
    c[2] = AsciiToCharacter((char) m2);
    c[3] = AsciiToCharacter((char) m1);

    s1 = ts->sec / 10;
    s2 = ts->sec % 10;
    s1 += 0x30;
    s2 += 0x30;
    c[0] = AsciiToCharacter((char) s2);
    c[1] = AsciiToCharacter((char) s1);

    d[0] = DOT_OBSCURE;
    d[1] = DOT_OBSCURE;
    d[2] = DOT_HIGHLIGHT;
    d[3] = DOT_OBSCURE;
    d[4] = DOT_HIGHLIGHT;
    d[5] = DOT_OBSCURE;

    DisplayWrite(display, c, d, n_places);
    DisplaySync(display);
}

/* Отобразить чч.мм с мигающим индикатором */
void Display_A1_2(Display *display,
                  Rtc_Timestamp *ts)
{
    const size_t n_places = 6;
    Character c[n_places];
    Dot d[n_places];

    uint8_t h1, h2;
    uint8_t m1, m2;

    h1 = ts->hour / 10;
    h2 = ts->hour % 10;
    h1 += 0x30;
    h2 += 0x30;

    c[4 - 1] = AsciiToCharacter((char) h2);

    if (h1 != 0x30)
        c[5 - 1] = AsciiToCharacter((char) h1);
    else
        c[5 - 1] = CH_BLANK;

    m1 = ts->min / 10;
    m2 = ts->min % 10;
    m1 += 0x30;
    m2 += 0x30;
    c[2 - 1] = AsciiToCharacter((char) m2);
    c[3 - 1] = AsciiToCharacter((char) m1);

    c[0] = AsciiToCharacter((char) ' ');
    c[5] = AsciiToCharacter((char) ' ');

    d[0] = DOT_OBSCURE;
    d[1] = DOT_OBSCURE;
    d[2] = DOT_OBSCURE;
    d[3] = (ts->sec % 2 == 0 ? DOT_HIGHLIGHT : DOT_OBSCURE);
    d[4] = DOT_OBSCURE;
    d[5] = DOT_OBSCURE;

    DisplayWrite(display, c, d, n_places);
    DisplaySync(display);
}

/* Отобразить дд.ММ.гг */
void Display_A1_3(Display *display,
                  Rtc_Timestamp *ts)
{
    const size_t n_places = 6;
    Character c[n_places];
    Dot d[n_places];

    uint8_t h1, h2;
    uint8_t m1, m2;
    uint8_t s1, s2;

    h1 = ts->mday / 10;
    h2 = ts->mday % 10;
    h1 += 0x30;
    h2 += 0x30;
    c[4] = AsciiToCharacter((char) h2);

    if (h1 != 0x30)
        c[5] = AsciiToCharacter((char) h1);
    else
        c[5] = CH_BLANK;

    m1 = ts->mon / 10;
    m2 = ts->mon % 10;
    m1 += 0x30;
    m2 += 0x30;
    c[2] = AsciiToCharacter((char) m2);
    c[3] = AsciiToCharacter((char) m1);

    s1 = (ts->year % 100) / 10;
    s2 = (ts->year % 100) % 10;
    s1 += 0x30;
    s2 += 0x30;
    c[0] = AsciiToCharacter((char) s2);
    c[1] = AsciiToCharacter((char) s1);

    d[0] = DOT_OBSCURE;
    d[1] = DOT_OBSCURE;
    d[2] = DOT_HIGHLIGHT;
    d[3] = DOT_OBSCURE;
    d[4] = DOT_HIGHLIGHT;
    d[5] = DOT_OBSCURE;

    DisplayWrite(display, c, d, n_places);
    DisplaySync(display);
}

/* Редактировать время в формате чч.мм.сс */
void Display_EditTime1(Display *display,
                       Rtc_Timestamp *ts)
{
    Display_A1_1(display, ts);
}

/* Редактировать время в формате чч.мм */
void Display_EditTime2(Display *display,
                       Rtc_Timestamp *ts)
{
    Display_A1_2(display, ts);
}

/* Отобразить параметр точной настройки */
void Display_EditAging(Display *display,
                       int8_t aging)
{
    const size_t n_places = 6;
    Character c[n_places];
    Dot d[n_places];

    d[0] = DOT_OBSCURE;
    d[1] = DOT_OBSCURE;
    d[2] = DOT_OBSCURE;
    d[3] = DOT_OBSCURE;
    d[4] = DOT_HIGHLIGHT;
    d[5] = DOT_OBSCURE;

    c[5] = CH_A;
    c[4] = CH_G;
    c[3] = CH_BLANK;

    if (aging < 0) {
        c[3] = CH_MINUS;
        aging = abs(aging);
    }

    c[2] = AsciiToCharacter((aging / 100) % 10 + 0x30);
    c[1] = AsciiToCharacter((aging / 10) % 10 + 0x30);
    c[0] = AsciiToCharacter((aging / 1) % 10 + 0x30);

    DisplayWrite(display, c, d, n_places);
    DisplaySync(display);
}

/* Редактирование широты (-90.00 ... +90.00) */
void Display_EditLatitude(Display *display,
                          double latitude)
{
    const size_t n_places = 6;
    Character c[n_places];
    Dot d[n_places];

    d[0] = DOT_OBSCURE;
    d[1] = DOT_OBSCURE;
    d[2] = DOT_HIGHLIGHT;
    d[3] = DOT_OBSCURE;
    d[4] = DOT_OBSCURE;
    d[5] = DOT_OBSCURE;

    c[5] = (latitude > 0.0 ? CH_N : CH_S);
    c[4] = CH_BLANK;
    c[3] = CH_BLANK;

    latitude = fabs(latitude);
    int hi = latitude;
    int low = ((latitude - hi) * 100.0);
    int r = hi * 100 + low;


    c[3] = AsciiToCharacter((r / 1000) % 10 + 0x30);
    c[2] = AsciiToCharacter((r / 100) % 10 + 0x30);
    c[1] = AsciiToCharacter((r / 10) % 10 + 0x30);
    c[0] = AsciiToCharacter((r / 1) % 10 + 0x30);

    DisplayWrite(display, c, d, n_places);
    DisplaySync(display);
}

/* Редактирование долготы (-180.00 ... +180.00) */
void Display_EditLongitude(Display *display,
                           double longitude)
{
    const size_t n_places = 6;
    Character c[n_places];
    Dot d[n_places];

    d[0] = DOT_OBSCURE;
    d[1] = DOT_OBSCURE;
    d[2] = DOT_HIGHLIGHT;
    d[3] = DOT_OBSCURE;
    d[4] = DOT_OBSCURE;
    d[5] = DOT_OBSCURE;

    c[5] = (longitude >= 0.0 ? CH_E : CH_W);
    c[4] = CH_BLANK;
    c[3] = CH_BLANK;

    longitude = fabs(longitude);
    int hi = longitude;
    int low = ((longitude - hi) * 100.0);
    int r = hi * 100 + low;


    c[3] = AsciiToCharacter((r / 1000) % 10 + 0x30);
    c[2] = AsciiToCharacter((r / 100) % 10 + 0x30);
    c[1] = AsciiToCharacter((r / 10) % 10 + 0x30);
    c[0] = AsciiToCharacter((r / 1) % 10 + 0x30);

    DisplayWrite(display, c, d, n_places);
    DisplaySync(display);
}

/* Редактирование временной зоны */
void Display_EditTimezone(Display *display,
                          double tz)
{
    const size_t n_places = 6;
    Character c[n_places];
    Dot d[n_places];

    d[0] = DOT_OBSCURE;
    d[1] = DOT_OBSCURE;
    d[2] = DOT_HIGHLIGHT;
    d[3] = DOT_OBSCURE;
    d[4] = DOT_OBSCURE;
    d[5] = DOT_OBSCURE;

    c[5] = CH_Z;
    c[4] = (tz > 0.0 ? CH_PLUS : CH_MINUS);
    c[3] = CH_BLANK;

    tz = fabs(tz);
    int hi = tz;
    int low = ((tz - hi) * 100.0);
    int r = hi * 100 + low;

    c[3] = AsciiToCharacter((r / 1000) % 10 + 0x30);
    c[2] = AsciiToCharacter((r / 100) % 10 + 0x30);
    c[1] = AsciiToCharacter((r / 10) % 10 + 0x30);
    c[0] = AsciiToCharacter((r / 1) % 10 + 0x30);

    DisplayWrite(display, c, d, n_places);
    DisplaySync(display);
}

/* Редактировать дату */
void Display_EditDate(Display *display,
                      Rtc_Timestamp *ts)
{
    Display_A1_3(display, ts);
}

//uint16_t to_U_LE(uint16_t value) {
//    return (value >> 8) | (value << 8);
//}


void ModbusTask(void const * argument);

/*
 * FSM luminosity
 */
typedef struct {
	KalmanFilter luminosity_filter;
    float lum;
    float br;
} FsmLuminosity;

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

/* private */
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

/* public */
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

//    fsm->btn1_state_curr = BTN1_STATE();

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
    uint16_t pressure = 1;

    {
        const uint32_t delay = 5000u;
        uint8_t data[6];
        uint8_t address[1];

        CS_SET();
        data[0] = 0xF4 & ~0x80;
        data[1] = 0b00100111;
        HAL_SPI_Transmit(&hspi3, data, 2, delay);
        CS_RESET();

        // cooeff's
        // T1
        CS_SET();
        address[0] = BMP280_REGISTER_DIG_T1 | 0x80;
        HAL_SPI_Transmit(&hspi3, address, 1, delay);
        HAL_SPI_Receive(&hspi3, (uint8_t *)&dig_T1, 2, delay);
        CS_RESET();

        // T2
        CS_SET();
        address[0] = BMP280_REGISTER_DIG_T2 | 0x80;
        HAL_SPI_Transmit(&hspi3, address, 1, delay);
        HAL_SPI_Receive(&hspi3, (uint8_t *)&dig_T2, 2, delay);
        CS_RESET();

        // T3
        CS_SET();
        address[0] = BMP280_REGISTER_DIG_T3 | 0x80;
        HAL_SPI_Transmit(&hspi3, address, 1, delay);
        HAL_SPI_Receive(&hspi3, (uint8_t *)&dig_T3, 2, delay);
        CS_RESET();

        // P1
        CS_SET();
        address[0] = BMP280_REGISTER_DIG_P1 | 0x80;
        HAL_SPI_Transmit(&hspi3, address, 1, delay);
        HAL_SPI_Receive(&hspi3, (uint8_t *)&dig_P1, 2, delay);
        CS_RESET();

        // P2
        CS_SET();
        address[0] = BMP280_REGISTER_DIG_P2 | 0x80;
        HAL_SPI_Transmit(&hspi3, address, 1, delay);
        HAL_SPI_Receive(&hspi3, (uint8_t *)&dig_P2, 2, delay);
        CS_RESET();

        // P3
        CS_SET();
        address[0] = BMP280_REGISTER_DIG_P3 | 0x80;
        HAL_SPI_Transmit(&hspi3, address, 1, delay);
        HAL_SPI_Receive(&hspi3, (uint8_t *)&dig_P3, 2, delay);
        CS_RESET();

        // P4
        CS_SET();
        address[0] = BMP280_REGISTER_DIG_P4 | 0x80;
        HAL_SPI_Transmit(&hspi3, address, 1, delay);
        HAL_SPI_Receive(&hspi3, (uint8_t *)&dig_P4, 2, delay);
        CS_RESET();

        // P5
        CS_SET();
        address[0] = BMP280_REGISTER_DIG_P5 | 0x80;
        HAL_SPI_Transmit(&hspi3, address, 1, delay);
        HAL_SPI_Receive(&hspi3, (uint8_t *)&dig_P5, 2, delay);
        CS_SET();
        CS_RESET();

        // P6
        CS_SET();
        address[0] = BMP280_REGISTER_DIG_P6 | 0x80;
        HAL_SPI_Transmit(&hspi3, address, 1, delay);
        HAL_SPI_Receive(&hspi3, (uint8_t *)&dig_P6, 2, delay);
        CS_RESET();

        // P7
        CS_SET();
        address[0] = BMP280_REGISTER_DIG_P7 | 0x80;
        HAL_SPI_Transmit(&hspi3, address, 1, delay);
        HAL_SPI_Receive(&hspi3, (uint8_t *)&dig_P7, 2, delay);
        CS_RESET();

        // P8
        CS_SET();
        address[0] = BMP280_REGISTER_DIG_P8 | 0x80;
        HAL_SPI_Transmit(&hspi3, address, 1, delay);
        HAL_SPI_Receive(&hspi3, (uint8_t *)&dig_P8, 2, delay);
        CS_RESET();

        // P9
        CS_SET();
        address[0] = BMP280_REGISTER_DIG_P9 | 0x80;
        HAL_SPI_Transmit(&hspi3, address, 1, delay);
        HAL_SPI_Receive(&hspi3, (uint8_t *)&dig_P9, 2, delay);
        CS_RESET();


        // adc values
        CS_SET();
        address[0] = 0xF7 | 0x80;
        HAL_SPI_Transmit(&hspi3, address, 1, delay);
        HAL_SPI_Receive(&hspi3, data, 6, delay);
        CS_RESET();

        uint8_t adc_P_msb = data[0];
        uint8_t adc_P_lsb = data[1];
        uint8_t adc_P_xlsb = data[2];

        uint8_t adc_T_msb = data[3];
        uint8_t adc_T_lsb = data[4];
        uint8_t adc_T_xlsb = data[5];

        BMP280_S32_t adc_T = ((adc_T_msb << 16) | (adc_T_lsb << 8) | adc_T_xlsb) >> 4;
        BMP280_S32_t adc_P = ((adc_P_msb << 16) | (adc_P_lsb << 8) | adc_P_xlsb) >> 4;

        BMP280_S32_t p, t;

        t = bmp280_compensate_T_int32(adc_T);
        p = bmp280_compensate_P_int32(adc_P);

        double tmp = p;
        tmp *= 0.00750062;
        tmp *= 10.0;

        pressure = tmp;
//                volatile uint8_t a = *(volatile uint32_t *)TZ_IDX_ADDR;
//                pressure = a;

        HAL_Delay(500u);
    }

    Display_P1(&display, pressure);

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
                /* TODO: save latitude to flash */
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
                /* TODO: save longitude to flash */
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
                /* TODO: save fsm->tz_idx to flash */
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

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 1024);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadDef(modbus_task, ModbusTask, osPriorityNormal, 0, 128);
  modbus_task_handle = osThreadCreate(osThread(modbus_task), NULL);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
    {
        /* Hardware setup */
        HAL_Delay(10);
        ws2812_init();
        SetBrightnessOn();
        SetBrightness(200u);
        Beep();
    }

//    FlashWrite(17, 33, 44);

		{
	    	volatile int tz_idx2 = FlashReadTZ();
	    	volatile double latitude_deg2 = FlashReadLatitude();
	    	volatile double longitude_deg2 = FlashReadLongitude();

	    	for (int i = 0; i != tz_idx2; ++i) {
	    		latitude_deg2 += 1.0;
	    		longitude_deg2 += 2.0;
//	    		Beep();
	    	}
		}
//    	int tz_idx = 15;
//    	double latitude_deg = 1.0;
//    	double longitude_deg = 2.0;

//    	FLASH_Erase_Sector(FLASH_SECTOR_11, VOLTAGE_RANGE_3);
//    	FlashWrite(16, 1.0, 2.0);
//
//    	volatile int tz_idx2 = FlashReadTZ();
//    	volatile double latitude_deg2 = FlashReadLatitude();
//    	volatile double longitude_deg2 = FlashReadLongitude();
//
//    	for (int i = 0; i != tz_idx2; ++i) {
//    		latitude_deg2 += 1.0;
//    		longitude_deg2 += 2.0;
//    		Beep();
//    	}

//    	HAL_StatusTypeDef flash_ok = HAL_ERROR;
//
//    	flash_ok = HAL_ERROR;
//		while(flash_ok != HAL_OK){
//			flash_ok = HAL_FLASH_Unlock();
//		}
//
//    	FLASH_Erase_Sector(FLASH_SECTOR_11, VOLTAGE_RANGE_3);
//
//    	flash_ok = HAL_ERROR;
//		while(flash_ok != HAL_OK){
//			flash_ok = HAL_FLASH_Lock();
//		}
//
//    	FlashWriteTZ(3);
//    	volatile int tag = FlashReadTZ();
//
//    	for (int i = 0; i != tag; ++i) {
//    		Beep();
//    	}
//    }

    FsmData fsm;
    FsmLuminosity fsm_lum;
    
    FsmDataCreate(&fsm);
    FsmLuminosityCreate(&fsm_lum);

    DisplayCreate(&display,
                  n_places,
                  &hspi1,
                  LED_DATA_LATCH_GPIO_Port,
                  LED_DATA_LATCH_Pin);

  /* Infinite loop */
    while (1) {
        FsmLuminosityDoCalc(&fsm_lum);

        FsmDataCalcCelestial(&fsm);
        FsmDataSetBacklight(&fsm);
        
        FsmDataProcess(&fsm);

        HAL_Delay(CYCLE_DELAY);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void ModbusTask(void const * argument)
{
    eMBErrorCode eStatus;

    eStatus = eMBInit(MB_RTU, 1, 2, 115200, MB_PAR_NONE);
    eStatus = eMBEnable();

    if (eStatus != MB_ENOERR) {
        return;
    }

    while (1) {
        eMBPoll();
        taskYIELD();
    }
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
