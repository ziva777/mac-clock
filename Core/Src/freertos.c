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
typedef enum {
    BTN_UP = GPIO_PIN_SET,
    BTN_DOWN = GPIO_PIN_RESET
} BTN_STATES;

typedef enum {
	DISPLAY_MODE_A1_1,
	DISPLAY_MODE_A1_2,
	DISPLAY_MODE_A1_3,
	DISPLAY_MODE_A1_4,
	DISPLAY_MODE_A1_5,

	DISPLAY_MODE_S1_1,
	DISPLAY_MODE_S1_2,

	DISPLAY_MODE_EDIT_TIME_1,
	DISPLAY_MODE_EDIT_TIME_2,
	DISPLAY_MODE_EDIT_AGING,
	DISPLAY_MODE_EDIT_LATITUDE,
	DISPLAY_MODE_EDIT_LONGITUDE,
	DISPLAY_MODE_EDIT_TIMEZONE,
	DISPLAY_MODE_EDIT_DATE
} DISPLAY_MODES;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DEFAULT_LATITUDE  	55.75
#define DEFAULT_LONGITUDE  	37.50

#define BTN1_STATE()   (BTN_STATES)HAL_GPIO_ReadPin(BTN0_GPIO_Port, BTN0_Pin)
#define BTN2_STATE()   (BTN_STATES)HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin)
#define BTN3_STATE()   (BTN_STATES)HAL_GPIO_ReadPin(BTN2_GPIO_Port, BTN2_Pin)
#define BTN4_STATE()   (BTN_STATES)HAL_GPIO_ReadPin(BTN3_GPIO_Port, BTN3_Pin)
#define BTN5_STATE()   (BTN_STATES)HAL_GPIO_ReadPin(BTN4_GPIO_Port, BTN4_Pin)
#define BTN6_STATE()   (BTN_STATES)HAL_GPIO_ReadPin(BTN5_GPIO_Port, BTN5_Pin)
#define BTN7_STATE()   (BTN_STATES)HAL_GPIO_ReadPin(BTN6_GPIO_Port, BTN6_Pin)
#define BTN8_STATE()   (BTN_STATES)HAL_GPIO_ReadPin(BTN7_GPIO_Port, BTN7_Pin)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static KalmanFilter luminosity_filter;

static double TIMEZONES[] = {
    -12.00,
    -11.00,
    -10.00,
    -09.3001,
    -09.00,
    -08.00,
    -07.00,
    -06.00,
    -05.00,
    -04.00,
    -03.3001,
    -03.00,
    -02.00,
    -01.00,
    +00.00,
    +01.00,
    +02.00,
    +03.00,
    +03.3001,
    +04.00,
    +04.3001,
    +05.00,
    +05.3001,
    +05.4501,
    +06.00,
    +06.3001,
    +07.00,
    +08.00,
    +08.4501,
    +09.00,
    +09.3001,
    +10.00,
    +10.3001,
    +11.00,
    +12.00,
    +12.4501,
    +13.00,
    +14.00,
};
static const uint8_t N_TIMEZONES = sizeof(TIMEZONES) / sizeof(TIMEZONES[0]);
static const uint8_t MSK_ZONE = 17;

osThreadId modbus_task_handle;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* Установка яркости */
static
void SetBrightness(uint16_t br)
{
    htim5.Instance->CCR4 = br;
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
}

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

static const uint8_t BR_MIN = 1u;        /* Abs brightness minimum */
static const uint8_t BR_MAX = 254u;      /* Abs brightness maximum */
static const uint16_t LUM_MIN = 0u;      /* Abs luminosity minimum */
static const uint16_t LUM_MAX = 4095u;   /* Abs luminosity maximum */

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
				  Timestamp *ts)
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
//	const size_t n_places = 6;
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

/* Отобразить чч.мм.сс */
void Display_A1_1(Display *display,
				  Timestamp *ts)
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
				  Timestamp *ts)
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
				  Timestamp *ts)
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

/* Отобразить чередование чч.мм.сс и дд.ММ.гг */
void Display_A1_4(Display *display,
				  Timestamp *ts)
{
	static int flip_flop = 0;
	static int counter = 0;
	static const int counter_max = 5;
	static int sec_prev = 0;

	if (sec_prev != ts->sec) {
		sec_prev = ts->sec;

		if (++counter == counter_max) {
			counter = 0;
			++flip_flop;
		}

		if (flip_flop % 2) {
			Display_A1_1(display, ts);
		} else {
			Display_A1_3(display, ts);
		}
	}
}

/* Отобразить чередование чч.мм и дд.ММ.гг */
void Display_A1_5(Display *display,
				  Timestamp *ts)
{
	static int flip_flop = 0;
	static int counter = 0;
	static const int counter_max = 5;
	static int sec_prev = 0;

	if (sec_prev != ts->sec) {
		sec_prev = ts->sec;

		if (++counter == counter_max) {
			counter = 0;
			++flip_flop;
		}

		if (flip_flop % 2) {
			Display_A1_2(display, ts);
		} else {
			Display_A1_3(display, ts);
		}
	}
}

/* Редактировать время в формате чч.мм.сс */
void Display_EditTime1(Display *display,
				  	   Timestamp *ts)
{
	Display_A1_1(display, ts);
}

/* Редактировать время в формате чч.мм */
void Display_EditTime2(Display *display,
				  	   Timestamp *ts)
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
				  	  Timestamp *ts)
{
	Display_A1_3(display, ts);
}

void ModbusTask(void const * argument);

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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
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
	HAL_Delay(10);

	ws2812_init();

	Beep();
//	SetBacklight(255, 255, 255);

	HAL_TIM_Base_Start(&htim5);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);


	Wire_SetI2C(&hi2c1);
	Rtc_DS3231_init(DS3231_INTCN);

	Timestamp ts;
	Rtc_DS3231_get(&ts);

	Timestamp sun_rise1 = {0}, sun_set1 = {0};
	Timestamp sun_rise2 = {0}, sun_set2 = {0};

	double latitude_deg = DEFAULT_LATITUDE;
	double longitude_deg = DEFAULT_LONGITUDE;
	uint8_t tz_idx = MSK_ZONE;

	double tz = TIMEZONES[tz_idx];
	double latitude = latitude_deg * snm_DEG_TO_RAD;
	double longitude = longitude_deg * snm_DEG_TO_RAD;

	snm_Calculator calc;

	snm_CalculatorCreate(&calc);
	snm_CalculatorSetPoint(&calc, latitude, longitude);

	if (ts.mday == 0 || ts.mon == 0 || ts.year < 2015) {
		/* ... */
		ts.mday = 1;
		ts.mon = 1;
		ts.year = 2015;

		ts.hour = 0;
		ts.min = 0;
		ts.sec = 0;
		Rtc_DS3231_set(ts);
	}

	/* Yep, these params are magic! */
	KalmanFilterInit(&luminosity_filter,
					 4000.0f,
					 4000.0f,
					 0.125f);

	Display display;
	size_t n_places = 6;

	DisplayCreate(&display,
				  n_places,
				  &hspi1,
				  LED_DATA_LATCH_GPIO_Port,
				  LED_DATA_LATCH_Pin);

	LuminositySensorCreate(&luminosity_sensor);
	LuminositySensorBegin(&luminosity_sensor);
	HAL_ADC_Start_DMA(&hadc1, luminosity_sensor.value, 1);

	DISPLAY_MODES display_mode_curr = DISPLAY_MODE_A1_1;
	DISPLAY_MODES display_mode_prev = display_mode_curr;

	const uint32_t cycle_delay = 10u;
	const uint32_t btn_repeat_delay = 1000u;
	const uint32_t btn_fast_repeat_delay = 850u;

	uint32_t btn_fast_repeat_delay_accelerated = btn_fast_repeat_delay;

	int update_time = 0;
	int update_aging = 0;
	int update_latitude = 0;
	int update_longitude = 0;
	int update_timezone = 0;
	int update_celestial = 0;

	int8_t aging = Rtc_DS3231_get_aging();
	uint8_t brightness = 200u;

	uint32_t btn6_pressed_counter = 0;
	uint32_t btn7_pressed_counter = 0;
	uint32_t btn8_pressed_counter = 0;

	BTN_STATES btn1_state_curr = BTN1_STATE();
	BTN_STATES btn2_state_curr = BTN2_STATE();
	BTN_STATES btn3_state_curr = BTN3_STATE();
	BTN_STATES btn4_state_curr = BTN4_STATE();
	BTN_STATES btn5_state_curr = BTN5_STATE();
	BTN_STATES btn6_state_curr = BTN6_STATE();
	BTN_STATES btn7_state_curr = BTN7_STATE();
	BTN_STATES btn8_state_curr = BTN8_STATE();

	BTN_STATES btn1_state_prev = btn1_state_curr;
	BTN_STATES btn2_state_prev = btn2_state_curr;
	BTN_STATES btn3_state_prev = btn3_state_curr;
	BTN_STATES btn4_state_prev = btn4_state_curr;
	BTN_STATES btn5_state_prev = btn5_state_curr;
	BTN_STATES btn6_state_prev = btn6_state_curr;
	BTN_STATES btn7_state_prev = btn7_state_curr;
	BTN_STATES btn8_state_prev = btn8_state_curr;

	SetBrightness(brightness);

	float lum;
	float br = 0;

  /* Infinite loop */
	while (1) {

		if (LuminositySensorIsReady(&luminosity_sensor)) {
			lum = luminosity_sensor.value[0];
			lum = KalmanFilterUpdate(&luminosity_filter, lum);
			br = calc_brightness_linear(lum, BR_MIN, BR_MAX);
			SetBrightness(br);

			LuminositySensorBegin(&luminosity_sensor);
			HAL_ADC_Start_DMA(&hadc1, luminosity_sensor.value, 1);
		}

//		if (ts.hour >= 22) {
//			v_curr = (ts.min + 1) * 4;
//			if (v_prev != v_curr) {
//				SetBacklight((240-v_curr) % v_lim, v_curr % v_lim, 0);
//				v_prev = v_curr;
//			}
//		} else if (ts.hour <= 6) {
//			SetBacklight(0, 0, 255);
//		} else if (ts.hour <= 18) {
//			SetBacklight(255, 255, 255);
//		} else {
//			SetBacklight(100, 100, 100);
//		}

		btn1_state_curr = BTN1_STATE();
		btn2_state_curr = BTN2_STATE();
		btn3_state_curr = BTN3_STATE();
		btn4_state_curr = BTN4_STATE();
		btn5_state_curr = BTN5_STATE();
		btn6_state_curr = BTN6_STATE();
		btn7_state_curr = BTN7_STATE();
		btn8_state_curr = BTN8_STATE();

		if (sun_rise1.mday != ts.mday || update_celestial) {
			struct tm rise, set;

			snm_CalculatorSetTime(&calc, ts.mday, ts.mon, ts.year);
			snm_CalculatorSetDate(&calc, ts.hour, ts.min, ts.sec);
			snm_CalculatorSetPoint(&calc, latitude, longitude);

			snm_CalculatorSetTwilight(&calc, HORIZON_34arcmin);
			snm_CalculatorCalc(&calc);
			rise = snm_CalculatorGetDateAsTm(calc.sun_rise, tz);
			set = snm_CalculatorGetDateAsTm(calc.sun_set, tz);

			sun_rise1 = GetTimestampFromTm(rise);
			sun_set1 = GetTimestampFromTm(set);

			snm_CalculatorSetTwilight(&calc, TWILIGHT_CIVIL);
			snm_CalculatorCalc(&calc);
			rise = snm_CalculatorGetDateAsTm(calc.sun_rise, tz);
			set = snm_CalculatorGetDateAsTm(calc.sun_set, tz);

			sun_rise2 = GetTimestampFromTm(rise);
			sun_set2 = GetTimestampFromTm(set);
			update_celestial = 0;
		}

		{
			if ((ts.hour == 0 && ts.min == 0 && ts.sec == 0)
					|| (ts.hour == sun_rise2.hour && ts.min < sun_rise2.min)) {
				SetBacklight(255, 0, 0); // ночь
			} else if ((ts.hour > sun_set2.hour)
					|| (ts.hour == sun_set2.hour && ts.min > sun_set2.min)) {
				SetBacklight(255, 0, 0); // ночь
			} else
			{
				if ((ts.hour < sun_rise1.hour)
						|| (ts.hour == sun_rise1.hour && ts.min < sun_rise1.min)) {
					SetBacklight(255, 127, 0); // сумерки
				} else if ((ts.hour > sun_set1.hour)
						|| (ts.hour == sun_set1.hour && ts.min > sun_set1.min)) {
					SetBacklight(255, 127, 0); // сумерки
				} else {
					SetBacklight(255, 255, 255); // день
				}
			}
//			SetBacklight(255, 0, 0);
//			SetBacklight(255, 255, 255);
//			SetBacklight(255, 127, 0);
//			SetBacklight(0, 0, 255);
		}

		switch (display_mode_curr) {
		case DISPLAY_MODE_A1_1:
			Rtc_DS3231_get(&ts);
			Display_A1_1(&display, &ts);

			{
				if (btn1_state_prev != btn1_state_curr) {
					btn1_state_prev = btn1_state_curr;
					display_mode_prev = display_mode_curr;
					display_mode_curr = DISPLAY_MODE_EDIT_TIME_1;
					Beep();
				} else if (btn2_state_prev != btn2_state_curr) {
					btn2_state_prev = btn2_state_curr;

					if (btn2_state_curr == BTN_DOWN) {
						display_mode_prev = display_mode_curr;
						display_mode_curr = DISPLAY_MODE_A1_2;
						Beep();
					}
				} /*else if (btn8_state_curr == BTN_DOWN) {
					btn8_state_prev = btn8_state_curr;

					int day = 9, month = 10, year = 2018;
					int hour = 9, minute = 0, second = 0;

					double latitude = 55.75 * snm_DEG_TO_RAD;
					double longitude = 37.50 * snm_DEG_TO_RAD;
					double tz = +3.0;

					snm_Calculator calc;
					struct tm sun_rise, moon_rise;
					struct tm sun_set, moon_set;

					snm_CalculatorCreate(&calc);
					snm_CalculatorSetTime(&calc, day, month, year);
					snm_CalculatorSetDate(&calc, hour, minute, second);
					snm_CalculatorSetPoint(&calc, latitude, longitude);
					snm_CalculatorSetTwilight(&calc, HORIZON_34arcmin);
//					Beep();
					snm_CalculatorCalc(&calc);
//					Beep();

					sun_rise = snm_CalculatorGetDateAsTm(calc.sun_rise, tz);
					sun_set = snm_CalculatorGetDateAsTm(calc.sun_set, tz);

					moon_rise = snm_CalculatorGetDateAsTm(calc.moon_rise, tz);
					moon_set = snm_CalculatorGetDateAsTm(calc.moon_set, tz);

					Timestamp sun_rise2, sun_set2;
					Timestamp moon_rise2, moon_set2;

					sun_rise2.hour = sun_rise.tm_hour;
					sun_rise2.min = sun_rise.tm_min;
					sun_rise2.sec = sun_rise.tm_sec;

					sun_rise2.mday = sun_rise.tm_mday;
					sun_rise2.mon = sun_rise.tm_mon + 1;
					sun_rise2.year = sun_rise.tm_year;

					Display_S1_1(&display, &sun_rise2);
				}*/ else if (btn8_state_prev != btn8_state_curr || btn8_pressed_counter > btn_repeat_delay) {
					btn8_pressed_counter = btn_fast_repeat_delay;
					btn8_state_prev = btn8_state_curr;

					if (btn8_state_curr == BTN_DOWN) {
					}
				} else if (btn7_state_prev != btn7_state_curr || btn7_pressed_counter > btn_repeat_delay) {
					btn7_pressed_counter = btn_fast_repeat_delay;
					btn7_state_prev = btn7_state_curr;

					if (btn7_state_curr == BTN_DOWN) {
					}
				} else if (btn3_state_prev != btn3_state_curr) {
					btn3_state_prev = btn3_state_curr;

					if (btn3_state_curr == BTN_DOWN) {
						display_mode_prev = display_mode_curr;
						display_mode_curr = DISPLAY_MODE_S1_1;
						Beep();
					}
				}
			}
			break;

		case DISPLAY_MODE_A1_2:
			Rtc_DS3231_get(&ts);
			Display_A1_2(&display, &ts);
			{
				if (btn1_state_prev != btn1_state_curr) {
					btn1_state_prev = btn1_state_curr;
					display_mode_prev = display_mode_curr;
					display_mode_curr = DISPLAY_MODE_EDIT_TIME_2;
					Beep();
				} else if (btn2_state_prev != btn2_state_curr) {
					btn2_state_prev = btn2_state_curr;

					if (btn2_state_curr == BTN_DOWN) {
						display_mode_prev = display_mode_curr;
						display_mode_curr = DISPLAY_MODE_A1_3;
						Beep();
					}
				}
			}
			break;

		case DISPLAY_MODE_A1_3:
			Rtc_DS3231_get(&ts);
			Display_A1_3(&display, &ts);
			{
				if (btn1_state_prev != btn1_state_curr) {
					btn1_state_prev = btn1_state_curr;
					display_mode_prev = display_mode_curr;
					display_mode_curr = DISPLAY_MODE_EDIT_DATE;
					Beep();
				} else if (btn2_state_prev != btn2_state_curr) {
					btn2_state_prev = btn2_state_curr;

					if (btn2_state_curr == BTN_DOWN) {
						display_mode_prev = display_mode_curr;
						display_mode_curr = DISPLAY_MODE_A1_1;
						Beep();
					}
				}
			}
			break;

		case DISPLAY_MODE_A1_4:
//			Rtc_DS3231_get(&ts);
//			Display_A1_4(&display, &ts);
//			{
//				if (btn2_state_prev != btn2_state_curr) {
//					btn2_state_prev = btn2_state_curr;
//
//					if (btn2_state_curr == BTN_DOWN) {
//						display_mode_prev = display_mode_curr;
//						display_mode_curr = DISPLAY_MODE_A1_5;
//						Beep();
//					}
//				}
//			}
			break;

		case DISPLAY_MODE_A1_5:
//			Rtc_DS3231_get(&ts);
//			Display_A1_5(&display, &ts);
//			{
//				if (btn2_state_prev != btn2_state_curr) {
//					btn2_state_prev = btn2_state_curr;
//
//					if (btn2_state_curr == BTN_DOWN) {
//						display_mode_prev = display_mode_curr;
//						display_mode_curr = DISPLAY_MODE_A1_1;
//						Beep();
//					}
//				}
//			}
			break;

		case DISPLAY_MODE_S1_1:
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
					Display_S1_1(&display, &sun_rise1);
				}

				if (btn3_state_prev != btn3_state_curr) {
					if (btn3_state_curr == BTN_DOWN) {
						counter = 0;
						flip_flop = 0;
						display_mode_curr = DISPLAY_MODE_S1_2;
					}

					btn3_state_prev = btn3_state_curr;
				}

				if (btn1_state_prev != btn1_state_curr
					|| btn2_state_prev != btn2_state_curr
					|| btn4_state_prev != btn4_state_curr
					|| btn5_state_prev != btn5_state_curr
					|| btn6_state_prev != btn6_state_curr
					|| btn7_state_prev != btn7_state_curr
					|| btn8_state_prev != btn8_state_curr)
				{
					counter = 0;
					flip_flop = 0;
					display_mode_curr = display_mode_prev;
//					btn1_state_prev = btn1_state_curr;
					btn2_state_prev = btn2_state_curr;
//					btn4_state_prev = btn4_state_curr;
//					btn5_state_prev = btn5_state_curr;
//					btn6_state_prev = btn6_state_curr;
//					btn7_state_prev = btn7_state_curr;
//					btn8_state_prev = btn8_state_curr;
				}
			}
			break;

		case DISPLAY_MODE_S1_2:
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
					Display_S1_1(&display, &sun_set1);
				}

				if (btn3_state_prev != btn3_state_curr) {
					if (btn3_state_curr == BTN_DOWN) {
						counter = 0;
						flip_flop = 0;
						display_mode_curr = DISPLAY_MODE_S1_1;
					}

					btn3_state_prev = btn3_state_curr;
				}

				if (btn1_state_prev != btn1_state_curr
					|| btn2_state_prev != btn2_state_curr
					|| btn4_state_prev != btn4_state_curr
					|| btn5_state_prev != btn5_state_curr
					|| btn6_state_prev != btn6_state_curr
					|| btn7_state_prev != btn7_state_curr
					|| btn8_state_prev != btn8_state_curr)
				{
					counter = 0;
					flip_flop = 0;
					display_mode_curr = display_mode_prev;
//					btn1_state_prev = btn1_state_curr;
					btn2_state_prev = btn2_state_curr;
//					btn4_state_prev = btn4_state_curr;
//					btn5_state_prev = btn5_state_curr;
//					btn6_state_prev = btn6_state_curr;
//					btn7_state_prev = btn7_state_curr;
//					btn8_state_prev = btn8_state_curr;
				}
			}
			break;

		case DISPLAY_MODE_EDIT_TIME_1:
			Display_EditTime1(&display, &ts);
			{
				if (btn1_state_prev != btn1_state_curr) {
					btn1_state_prev = btn1_state_curr;

					if (btn1_state_curr == BTN_UP) {
						if (update_time) {
							Rtc_DS3231_set(ts);
							update_time = 0;
							update_celestial = 1;
						}

						display_mode_curr = display_mode_prev;
					}
				} else if (btn8_state_prev != btn8_state_curr || btn8_pressed_counter > btn_repeat_delay) {
					if (btn8_state_curr == BTN_DOWN) {
						ts.sec += 1;
						ts.sec %= 60;
						update_time = 1;
					}

					btn8_pressed_counter = btn_fast_repeat_delay;
					btn8_state_prev = btn8_state_curr;
				} else if (btn7_state_prev != btn7_state_curr || btn7_pressed_counter > btn_repeat_delay) {
					if (btn7_state_curr == BTN_DOWN) {
						ts.min += 1;
						ts.min %= 60;
						update_time = 1;
					}

					btn7_pressed_counter = btn_fast_repeat_delay;
					btn7_state_prev = btn7_state_curr;
				} else if (btn6_state_prev != btn6_state_curr || btn6_pressed_counter > btn_repeat_delay) {
					if (btn6_state_curr == BTN_DOWN) {
						ts.hour += 1;
						ts.hour %= 24;
						update_time = 1;
					}

					btn6_pressed_counter = btn_fast_repeat_delay;
					btn6_state_prev = btn6_state_curr;
				} else if (btn2_state_prev != btn2_state_curr) {
					if (btn2_state_curr == BTN_DOWN) {
						display_mode_curr = DISPLAY_MODE_EDIT_AGING;
					}

					btn2_state_prev = btn2_state_curr;
				}
			}
			break;

		case DISPLAY_MODE_EDIT_TIME_2:
			Display_EditTime2(&display, &ts);
			{
				if (btn1_state_prev != btn1_state_curr) {
					btn1_state_prev = btn1_state_curr;

					if (btn1_state_curr == BTN_UP) {
						if (update_time) {
							Rtc_DS3231_set(ts);
							update_time = 0;
							update_celestial = 1;
						}

						display_mode_curr = display_mode_prev;
					}
				} else if (btn7_state_prev != btn7_state_curr || btn7_pressed_counter > btn_repeat_delay) {
					if (btn7_state_curr == BTN_DOWN) {
						ts.sec = 0;
						ts.min += 1;
						ts.min %= 60;
						update_time = 1;
					}

					btn7_pressed_counter = btn_fast_repeat_delay;
					btn7_state_prev = btn7_state_curr;
				} else if (btn6_state_prev != btn6_state_curr || btn6_pressed_counter > btn_repeat_delay) {
					if (btn6_state_curr == BTN_DOWN) {
						ts.sec = 0;
						ts.hour += 1;
						ts.hour %= 23;
						update_time = 1;
					}

					btn6_pressed_counter = btn_fast_repeat_delay;
					btn6_state_prev = btn6_state_curr;
				}
			}
			break;

		case DISPLAY_MODE_EDIT_AGING:
			{
				Display_EditAging(&display, aging);

				if (btn1_state_prev != btn1_state_curr) {
					btn1_state_prev = btn1_state_curr;

					if (btn1_state_curr == BTN_UP) {
						if (update_aging) {
							Rtc_DS3231_set_aging(aging);
							update_aging = 0;
						}
					}

					display_mode_curr = display_mode_prev;
				} else {
					if (btn8_state_prev != btn8_state_curr) {
						if (btn8_state_curr == BTN_DOWN) {
							--aging;
							update_aging = 1;
						}

						btn8_state_prev = btn8_state_curr;
					} else if (btn7_state_prev != btn7_state_curr) {
						if (btn7_state_curr == BTN_DOWN) {
							++aging;
							update_aging = 1;
						}

						btn7_state_prev = btn7_state_curr;
					} else if (btn2_state_prev != btn2_state_curr) {
						if (btn2_state_curr == BTN_DOWN) {
							display_mode_curr = DISPLAY_MODE_EDIT_LATITUDE;
						}

						btn2_state_prev = btn2_state_curr;
					}
				}
			}
			break;

		case DISPLAY_MODE_EDIT_LATITUDE:
			{
				Display_EditLatitude(&display, latitude_deg);

				if (btn1_state_prev != btn1_state_curr) {
					btn1_state_prev = btn1_state_curr;

					if (btn1_state_curr == BTN_UP) {
						if (update_latitude) {
							/* TODO: save latitude to flash */
							update_latitude = 0;
							update_celestial = 1;
						}
					}

					display_mode_curr = display_mode_prev;
				} else {
					if (btn8_state_prev != btn8_state_curr || btn8_pressed_counter > btn_repeat_delay) {
						if (btn8_state_curr == BTN_DOWN) {
							if (latitude_deg > -90.00)
								latitude_deg -= 0.01;

							latitude = latitude_deg * snm_DEG_TO_RAD;
							update_latitude = 1;
						} else {
							btn_fast_repeat_delay_accelerated = btn_fast_repeat_delay;
						}

						btn8_pressed_counter = btn_fast_repeat_delay_accelerated;
						btn_fast_repeat_delay_accelerated += 10;
						btn8_state_prev = btn8_state_curr;
					} else if (btn7_state_prev != btn7_state_curr || btn7_pressed_counter > btn_repeat_delay) {
						if (btn7_state_curr == BTN_DOWN) {
							if (latitude_deg < +90.00)
								latitude_deg += 0.01;

							latitude = latitude_deg * snm_DEG_TO_RAD;
							update_latitude = 1;
						} else {
							btn_fast_repeat_delay_accelerated = btn_fast_repeat_delay;
						}

						btn7_pressed_counter = btn_fast_repeat_delay_accelerated;
						btn_fast_repeat_delay_accelerated += 10;
						btn7_state_prev = btn7_state_curr;
					} else if (btn2_state_prev != btn2_state_curr) {
						if (btn2_state_curr == BTN_DOWN) {
							display_mode_curr = DISPLAY_MODE_EDIT_LONGITUDE;
						}

						btn2_state_prev = btn2_state_curr;
					}
				}
			}
			break;

		case DISPLAY_MODE_EDIT_LONGITUDE:
			{
				Display_EditLongitude(&display, longitude_deg);

				if (btn1_state_prev != btn1_state_curr) {
					btn1_state_prev = btn1_state_curr;

					if (btn1_state_curr == BTN_UP) {
						if (update_longitude) {
							/* TODO: save longitude to flash */
							update_longitude = 0;
							update_celestial = 1;
						}
					}

					display_mode_curr = display_mode_prev;
				} else {
					if (btn8_state_prev != btn8_state_curr || btn8_pressed_counter > btn_repeat_delay) {
						if (btn8_state_curr == BTN_DOWN) {
							if (longitude_deg > -180.00)
								longitude_deg -= 0.01;

							longitude = longitude_deg * snm_DEG_TO_RAD;
							update_longitude = 1;
						} else {
							btn_fast_repeat_delay_accelerated = btn_fast_repeat_delay;
						}

						btn8_pressed_counter = btn_fast_repeat_delay_accelerated;
						btn_fast_repeat_delay_accelerated += 10;
						btn8_state_prev = btn8_state_curr;
					} else if (btn7_state_prev != btn7_state_curr || btn7_pressed_counter > btn_repeat_delay) {
						if (btn7_state_curr == BTN_DOWN) {
							if (longitude_deg < +180.00)
								longitude_deg += 0.01;

							longitude = longitude_deg * snm_DEG_TO_RAD;
							update_longitude = 1;
						} else {
							btn_fast_repeat_delay_accelerated = btn_fast_repeat_delay;
						}

						btn7_pressed_counter = btn_fast_repeat_delay_accelerated;
						btn_fast_repeat_delay_accelerated += 10;
						btn7_state_prev = btn7_state_curr;
					} else if (btn2_state_prev != btn2_state_curr) {
						if (btn2_state_curr == BTN_DOWN) {
							display_mode_curr = DISPLAY_MODE_EDIT_TIMEZONE;
						}

						btn2_state_prev = btn2_state_curr;
					}
				}
			}
			break;

		case DISPLAY_MODE_EDIT_TIMEZONE:
			{
				Display_EditTimezone(&display, tz);

				if (btn1_state_prev != btn1_state_curr) {
					btn1_state_prev = btn1_state_curr;

					if (btn1_state_curr == BTN_UP) {
						if (update_timezone) {
							/* TODO: save tz_idx to flash */
//							HAL_StatusTypeDef flash_ok;
//
//							flash_ok = HAL_ERROR;
//							while(flash_ok != HAL_OK) {
//								flash_ok = HAL_FLASH_Unlock();
//							}
//
//							flash_ok = HAL_ERROR;
//							while(flash_ok != HAL_OK) {
//								flash_ok = HAL_FLASH_Program(TYPEPROGRAM_BYTE, 0x08100000, 0x7777);
//							}
//
//							flash_ok = HAL_ERROR;
//							while(flash_ok != HAL_OK) {
//								flash_ok = HAL_FLASH_Lock();
//							}
							update_timezone = 0;
							update_celestial = 1;
						}
					}

					display_mode_curr = display_mode_prev;
				} else {
					if (btn8_state_prev != btn8_state_curr) {
						if (btn8_state_curr == BTN_DOWN) {
							tz_idx = (tz_idx-1) % N_TIMEZONES;
							tz = TIMEZONES[tz_idx];
							update_timezone = 1;
						}

						btn8_state_prev = btn8_state_curr;
					} else if (btn7_state_prev != btn7_state_curr) {
						if (btn7_state_curr == BTN_DOWN) {
							tz_idx = (tz_idx+1) % N_TIMEZONES;
							tz = TIMEZONES[tz_idx];
							update_timezone = 1;
						}

						btn7_state_prev = btn7_state_curr;
					} else if (btn2_state_prev != btn2_state_curr) {
						if (btn2_state_curr == BTN_DOWN) {
							display_mode_curr = DISPLAY_MODE_EDIT_AGING;
						}

						btn2_state_prev = btn2_state_curr;
					}
				}
			}
			break;

		case DISPLAY_MODE_EDIT_DATE:
			Display_EditDate(&display, &ts);
			{
				if (btn1_state_prev != btn1_state_curr) {
					btn1_state_prev = btn1_state_curr;

					if (btn1_state_curr == BTN_UP) {
						if (update_time) {
							Rtc_DS3231_set(ts);
							update_time = 0;
						}

						display_mode_curr = display_mode_prev;
					}
				} else if (btn8_state_prev != btn8_state_curr || btn8_pressed_counter > btn_repeat_delay) {
					if (btn8_state_curr == BTN_DOWN) {
						if (ts.year < 2015)
							ts.year = 2015;

						ts.year -= 2000;
						ts.year += 1;
						ts.year %= 100;
						ts.year += 2000;
						update_time = 1;
					}

					btn8_pressed_counter = btn_fast_repeat_delay;
					btn8_state_prev = btn8_state_curr;
				} else if (btn7_state_prev != btn7_state_curr || btn7_pressed_counter > btn_repeat_delay) {
					if (btn7_state_curr == BTN_DOWN) {
						ts.mon += 1;
						ts.mon %= 12;
						update_time = 1;
					}

					btn7_pressed_counter = btn_fast_repeat_delay;
					btn7_state_prev = btn7_state_curr;
				} else if (btn6_state_prev != btn6_state_curr || btn6_pressed_counter > btn_repeat_delay) {
					if (btn6_state_curr == BTN_DOWN) {
						ts.mday = (ts.mday % 31) + 1;
						update_time = 1;
					}

					btn6_pressed_counter = btn_fast_repeat_delay;
					btn6_state_prev = btn6_state_curr;
				}
			}
			break;
		}

		if (btn6_state_curr == BTN_DOWN)
			btn6_pressed_counter += cycle_delay;

		if (btn7_state_curr == BTN_DOWN)
			btn7_pressed_counter += cycle_delay;

		if (btn8_state_curr == BTN_DOWN)
			btn8_pressed_counter += cycle_delay;

		HAL_Delay(cycle_delay);
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
