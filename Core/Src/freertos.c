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
#include "display.h"
#include "rtc_ds3221.h"
#include "luminosity_sensor.h"
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

	DISPLAY_MODE_EDIT_TIME_1,
	DISPLAY_MODE_EDIT_TIME_2,
	DISPLAY_MODE_EDIT_AGING,
	DISPLAY_MODE_EDIT_DATE
} DISPLAY_MODES;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
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
osThreadId modbus_task_handle;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* Установка яркости */
static void SetBrightness(uint16_t br)
{
    htim5.Instance->CCR4 = br;
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
}

/* Бип */
void Beep()
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_Delay(10);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
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
	HAL_Delay(100);

	ws2812_init();
	ws2812_pixel_rgb_to_buf_dma(255,   0,   0, 0);
	ws2812_pixel_rgb_to_buf_dma(  0,   0, 255, 1);
	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, BUF_DMA, ARRAY_LEN);

	Beep();

	HAL_TIM_Base_Start(&htim5);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);


	Wire_SetI2C(&hi2c1);
	Rtc_DS3231_init(DS3231_INTCN);
	Timestamp ts;


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

	int update_time = 0;
	int update_aging = 0;

	int8_t aging = Rtc_DS3231_get_aging();
	uint8_t brightness = 200u;

	uint32_t btn6_pressed_counter = 0;
	uint32_t btn7_pressed_counter = 0;
	uint32_t btn8_pressed_counter = 0;

	BTN_STATES btn1_state_curr = BTN1_STATE();
	BTN_STATES btn2_state_curr = BTN2_STATE();
	BTN_STATES btn6_state_curr = BTN6_STATE();
	BTN_STATES btn7_state_curr = BTN7_STATE();
	BTN_STATES btn8_state_curr = BTN8_STATE();

	BTN_STATES btn1_state_prev = btn1_state_curr;
	BTN_STATES btn2_state_prev = btn2_state_curr;
	BTN_STATES btn6_state_prev = btn6_state_curr;
	BTN_STATES btn7_state_prev = btn7_state_curr;
	BTN_STATES btn8_state_prev = btn8_state_curr;

	SetBrightness(brightness);


  /* Infinite loop */
  for(;;)
  {
		btn1_state_curr = BTN1_STATE();
		btn2_state_curr = BTN2_STATE();
		btn6_state_curr = BTN6_STATE();
		btn7_state_curr = BTN7_STATE();
		btn8_state_curr = BTN8_STATE();

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
				} else if (btn8_state_prev != btn8_state_curr || btn8_pressed_counter > btn_repeat_delay) {
					btn8_pressed_counter = btn_fast_repeat_delay;
					btn8_state_prev = btn8_state_curr;

					if (btn8_state_curr == BTN_DOWN) {
						if (brightness < 255)
							SetBrightness(++brightness);
					}
				} else if (btn7_state_prev != btn7_state_curr || btn7_pressed_counter > btn_repeat_delay) {
					btn7_pressed_counter = btn_fast_repeat_delay;
					btn7_state_prev = btn7_state_curr;

					if (btn7_state_curr == BTN_DOWN) {
						if (brightness)
							SetBrightness(--brightness);
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
						display_mode_curr = DISPLAY_MODE_A1_4;
						Beep();
					}
				}
			}
			break;

		case DISPLAY_MODE_A1_4:
			Rtc_DS3231_get(&ts);
			Display_A1_4(&display, &ts);
			{
				if (btn2_state_prev != btn2_state_curr) {
					btn2_state_prev = btn2_state_curr;

					if (btn2_state_curr == BTN_DOWN) {
						display_mode_prev = display_mode_curr;
						display_mode_curr = DISPLAY_MODE_A1_5;
						Beep();
					}
				}
			}
			break;

		case DISPLAY_MODE_A1_5:
			Rtc_DS3231_get(&ts);
			Display_A1_5(&display, &ts);
			{
				if (btn2_state_prev != btn2_state_curr) {
					btn2_state_prev = btn2_state_curr;

					if (btn2_state_curr == BTN_DOWN) {
						display_mode_prev = display_mode_curr;
						display_mode_curr = DISPLAY_MODE_A1_1;
						Beep();
					}
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
	                    ts.hour %= 23;
	                    update_time = 1;
	                }

	                btn6_pressed_counter = btn_fast_repeat_delay;
	                btn6_state_prev = btn6_state_curr;
	            } else if (btn2_state_prev != btn2_state_curr) {
	            	if (btn2_state_curr == BTN_DOWN) {
	            		display_mode_curr = DISPLAY_MODE_EDIT_AGING;
	            	}
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
					if  (btn8_state_prev != btn8_state_curr) {
						if (btn8_state_curr == BTN_DOWN) {
							--aging;
							update_aging = 1;
						}

						btn8_state_prev = btn8_state_curr;
					} else if  (btn7_state_prev != btn7_state_curr) {
						if (btn7_state_curr == BTN_DOWN) {
							++aging;
							update_aging = 1;
						}

						btn7_state_prev = btn7_state_curr;
					}

					Display_EditAging(&display, aging);
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
