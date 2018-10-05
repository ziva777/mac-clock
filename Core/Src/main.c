
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
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

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

//uint32_t tt[1];
//uint32_t flag_adc_dma;


/* ----------------------- Defines ------------------------------------------*/
#define REG_INPUT_START                 ( 1 )
#define REG_INPUT_NREGS                 ( 12 )

#define REG_HOLDING_START               ( 1 )
#define REG_HOLDING_NREGS               ( 14 )

#define REG_HOLDING_STORE_START         ( 2001 )
#define REG_HOLDING_STORE_NREGS         ( 22 )

#define REG_HOLDING_PROTECT_START       ( 9991 )
#define REG_HOLDING_PROTECT_NREGS       ( 1 )

#define REG_COILS_START     1
#define REG_COILS_SIZE      8

#define REG_DISC_START     	1
#define REG_DISC_SIZE      	8

/* ----------------------- Static variables ---------------------------------*/
static USHORT   usRegInputStart = REG_INPUT_START;
static USHORT   usRegInputBuf[REG_INPUT_NREGS];

static USHORT   usRegHoldingStart = REG_HOLDING_START;
static USHORT   usRegHoldingBuf[REG_HOLDING_NREGS];

static USHORT   usRegHoldingStoreStart = REG_HOLDING_STORE_START;
static USHORT   usRegHoldingStoreBuf[REG_HOLDING_STORE_NREGS];

static USHORT   usRegHoldingProtectStart = REG_HOLDING_PROTECT_START;
static USHORT   usRegHoldingProtectBuf[REG_HOLDING_PROTECT_NREGS];

static unsigned char ucRegCoilsBuf[REG_COILS_SIZE / 8];

static unsigned char ucRegDiscBuf[REG_DISC_SIZE / 8] = { 0 };

static uint32_t lock_nesting_count = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */



void __critical_enter( void )
{
	__disable_irq();
	++lock_nesting_count;
}

void __critical_exit( void )
{
	/* Unlock interrupts only when we are exiting the outermost nested  call. */
	--lock_nesting_count;
	if (lock_nesting_count == 0) {
		__enable_irq();
	}
}





uint8_t BcdToUint8(uint8_t val) {
    return val - 6 * (val >> 4);
}

uint8_t BcdToBin24Hour(uint8_t bcdHour) {
    uint8_t hour;

    if (bcdHour & 0x40) {
        // 12 hour mode, convert to 24
        int isPm = ((bcdHour & 0x20) != 0);

        hour = BcdToUint8(bcdHour & 0x1f);

        if (isPm) {
           hour += 12;
        }
    } else {
        hour = BcdToUint8(bcdHour);
    }

    return hour;
}

typedef enum {
    BTN_UP = GPIO_PIN_SET,
    BTN_DOWN = GPIO_PIN_RESET
} BTN_STATES;

#define BTN1_STATE()   (BTN_STATES)HAL_GPIO_ReadPin(BTN0_GPIO_Port, BTN0_Pin)
#define BTN2_STATE()   (BTN_STATES)HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin)
#define BTN3_STATE()   (BTN_STATES)HAL_GPIO_ReadPin(BTN2_GPIO_Port, BTN2_Pin)
#define BTN4_STATE()   (BTN_STATES)HAL_GPIO_ReadPin(BTN3_GPIO_Port, BTN3_Pin)
#define BTN5_STATE()   (BTN_STATES)HAL_GPIO_ReadPin(BTN4_GPIO_Port, BTN4_Pin)
#define BTN6_STATE()   (BTN_STATES)HAL_GPIO_ReadPin(BTN5_GPIO_Port, BTN5_Pin)
#define BTN7_STATE()   (BTN_STATES)HAL_GPIO_ReadPin(BTN6_GPIO_Port, BTN6_Pin)
#define BTN8_STATE()   (BTN_STATES)HAL_GPIO_ReadPin(BTN7_GPIO_Port, BTN7_Pin)

static void SetBrightness(uint16_t br)
{
    htim5.Instance->CCR4 = br;
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
}

//static uint32_t GetLuminosity()
//{
////	uint32_t tt[1];
////	uint32_t flag_adc_dma;
//}
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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	eMBErrorCode eStatus;
	DISPLAY_MODES display_mode_curr = DISPLAY_MODE_A1_1;
	DISPLAY_MODES display_mode_prev = display_mode_curr;

	Display display;
	size_t n_places = 6;

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_SPI3_Init();
  MX_USART3_UART_Init();
  MX_TIM5_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */


  ws2812_init();
  ws2812_pixel_rgb_to_buf_dma(127, 31, 0, 0);
  ws2812_pixel_rgb_to_buf_dma(127, 31, 0, 1);
  HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t*)BUF_DMA,
	ARRAY_LEN);





  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_Delay(10);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);


  HAL_TIM_Base_Start(&htim5);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);


  Wire_SetI2C(&hi2c1);
  Rtc_DS3231_init(DS3231_INTCN);
  Timestamp ts;

//  uint8_t buf[20];
//  buf[0] = 0xD0;
//  HAL_GPIO_WritePin(BMP280_NSS_GPIO_Port, BMP280_NSS_Pin, GPIO_PIN_RESET);
//  HAL_SPI_Transmit(&hspi3, buf, 1, 100);
//  HAL_SPI_Receive(&hspi3, buf, 1, 100);
//  HAL_GPIO_WritePin(BMP280_NSS_GPIO_Port, BMP280_NSS_Pin, GPIO_PIN_SET);
//
//  buf[0] = 0x00;
//  HAL_I2C_Master_Transmit(&hi2c1, 0xD0, buf, 1, 100);
//  HAL_I2C_Master_Receive(&hi2c1, 0xD0, buf, 19, 100);
//  buf[1] = buf[0];

	DisplayCreate(&display,
				  n_places,
				  &hspi1,
				  LED_DATA_LATCH_GPIO_Port,
				  LED_DATA_LATCH_Pin);

//	HAL_SPI_Transmit(&hspi1, aa_dma, 7, 1000);
//	HAL_GPIO_TogglePin(LED_DATA_LATCH_GPIO_Port, LED_DATA_LATCH_Pin);
//	HAL_Delay(1);
//	HAL_GPIO_TogglePin(LED_DATA_LATCH_GPIO_Port, LED_DATA_LATCH_Pin);



//	flag_adc_dma = 0;
//	HAL_ADC_Start_DMA(&hadc1, tt, 1);

	LuminositySensorCreate(&luminosity_sensor);
	LuminositySensorBegin(&luminosity_sensor);
	HAL_ADC_Start_DMA(&hadc1, luminosity_sensor.value, 1);

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
//  MX_FREERTOS_Init();

  /* Start scheduler */
//  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
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


	  eStatus = eMBInit(MB_RTU, 1, 2, 115200, MB_PAR_NONE);
	  eStatus = eMBEnable();


  while (1)
  {
//		if ( flag_adc_dma == 1 ) {
//			flag_adc_dma = 0;
//			usRegInputBuf[0] = tt[0];
//			HAL_ADC_Start_DMA(&hadc1, tt, 1);
////			DisplayWriteUint(&display, tt[0]);
//		}

//	  while (1) {
//		if (LuminositySensorIsReady(&luminosity_sensor)) {
//			uint32_t tt = luminosity_sensor.value[0];
//
//			DisplayWriteUint(&display, tt);
//			DisplaySync(&display);
//
//			LuminositySensorBegin(&luminosity_sensor);
//			HAL_ADC_Start_DMA(&hadc1, luminosity_sensor.value, 1);
//		} else {
//			DisplayWriteUint(&display, 10000);
//			DisplaySync(&display);
//		}
//		HAL_Delay(cycle_delay);
//	  }

		eMBPoll();

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
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
				//if (btn1_state_prev != btn1_state_curr) {
					//btn1_state_prev = btn1_state_curr;

//					if (btn1_state_curr == BTN_UP) {
//						if (update_aging) {
//							Rtc_DS3231_set_aging(aging);
//							update_aging = 0;
//						}
//
//						display_mode_curr = display_mode_prev;
//					}
//				} else if  (btn8_state_prev != btn8_state_curr) {
//					if (btn8_state_curr == BTN_DOWN) {
//						++aging;
//						update_aging = 1;
//						Display_EditAging(&display, aging);
//					}
//				}


			//}
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
//	  DisplayWriteStr(&display, "CLOUDY", n_places);
//	  HAL_Delay(1000);
//
//	  DisplayWriteStr(&display, "DOLLAR", n_places);
//	  HAL_Delay(1000);
//
//	  DisplayWriteUint(&display, 0);
//	  HAL_Delay(1000);
//
//	  DisplayWriteUint(&display, 123456);
//	  HAL_Delay(1000);

//	  buf[0] = 0x00;
//	  HAL_I2C_Master_Transmit(&hi2c1, 0xD0, buf, 1, 100);
//	  HAL_I2C_Master_Receive(&hi2c1, 0xD0, buf, 19, 100);
//
//	  //DisplayWriteUint(&display, i++);
//	  //DisplayWriteUint(&display, (buf[0] & 0b11110000) >> 4);
//	  //DisplayWriteUint(&display, (buf[0] & 0b00001111) >> 0);
//
//	  uint8_t s = ((buf[0] & 0b11110000) >> 4) * 10 + (buf[0] & 0b00001111) >> 0;
//	  uint8_t m = ((buf[1] & 0b11110000) >> 4) * 10 + (buf[1] & 0b00001111) >> 0;
//	  uint8_t h = BcdToBin24Hour(buf[2]);
//
//	  DisplayWriteUint(&display,
//			  h * 10000 + m * 100 + s);
//	  DisplaySync(&display);
//
//	  HAL_Delay(10);

//		c[0] = CH_D;
//		c[1] = CH_U;
//		c[2] = CH_O;
//		c[3] = CH_L;
//		c[4] = CH_C;
//		c[5] = CH_BLANK;
//
//		DisplayWrite(&display, c, d, n_places);
//
////		for (int i = 0; i != 256; ++i) {
////			htim5.Instance->CCR4 = i;
////			HAL_Delay(10);
////		}
////
//		HAL_Delay(1000);
////
////		for (int i = 255; i != 0; --i) {
////			htim5.Instance->CCR4 = i;
////			HAL_Delay(10);
////		}
//
//		//HAL_Delay(1000);
//
////		htim5.Instance->CCR4 = 250;
////		HAL_Delay(1000);
////
//		c[0] = CH_R;
//		c[1] = CH_I;
//		c[2] = CH_A;
//		c[3] = CH_F;
//		c[4] = CH_BLANK;
//		c[5] = CH_BLANK;
//
//		DisplayWrite(&display, c, d, n_places);
////		htim5.Instance->CCR4 = 100;
//		HAL_Delay(1000);
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* USER CODE BEGIN 4 */

eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if( ( usAddress >= REG_INPUT_START )
        && ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegInputStart );
        while( usNRegs > 0 )
        {
            *pucRegBuffer++ =
                ( unsigned char )( usRegInputBuf[iRegIndex] >> 8 );
            *pucRegBuffer++ =
                ( unsigned char )( usRegInputBuf[iRegIndex] & 0xFF );
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs,
                 eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if ( ( usAddress >= REG_HOLDING_START ) && ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegHoldingStart );
        switch ( eMode )
        {
        case MB_REG_READ:
            while( usNRegs > 0 )
            {
                *pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] >> 8 );
                *pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] & 0xFF );
                iRegIndex++;
                usNRegs--;
            }
            break;

        case MB_REG_WRITE:
            while( usNRegs > 0 )
            {
                usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
                iRegIndex++;
                usNRegs--;
            }
        }
    }
    else if ( ( usAddress >= REG_HOLDING_STORE_START ) && ( usAddress + usNRegs <= REG_HOLDING_STORE_START + REG_HOLDING_STORE_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegHoldingStoreStart );
        switch ( eMode )
        {
        case MB_REG_READ:
            while( usNRegs > 0 )
            {
                *pucRegBuffer++ = ( unsigned char )( usRegHoldingStoreBuf[iRegIndex] >> 8 );
                *pucRegBuffer++ = ( unsigned char )( usRegHoldingStoreBuf[iRegIndex] & 0xFF );
                iRegIndex++;
                usNRegs--;
            }
            break;

        case MB_REG_WRITE:
            while( usNRegs > 0 )
            {
                usRegHoldingStoreBuf[iRegIndex] = *pucRegBuffer++ << 8;
                usRegHoldingStoreBuf[iRegIndex] |= *pucRegBuffer++;
                iRegIndex++;
                usNRegs--;
            }
        }
    }
    else if ( ( usAddress >= REG_HOLDING_PROTECT_START ) && ( usAddress + usNRegs <= REG_HOLDING_PROTECT_START + REG_HOLDING_PROTECT_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegHoldingProtectStart );
        switch ( eMode )
        {
        case MB_REG_READ:
            while( usNRegs > 0 )
            {
                *pucRegBuffer++ = ( unsigned char )( usRegHoldingProtectBuf[iRegIndex] >> 8 );
                *pucRegBuffer++ = ( unsigned char )( usRegHoldingProtectBuf[iRegIndex] & 0xFF );
                iRegIndex++;
                usNRegs--;
            }
            break;

        case MB_REG_WRITE:
            while( usNRegs > 0 )
            {
                usRegHoldingProtectBuf[iRegIndex] = *pucRegBuffer++ << 8;
                usRegHoldingProtectBuf[iRegIndex] |= *pucRegBuffer++;
                iRegIndex++;
                usNRegs--;
            }
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}


eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils,
               eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iNCoils = ( int )usNCoils;
    unsigned short  usBitOffset;

    /* Check if we have registers mapped at this block. */
    if( ( usAddress >= REG_COILS_START ) &&
        ( usAddress + usNCoils <= REG_COILS_START + REG_COILS_SIZE ) )
    {
        usBitOffset = ( unsigned short )( usAddress - REG_COILS_START );
        switch ( eMode )
        {
                /* Read current values and pass to protocol stack. */
            case MB_REG_READ:
                while( iNCoils > 0 )
                {
                    *pucRegBuffer++ =
                        xMBUtilGetBits( ucRegCoilsBuf, usBitOffset,
                                        ( unsigned char )( iNCoils >
                                                           8 ? 8 :
                                                           iNCoils ) );
                    iNCoils -= 8;
                    usBitOffset += 8;
                }
                break;

                /* Update current register values. */
            case MB_REG_WRITE:
                while( iNCoils > 0 )
                {
                    xMBUtilSetBits( ucRegCoilsBuf, usBitOffset,
                                    ( unsigned char )( iNCoils > 8 ? 8 : iNCoils ),
                                    *pucRegBuffer++ );
                    iNCoils -= 8;
                    usBitOffset += 8;
                }
                break;
        }

    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    short           iNDiscrete = ( short )usNDiscrete;
    unsigned short  usBitOffset;

    /* Check if we have registers mapped at this block. */
    if( ( usAddress >= REG_DISC_START ) &&
        ( usAddress + usNDiscrete <= REG_DISC_START + REG_DISC_SIZE ) )
    {
        usBitOffset = ( unsigned short )( usAddress - REG_DISC_START );
        while( iNDiscrete > 0 )
        {
            *pucRegBuffer++ =
                xMBUtilGetBits( ucRegDiscBuf, usBitOffset,
                                ( unsigned char )( iNDiscrete >
                                                   8 ? 8 : iNDiscrete ) );
            iNDiscrete -= 8;
            usBitOffset += 8;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
