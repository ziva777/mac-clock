
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
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

//uint16_t aa_dma[ 2 * 3 * 8 * 2 + 1 ];
//uint32_t aa_dma[ 49 ] = {10,25,10,25,10,25,10,25,10,25,10,25,10,25,10,25,10,25,10,25,10,25,10,25,10,25,10,25,10,25,10,25,10,25,10,25,10,25,10,25,10,25,10,25,10,25,10,25,10};
//uint32_t aa_dma[ 20 ] = {10,1000,10,1000,10,1000,10,1000,10,1000,10,1000,10,1000,10,1000,10,1000,10,1000};
//uint16_t ii = 0;

//static const uint16_t A = 0b0010110110011001;



#define N_BITS_IN_BYTE 8



#define A_SEGMENT  7
#define B_SEGMENT  8
#define C_SEGMENT 10
#define D_SEGMENT 13
#define E_SEGMENT 15
#define F_SEGMENT 14
#define G_SEGMENT  0
#define H_SEGMENT  4

#define K_SEGMENT  5
#define M_SEGMENT  6
#define N_SEGMENT  9

#define U_SEGMENT  3
#define P_SEGMENT 11

#define T_SEGMENT  1
#define S_SEGMENT  2
#define R_SEGMENT 12


#define POW2(a)         (1 << a)

#define PASTE(a,b)      a ## b
#define XPASTE(a,b)     PASTE(a,b)
#define PP_NARG(...)    PP_NARG_(__VA_ARGS__, PP_RSEQ_N())
#define PP_NARG_(...)   PP_ARG_N(__VA_ARGS__)

#define PP_ARG_N( \
        _1, _2, _3, _4, _5, _6, _7, _8, _9,_10,  \
        _11,_12,_13,_14,_15,_16,_17,_18,_19,_20, \
        _21,_22,_23,_24,_25,_26,_27,_28,_29,_30, \
        _31,_32,_33,_34,_35,_36,_37,_38,_39,_40, \
        _41,_42,_43,_44,_45,_46,_47,_48,_49,_50, \
        _51,_52,_53,_54,_55,_56,_57,_58,_59,_60, \
        _61,_62,_63,N,...) N

#define PP_RSEQ_N() \
        63,62,61,60,                   \
        59,58,57,56,55,54,53,52,51,50, \
        49,48,47,46,45,44,43,42,41,40, \
        39,38,37,36,35,34,33,32,31,30, \
        29,28,27,26,25,24,23,22,21,20, \
        19,18,17,16,15,14,13,12,11,10, \
        9,8,7,6,5,4,3,2,1,0

#define  SEGMENTS1(a)                                               (POW2(a))
#define  SEGMENTS2(a, b)                                            (SEGMENTS1(a) | SEGMENTS1(b))
#define  SEGMENTS3(a, b, c)                                         (SEGMENTS1(a) | SEGMENTS2(b, c))
#define  SEGMENTS4(a, b, c, d)                                      (SEGMENTS1(a) | SEGMENTS3(b, c, d))
#define  SEGMENTS5(a, b, c, d, e)                                   (SEGMENTS1(a) | SEGMENTS4(b, c, d, e))
#define  SEGMENTS6(a, b, c, d, e, f)                                (SEGMENTS1(a) | SEGMENTS5(b, c, d, e, f))
#define  SEGMENTS7(a, b, c, d, e, f, g)                             (SEGMENTS1(a) | SEGMENTS6(b, c, d, e, f, g))
#define  SEGMENTS8(a, b, c, d, e, f, g, h)                          (SEGMENTS1(a) | SEGMENTS7(b, c, d, e, f, g, h))
#define  SEGMENTS9(a, b, c, d, e, f, g, h, i)                       (SEGMENTS1(a) | SEGMENTS8(b, c, d, e, f, g, h, i))
#define SEGMENTS10(a, b, c, d, e, f, g, h, i, j)                    (SEGMENTS1(a) | SEGMENTS9(b, c, d, e, f, g, h, i, j))
#define SEGMENTS11(a, b, c, d, e, f, g, h, i, j, k)                 (SEGMENTS1(a) | SEGMENTS10(b, c, d, e, f, g, h, i, j, k))
#define SEGMENTS12(a, b, c, d, e, f, g, h, i, j, k, l)              (SEGMENTS1(a) | SEGMENTS11(b, c, d, e, f, g, h, i, j, k, l))
#define SEGMENTS13(a, b, c, d, e, f, g, h, i, j, k, l, m)           (SEGMENTS1(a) | SEGMENTS12(b, c, d, e, f, g, h, i, j, k, l, m))
#define SEGMENTS14(a, b, c, d, e, f, g, h, i, j, k, l, m, n)        (SEGMENTS1(a) | SEGMENTS13(b, c, d, e, f, g, h, i, j, k, l, m, n))
#define SEGMENTS15(a, b, c, d, e, f, g, h, i, j, k, l, m, n, o)     (SEGMENTS1(a) | SEGMENTS14(b, c, d, e, f, g, h, i, j, k, l, m, n, o))
#define SEGMENTS16(a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p)  (SEGMENTS1(a) | SEGMENTS15(b, c, d, e, f, g, h, i, j, k, l, m, n, o, p))

#define SEGMENTS_(A, ...)    A(__VA_ARGS__)
#define SEGMENTS(...)        SEGMENTS_(XPASTE(SEGMENTS, PP_NARG(__VA_ARGS__)), __VA_ARGS__)


/*
 * Character
 */
typedef enum {

    CH_BLANK = 0,

    CH_A = SEGMENTS(G_SEGMENT, H_SEGMENT, A_SEGMENT, B_SEGMENT, C_SEGMENT, D_SEGMENT, U_SEGMENT, P_SEGMENT),
    CH_B = SEGMENTS(A_SEGMENT, B_SEGMENT, C_SEGMENT, D_SEGMENT, E_SEGMENT, F_SEGMENT, M_SEGMENT, S_SEGMENT, P_SEGMENT),
    CH_C = SEGMENTS(B_SEGMENT, A_SEGMENT, H_SEGMENT, G_SEGMENT, F_SEGMENT, E_SEGMENT),
    CH_D = SEGMENTS(A_SEGMENT, B_SEGMENT, C_SEGMENT, D_SEGMENT, E_SEGMENT, F_SEGMENT, M_SEGMENT, S_SEGMENT),
    CH_E = SEGMENTS(B_SEGMENT, A_SEGMENT, H_SEGMENT, G_SEGMENT, F_SEGMENT, E_SEGMENT, U_SEGMENT, P_SEGMENT),
    CH_F = SEGMENTS(B_SEGMENT, A_SEGMENT, H_SEGMENT, G_SEGMENT, U_SEGMENT, P_SEGMENT),
    CH_G = SEGMENTS(A_SEGMENT, B_SEGMENT, H_SEGMENT, G_SEGMENT, F_SEGMENT, E_SEGMENT, D_SEGMENT, P_SEGMENT),
    CH_H = SEGMENTS(H_SEGMENT, G_SEGMENT, D_SEGMENT, C_SEGMENT, U_SEGMENT, P_SEGMENT),
    CH_I = SEGMENTS(A_SEGMENT, B_SEGMENT, F_SEGMENT, E_SEGMENT, M_SEGMENT, S_SEGMENT),
    CH_J = SEGMENTS(C_SEGMENT, D_SEGMENT, E_SEGMENT, F_SEGMENT, G_SEGMENT),
    CH_K = SEGMENTS(H_SEGMENT, G_SEGMENT, U_SEGMENT, N_SEGMENT, R_SEGMENT),
    CH_L = SEGMENTS(H_SEGMENT, G_SEGMENT, F_SEGMENT, E_SEGMENT),
    CH_M = SEGMENTS(G_SEGMENT, H_SEGMENT, K_SEGMENT, N_SEGMENT, C_SEGMENT, D_SEGMENT),
    CH_N = SEGMENTS(G_SEGMENT, H_SEGMENT, K_SEGMENT, R_SEGMENT, D_SEGMENT, C_SEGMENT),
    CH_O = SEGMENTS(B_SEGMENT, A_SEGMENT, H_SEGMENT, G_SEGMENT, F_SEGMENT, E_SEGMENT, C_SEGMENT, D_SEGMENT),
    CH_P = SEGMENTS(G_SEGMENT, H_SEGMENT, A_SEGMENT, B_SEGMENT, C_SEGMENT, U_SEGMENT, P_SEGMENT),
    CH_Q = SEGMENTS(B_SEGMENT, A_SEGMENT, H_SEGMENT, G_SEGMENT, F_SEGMENT, E_SEGMENT, C_SEGMENT, D_SEGMENT, R_SEGMENT),
    CH_R = SEGMENTS(G_SEGMENT, H_SEGMENT, A_SEGMENT, B_SEGMENT, C_SEGMENT, U_SEGMENT, P_SEGMENT, R_SEGMENT),
    CH_S = SEGMENTS(B_SEGMENT, A_SEGMENT, H_SEGMENT, U_SEGMENT, P_SEGMENT, D_SEGMENT, E_SEGMENT, F_SEGMENT),
    CH_T = SEGMENTS(A_SEGMENT, B_SEGMENT, M_SEGMENT, S_SEGMENT),
    CH_U = SEGMENTS(H_SEGMENT, G_SEGMENT, F_SEGMENT, E_SEGMENT, C_SEGMENT, D_SEGMENT),
    CH_V = SEGMENTS(H_SEGMENT, G_SEGMENT, T_SEGMENT, N_SEGMENT),
    CH_W = SEGMENTS(H_SEGMENT, G_SEGMENT, T_SEGMENT, R_SEGMENT, C_SEGMENT, D_SEGMENT),
    CH_X = SEGMENTS(K_SEGMENT, N_SEGMENT, T_SEGMENT, R_SEGMENT),
    CH_Y = SEGMENTS(H_SEGMENT, U_SEGMENT, P_SEGMENT, F_SEGMENT, E_SEGMENT, D_SEGMENT, C_SEGMENT),
    CH_Z = SEGMENTS(A_SEGMENT, B_SEGMENT, N_SEGMENT, T_SEGMENT, F_SEGMENT, E_SEGMENT),

	CH_a = SEGMENTS(G_SEGMENT, U_SEGMENT, S_SEGMENT, F_SEGMENT, E_SEGMENT),
	CH_b = SEGMENTS(G_SEGMENT, U_SEGMENT, S_SEGMENT, F_SEGMENT, H_SEGMENT),
	CH_c = SEGMENTS(U_SEGMENT, G_SEGMENT, F_SEGMENT),
	CH_d = SEGMENTS(S_SEGMENT, P_SEGMENT, E_SEGMENT, D_SEGMENT, C_SEGMENT),
	CH_e = SEGMENTS(U_SEGMENT, G_SEGMENT, T_SEGMENT, F_SEGMENT),
	CH_f = SEGMENTS(U_SEGMENT, P_SEGMENT, M_SEGMENT, S_SEGMENT, B_SEGMENT),
	CH_g = SEGMENTS(H_SEGMENT, A_SEGMENT, M_SEGMENT, U_SEGMENT, S_SEGMENT, F_SEGMENT),
	CH_h = SEGMENTS(H_SEGMENT, G_SEGMENT, U_SEGMENT, S_SEGMENT),
	CH_i = SEGMENTS(S_SEGMENT),
	CH_j = SEGMENTS(M_SEGMENT, S_SEGMENT, F_SEGMENT, G_SEGMENT),
	CH_k = SEGMENTS(M_SEGMENT, S_SEGMENT, N_SEGMENT, R_SEGMENT),
	CH_l = SEGMENTS(H_SEGMENT, G_SEGMENT),
	CH_m = SEGMENTS(G_SEGMENT, U_SEGMENT, S_SEGMENT, P_SEGMENT, D_SEGMENT),
	CH_n = SEGMENTS(G_SEGMENT, U_SEGMENT, S_SEGMENT),
	CH_o = SEGMENTS(G_SEGMENT, U_SEGMENT, S_SEGMENT, F_SEGMENT),
	CH_p = SEGMENTS(G_SEGMENT, H_SEGMENT, A_SEGMENT, M_SEGMENT, U_SEGMENT),
	CH_q = SEGMENTS(S_SEGMENT, H_SEGMENT, A_SEGMENT, M_SEGMENT, U_SEGMENT),
	CH_r = SEGMENTS(G_SEGMENT, U_SEGMENT),
	CH_s = SEGMENTS(A_SEGMENT, H_SEGMENT, U_SEGMENT, S_SEGMENT, F_SEGMENT),
	CH_t = SEGMENTS(H_SEGMENT, G_SEGMENT, U_SEGMENT, F_SEGMENT),
	CH_u = SEGMENTS(G_SEGMENT, F_SEGMENT, S_SEGMENT),
	CH_v = SEGMENTS(G_SEGMENT, T_SEGMENT),
	CH_w = SEGMENTS(G_SEGMENT, T_SEGMENT, R_SEGMENT, D_SEGMENT),
	CH_x = SEGMENTS(K_SEGMENT, T_SEGMENT, N_SEGMENT, R_SEGMENT),
	CH_y = SEGMENTS(M_SEGMENT, P_SEGMENT, C_SEGMENT, D_SEGMENT, E_SEGMENT),
	CH_z = SEGMENTS(U_SEGMENT, T_SEGMENT, F_SEGMENT),

    CH_0 = SEGMENTS(B_SEGMENT, A_SEGMENT, H_SEGMENT, G_SEGMENT, F_SEGMENT, E_SEGMENT, C_SEGMENT, D_SEGMENT),
    CH_1 = SEGMENTS(N_SEGMENT, C_SEGMENT, D_SEGMENT),
    CH_2 = SEGMENTS(A_SEGMENT, B_SEGMENT, C_SEGMENT, P_SEGMENT, U_SEGMENT, G_SEGMENT, F_SEGMENT, E_SEGMENT),
    CH_3 = SEGMENTS(A_SEGMENT, B_SEGMENT, C_SEGMENT, P_SEGMENT, D_SEGMENT, E_SEGMENT, F_SEGMENT),
    CH_4 = SEGMENTS(H_SEGMENT, U_SEGMENT, P_SEGMENT, C_SEGMENT, D_SEGMENT),
    CH_5 = SEGMENTS(B_SEGMENT, A_SEGMENT, H_SEGMENT, U_SEGMENT, P_SEGMENT, D_SEGMENT, E_SEGMENT, F_SEGMENT),
    CH_6 = SEGMENTS(A_SEGMENT, B_SEGMENT, H_SEGMENT, U_SEGMENT, P_SEGMENT, G_SEGMENT, F_SEGMENT, E_SEGMENT, D_SEGMENT),
    CH_7 = SEGMENTS(A_SEGMENT, B_SEGMENT, C_SEGMENT, D_SEGMENT),
    CH_8 = SEGMENTS(A_SEGMENT, B_SEGMENT, H_SEGMENT, U_SEGMENT, P_SEGMENT, G_SEGMENT, F_SEGMENT, E_SEGMENT, D_SEGMENT, C_SEGMENT),
    CH_9 = SEGMENTS(A_SEGMENT, B_SEGMENT, H_SEGMENT, U_SEGMENT, P_SEGMENT, F_SEGMENT, E_SEGMENT, D_SEGMENT, C_SEGMENT),

	CH_EQ = SEGMENTS(U_SEGMENT, P_SEGMENT, F_SEGMENT, E_SEGMENT),
	CH_NEQ = SEGMENTS(U_SEGMENT, P_SEGMENT, T_SEGMENT, N_SEGMENT),
	CH_PLUS = SEGMENTS(U_SEGMENT, P_SEGMENT, M_SEGMENT, S_SEGMENT),
	CH_MINUS = SEGMENTS(U_SEGMENT, P_SEGMENT),
	CH_DIVISION = SEGMENTS(T_SEGMENT, N_SEGMENT),
	CH_LESS = SEGMENTS(R_SEGMENT, N_SEGMENT),
	CH_MORE = SEGMENTS(K_SEGMENT, T_SEGMENT),
	CH_LESS_OR_EQ = SEGMENTS(R_SEGMENT, N_SEGMENT, U_SEGMENT),
	CH_MORE_OR_EQ = SEGMENTS(K_SEGMENT, T_SEGMENT, P_SEGMENT),

	CH_OPEN_ROUND_BRACKET = SEGMENTS(R_SEGMENT, N_SEGMENT),
	CH_CLOSE_ROUND_BRACKET = SEGMENTS(K_SEGMENT, T_SEGMENT),
	CH_OPEN_SQUARE_BRACKET = SEGMENTS(B_SEGMENT, M_SEGMENT, S_SEGMENT, E_SEGMENT),
	CH_CLOSE_SQUARE_BRACKET = SEGMENTS(A_SEGMENT, M_SEGMENT, S_SEGMENT, F_SEGMENT),
	CH_OPEN_CURLY_BRACKET = SEGMENTS(B_SEGMENT, M_SEGMENT, S_SEGMENT, E_SEGMENT, U_SEGMENT),
	CH_CLOSE_CURLY_BRACKET = SEGMENTS(A_SEGMENT, M_SEGMENT, S_SEGMENT, F_SEGMENT, P_SEGMENT),
	CH_APOSTROPHE = SEGMENTS(M_SEGMENT),
	CH_QUOTE = SEGMENTS(M_SEGMENT, C_SEGMENT),
	CH_ASTERISK = SEGMENTS(K_SEGMENT, M_SEGMENT, N_SEGMENT, P_SEGMENT, R_SEGMENT, S_SEGMENT, T_SEGMENT, U_SEGMENT),
	CH_NUMBER = SEGMENTS(M_SEGMENT, C_SEGMENT, U_SEGMENT, F_SEGMENT, S_SEGMENT, D_SEGMENT, P_SEGMENT, E_SEGMENT),
	CH_DOLLAR = SEGMENTS(B_SEGMENT, A_SEGMENT, H_SEGMENT, U_SEGMENT, P_SEGMENT, D_SEGMENT, E_SEGMENT, F_SEGMENT, M_SEGMENT, S_SEGMENT),
	CH_PERSENT = SEGMENTS(T_SEGMENT, N_SEGMENT, M_SEGMENT, S_SEGMENT, A_SEGMENT, H_SEGMENT, E_SEGMENT, D_SEGMENT, U_SEGMENT, P_SEGMENT),
	CH_AMP = SEGMENTS(K_SEGMENT, A_SEGMENT, M_SEGMENT, R_SEGMENT, E_SEGMENT, U_SEGMENT, G_SEGMENT, F_SEGMENT),
	CH_COMMA = SEGMENTS(T_SEGMENT),
	CH_BROKEN_BAR = SEGMENTS(M_SEGMENT, S_SEGMENT),
	CH_UNDERSCORE = SEGMENTS(F_SEGMENT, E_SEGMENT),

    N_CHARS
} Character;
/* */


/*
 * Dot
 */
typedef enum {
    DOT_OBSCURE = 0,
	DOT_HIGHLIGHT = 1
} Dot;
/* */


/*
 * Display
 */
typedef struct {
    void *buffer;
    size_t size; 				/* number of places */
    size_t buffer_size; 		/* buffer size in bytes */

    size_t symbols_buffer_size; /* symbols chunk size in 16bits */
    size_t dots_buffer_size;	/* dots chunk size in 16bits */
} Display;

void DisplayCreate(Display *display,
                   size_t size)
{
	/* Display buffer structure:
	 *  symbols            dots
	 * |5|4|3|2|1|0| xxxx |0|1|2|3|4|5|
	 * */

	display->size = size;
	display->symbols_buffer_size = size;
	display->dots_buffer_size = (size / (sizeof(uint16_t) * N_BITS_IN_BYTE) + 1);
	display->buffer_size =
			(display->symbols_buffer_size + display->dots_buffer_size) * sizeof(uint16_t);

    display->buffer = malloc(display->buffer_size);

    memset(display->buffer, 0, display->buffer_size);
}

void DisplayDispose(Display *display)
{
    display->size = 0;
    display->symbols_buffer_size = 0;
    display->dots_buffer_size = 0;
    display->buffer_size = 0;

    free(display->buffer);
}

void DisplayWrite(Display *display,
                  Character str[],
                  Dot dot[],
                  size_t size)
{
	size_t dot_buff_size = (size / N_BITS_IN_BYTE) + 1;
	uint8_t *dot_buff = malloc(dot_buff_size);
	size_t k, l;

	/* Dots buffer */
	memset(dot_buff, 0, dot_buff_size);

	for (size_t i = 0, j = size-1;
		 i != size; ++i, --j)
	{
		k = i / 8;
		l = i % 8;

		if (dot[j] == DOT_HIGHLIGHT) {
			dot_buff[k] |= POW2(l);
		}
	}

	memcpy(display->buffer, dot_buff, dot_buff_size);
	free(dot_buff);

	/* Symbol buffer */
	for (size_t i = display->dots_buffer_size;
		 i != size + display->dots_buffer_size;
		 ++i)
	{
		((uint16_t *)display->buffer)[i] = str[i-display->dots_buffer_size];
	}

	/* Sending */
//	HAL_SPI_Transmit(&hspi1, display->buffer, display->buffer_size / sizeof(uint16_t), 1000);
//	HAL_GPIO_TogglePin(LED_DATA_LATCH_GPIO_Port, LED_DATA_LATCH_Pin);
//	HAL_Delay(1);
//	HAL_GPIO_TogglePin(LED_DATA_LATCH_GPIO_Port, LED_DATA_LATCH_Pin);
}

void DisplayWriteWithoutDots(Display *display,
							 Character str[],
							 size_t size)
{
	/* Symbol buffer */
	for (size_t i = display->dots_buffer_size;
		 i != size + display->dots_buffer_size;
		 ++i)
	{
		((uint16_t *)display->buffer)[i] = str[i-display->dots_buffer_size];
	}

	/* Sending */
//	HAL_SPI_Transmit(&hspi1, display->buffer, display->buffer_size / sizeof(uint16_t), 1000);
//	HAL_GPIO_TogglePin(LED_DATA_LATCH_GPIO_Port, LED_DATA_LATCH_Pin);
//	HAL_Delay(1);
//	HAL_GPIO_TogglePin(LED_DATA_LATCH_GPIO_Port, LED_DATA_LATCH_Pin);
}

Character AsciiToCharacter(char c)
{
	Character ch;

	switch (c) {
	case 'A':
		ch = CH_A;
		break;
	case 'B':
		ch = CH_B;
		break;
	case 'C':
		ch = CH_C;
		break;
	case 'D':
		ch = CH_D;
		break;
	case 'E':
		ch = CH_E;
		break;
	case 'F':
		ch = CH_F;
		break;
	case 'G':
		ch = CH_G;
		break;
	case 'H':
		ch = CH_H;
		break;
	case 'I':
		ch = CH_I;
		break;
	case 'J':
		ch = CH_J;
		break;
	case 'K':
		ch = CH_K;
		break;
	case 'L':
		ch = CH_L;
		break;
	case 'M':
		ch = CH_M;
		break;
	case 'N':
		ch = CH_N;
		break;
	case 'O':
		ch = CH_O;
		break;
	case 'P':
		ch = CH_P;
		break;
	case 'Q':
		ch = CH_Q;
		break;
	case 'R':
		ch = CH_R;
		break;
	case 'S':
		ch = CH_S;
		break;
	case 'T':
		ch = CH_T;
		break;
	case 'U':
		ch = CH_U;
		break;
	case 'V':
		ch = CH_V;
		break;
	case 'W':
		ch = CH_W;
		break;
	case 'X':
		ch = CH_X;
		break;
	case 'Y':
		ch = CH_Y;
		break;
	case 'Z':
		ch = CH_Z;
		break;

	case 'a':
		ch = CH_a;
		break;
	case 'b':
		ch = CH_b;
		break;
	case 'c':
		ch = CH_c;
		break;
	case 'd':
		ch = CH_d;
		break;
	case 'e':
		ch = CH_e;
		break;
	case 'f':
		ch = CH_f;
		break;
	case 'g':
		ch = CH_g;
		break;
	case 'h':
		ch = CH_h;
		break;
	case 'i':
		ch = CH_i;
		break;
	case 'j':
		ch = CH_j;
		break;
	case 'k':
		ch = CH_k;
		break;
	case 'l':
		ch = CH_l;
		break;
	case 'm':
		ch = CH_m;
		break;
	case 'n':
		ch = CH_n;
		break;
	case 'o':
		ch = CH_o;
		break;
	case 'p':
		ch = CH_p;
		break;
	case 'q':
		ch = CH_q;
		break;
	case 'r':
		ch = CH_r;
		break;
	case 's':
		ch = CH_s;
		break;
	case 't':
		ch = CH_t;
		break;
	case 'u':
		ch = CH_u;
		break;
	case 'v':
		ch = CH_v;
		break;
	case 'w':
		ch = CH_w;
		break;
	case 'x':
		ch = CH_x;
		break;
	case 'y':
		ch = CH_y;
		break;
	case 'z':
		ch = CH_z;
		break;

	case '0':
		ch = CH_0;
		break;
	case '1':
		ch = CH_1;
		break;
	case '2':
		ch = CH_2;
		break;
	case '3':
		ch = CH_3;
		break;
	case '4':
		ch = CH_4;
		break;
	case '5':
		ch = CH_5;
		break;
	case '6':
		ch = CH_6;
		break;
	case '7':
		ch = CH_7;
		break;
	case '8':
		ch = CH_8;
		break;
	case '9':
		ch = CH_9;
		break;

	case ' ':
		ch = CH_BLANK;
		break;
	case '=':
		ch = CH_EQ;
		break;
	case '+':
		ch = CH_PLUS;
		break;
	case '-':
		ch = CH_MINUS;
		break;
	case '/':
		ch = CH_DIVISION;
		break;
	case '<':
		ch = CH_LESS;
		break;
	case '>':
		ch = CH_MORE;
		break;

	case '(':
		ch = CH_OPEN_ROUND_BRACKET;
		break;
	case ')':
		ch = CH_CLOSE_ROUND_BRACKET;
		break;
	case '[':
		ch = CH_OPEN_SQUARE_BRACKET;
		break;
	case ']':
		ch = CH_CLOSE_SQUARE_BRACKET;
		break;
	case '{':
		ch = CH_OPEN_CURLY_BRACKET;
		break;
	case '}':
		ch = CH_CLOSE_CURLY_BRACKET;
		break;

	case '\'':
		ch = CH_APOSTROPHE;
		break;
	case '"':
		ch = CH_QUOTE;
		break;
	case '#':
		ch = CH_NUMBER;
		break;
	case '$':
		ch = CH_DOLLAR;
		break;
	case '%':
		ch = CH_PERSENT;
		break;
	case '&':
		ch = CH_AMP;
		break;
	case ',':
	case '.':
		ch = CH_COMMA;
		break;
	case '|':
		ch = CH_BROKEN_BAR;
		break;
	case '_':
		ch = CH_UNDERSCORE;
		break;

	default:
		ch = CH_ASTERISK;
		break;
	}

	return ch;
}

void DisplayWriteStr(Display *display,
					 const char *str,
					 size_t n)
{
	Character c[n];

	for (size_t i = 0, j = n-1; i != n; ++i, --j) {
		c[j] = AsciiToCharacter(str[i]);
	}

	DisplayWriteWithoutDots(display, c, n);
}

void DisplayWriteUint(Display *display,
					  uint32_t value)
{
	Character c[display->size];

	memset(c, CH_BLANK, sizeof(c));

	if (value) {
		uint8_t d;
		size_t i = 0;

		while (value) {
			d = value % 10;
			c[i] = AsciiToCharacter(d + '0');
			value /= 10;
			++i;

		}
	} else {
		c[0] = CH_0;
	}

	DisplayWriteWithoutDots(display, c, display->size);
}
/* */


/*
 * Display controller
 */
typedef struct {
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *port;
	uint16_t pin;
	uint32_t transmit_timeout;

} DisplayController;

void DisplayControllerCreate(DisplayController *controller,
							 SPI_HandleTypeDef *hspi,
							 GPIO_TypeDef *port,
							 uint16_t pin)
{
	controller->hspi = hspi;
	controller->port = port;
	controller->pin = pin;
	controller->transmit_timeout = 1000;
}

void DisplayControllerDispose(DisplayController *controller)
{

}

HAL_StatusTypeDef DisplayControllerSend(DisplayController *controller,
										Display *display)
{
	HAL_StatusTypeDef ret;

	/* Sending */
	ret = HAL_SPI_Transmit(&hspi1,
						   display->buffer,
						   display->buffer_size / sizeof(uint16_t),
						   controller->transmit_timeout);
	HAL_GPIO_TogglePin(controller->port, controller->pin);
	HAL_Delay(1);
	HAL_GPIO_TogglePin(controller->port, controller->pin);

	return ret;
}
/* */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	Display display;
	DisplayController controller;
	size_t n_places = 6;
	Character c[6];
	Dot d[6];

	c[0] = CH_Y;
	c[1] = CH_D;
	c[2] = CH_U;
	c[3] = CH_O;
	c[4] = CH_L;
	c[5] = CH_C;

	d[0] = DOT_OBSCURE;
	d[1] = DOT_OBSCURE;
	d[2] = DOT_HIGHLIGHT;
	d[3] = DOT_OBSCURE;
	d[4] = DOT_HIGHLIGHT;
	d[5] = DOT_OBSCURE;

	DisplayCreate(&display, n_places);
	DisplayControllerCreate(&controller, &hspi1, LED_DATA_LATCH_GPIO_Port, LED_DATA_LATCH_Pin);

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
  /* USER CODE BEGIN 2 */

  //HAL_TIM_Base_Start(&htim5);
  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);

  uint8_t buf[20];
  buf[0] = 0xD0;
  HAL_GPIO_WritePin(BMP280_NSS_GPIO_Port, BMP280_NSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi3, buf, 1, 100);
  HAL_SPI_Receive(&hspi3, buf, 1, 100);
  HAL_GPIO_WritePin(BMP280_NSS_GPIO_Port, BMP280_NSS_Pin, GPIO_PIN_SET);

  buf[0] = 0x00;
  HAL_I2C_Master_Transmit(&hi2c1, 0xD0, buf, 1, 100);
  HAL_I2C_Master_Receive(&hi2c1, 0xD0, buf, 19, 100);
  buf[1] = buf[0];

  ///
  DisplayWrite(&display, c, d, n_places);
  DisplayControllerSend(&controller, &display);
  DisplayWriteStr(&display, "CLOUDY", n_places);
  DisplayControllerSend(&controller, &display);
//  uint16_t aa_dma[7];
	//memset(aa_dma, 0xFF, 14);
//  memset(aa_dma, 0b1, 14);
//  aa_dma[0] = 0;
//  aa_dma[1] = CH_A;
//  aa_dma[2] = CH_B;
//  aa_dma[3] = CH_C;
//  aa_dma[4] = CH_D;
//  aa_dma[5] = CH_E;
//  aa_dma[6] = CH_F;


//	HAL_SPI_Transmit(&hspi1, aa_dma, 7, 1000);
//	HAL_GPIO_TogglePin(LED_DATA_LATCH_GPIO_Port, LED_DATA_LATCH_Pin);
//	HAL_Delay(1);
//	HAL_GPIO_TogglePin(LED_DATA_LATCH_GPIO_Port, LED_DATA_LATCH_Pin);

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  //osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  DisplayWriteStr(&display, "CLOUDY", n_places);
	  DisplayControllerSend(&controller, &display);
	  HAL_Delay(1000);
	  DisplayWriteStr(&display, "DOLLAR", n_places);
	  DisplayControllerSend(&controller, &display);
	  HAL_Delay(1000);
	  DisplayWriteUint(&display, 0);
	  DisplayControllerSend(&controller, &display);
	  HAL_Delay(1000);
	  DisplayWriteUint(&display, 123456);
	  DisplayControllerSend(&controller, &display);
	  HAL_Delay(1000);
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
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
