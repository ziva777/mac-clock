/*
 * character.h
 *
 *  Created on: 27 сент. 2018 г.
 *      Author: ziva
 */

#ifndef INC_CHARACTER_H_
#define INC_CHARACTER_H_

/* defines */
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

	CH_OPEN_ROUND_BRACKET =
			SEGMENTS(R_SEGMENT, N_SEGMENT),
	CH_CLOSE_ROUND_BRACKET =
			SEGMENTS(K_SEGMENT, T_SEGMENT),
	CH_OPEN_SQUARE_BRACKET =
			SEGMENTS(B_SEGMENT, M_SEGMENT, S_SEGMENT, E_SEGMENT),
	CH_CLOSE_SQUARE_BRACKET =
			SEGMENTS(A_SEGMENT, M_SEGMENT, S_SEGMENT, F_SEGMENT),
	CH_OPEN_CURLY_BRACKET =
			SEGMENTS(B_SEGMENT, M_SEGMENT, S_SEGMENT, E_SEGMENT, U_SEGMENT),
	CH_CLOSE_CURLY_BRACKET =
			SEGMENTS(A_SEGMENT, M_SEGMENT, S_SEGMENT, F_SEGMENT, P_SEGMENT),
	CH_APOSTROPHE =
			SEGMENTS(M_SEGMENT),
	CH_QUOTE =
			SEGMENTS(M_SEGMENT, C_SEGMENT),
	CH_ASTERISK =
			SEGMENTS(K_SEGMENT, M_SEGMENT, N_SEGMENT, P_SEGMENT, R_SEGMENT, S_SEGMENT, T_SEGMENT, U_SEGMENT),
	CH_NUMBER =
			SEGMENTS(M_SEGMENT, C_SEGMENT, U_SEGMENT, F_SEGMENT, S_SEGMENT, D_SEGMENT, P_SEGMENT, E_SEGMENT),
	CH_DOLLAR =
			SEGMENTS(B_SEGMENT, A_SEGMENT, H_SEGMENT, U_SEGMENT, P_SEGMENT, D_SEGMENT, E_SEGMENT, F_SEGMENT, M_SEGMENT, S_SEGMENT),
	CH_PERSENT =
			SEGMENTS(T_SEGMENT, N_SEGMENT, M_SEGMENT, S_SEGMENT, A_SEGMENT, H_SEGMENT, E_SEGMENT, D_SEGMENT, U_SEGMENT, P_SEGMENT),
	CH_AMP =
			SEGMENTS(K_SEGMENT, A_SEGMENT, M_SEGMENT, R_SEGMENT, E_SEGMENT, U_SEGMENT, G_SEGMENT, F_SEGMENT),
	CH_COMMA =
			SEGMENTS(T_SEGMENT),
	CH_BROKEN_BAR =
			SEGMENTS(M_SEGMENT, S_SEGMENT),
	CH_UNDERSCORE =
			SEGMENTS(F_SEGMENT, E_SEGMENT),

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

static inline Character AsciiToCharacter(char c)
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

#endif /* INC_CHARACTER_H_ */
