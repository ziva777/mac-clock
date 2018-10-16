/*
 * display_aux.c
 *
 *  Created on: 16 окт. 2018 г.
 *      Author: ziva
 */

#include "display_aux.h"

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