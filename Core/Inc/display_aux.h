/*
 * display_aux.h
 *
 *  Created on: 16 окт. 2018 г.
 *      Author: ziva
 */

#ifndef INC_DISPLAY_AUX_H_
#define INC_DISPLAY_AUX_H_

#include "display.h"
#include "rtc_ds3221.h"

/* Отобразить сообщение */
void Display_Msg(Display *display,
                 Character msg[],
                 size_t n_places);

/* Отобразить восход солнца чч.мм */
void Display_S1_1(Display *display,
                  Rtc_Timestamp *ts);

/* Отобразить восход солнца (сообщение) */
void Display_S1_1_Msg(Display *display);

/* Отобразить заход солнца (сообщение) */
void Display_S2_1_Msg(Display *display);

/* Отобразить давление */
void Display_P1(Display *display,
                uint16_t pressure);

/* Отобразить чч.мм.сс */
void Display_A1_1(Display *display,
                  Rtc_Timestamp *ts);

/* Отобразить чч.мм с мигающим индикатором */
void Display_A1_2(Display *display,
                  Rtc_Timestamp *ts);

/* Отобразить дд.ММ.гг */
void Display_A1_3(Display *display,
                  Rtc_Timestamp *ts);

/* Редактировать время в формате чч.мм.сс */
void Display_EditTime1(Display *display,
                       Rtc_Timestamp *ts);

/* Редактировать время в формате чч.мм */
void Display_EditTime2(Display *display,
                       Rtc_Timestamp *ts);

/* Отобразить параметр точной настройки */
void Display_EditAging(Display *display,
                       int8_t aging);

/* Редактирование широты (-90.00 ... +90.00) */
void Display_EditLatitude(Display *display,
                          double latitude);

/* Редактирование долготы (-180.00 ... +180.00) */
void Display_EditLongitude(Display *display,
                           double longitude);

/* Редактирование временной зоны */
void Display_EditTimezone(Display *display,
                          double tz);

/* Редактировать дату */
void Display_EditDate(Display *display,
                      Rtc_Timestamp *ts);

#endif /* INC_DISPLAY_AUX_H_ */
