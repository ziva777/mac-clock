/*
 * rtc_ds3221.h
 *
 *  Created on: 27 сент. 2018 г.
 *      Author: ziva
 */

#ifndef INC_RTC_DS3221_H_
#define INC_RTC_DS3221_H_

#include <stddef.h>
#include <stdint.h>
#include <time.h>

#define SECONDS_FROM_1970_TO_2000 946684800

// i2c slave address of the DS3231 chip
#define DS3231_I2C_ADDR             0xD0 /* 0x68 << 1*/

// timekeeping registers
#define DS3231_TIME_CAL_ADDR        0x00
#define DS3231_ALARM1_ADDR          0x07
#define DS3231_ALARM2_ADDR          0x0B
#define DS3231_CONTROL_ADDR         0x0E
#define DS3231_STATUS_ADDR          0x0F
#define DS3231_AGING_OFFSET_ADDR    0x10
#define DS3231_TEMPERATURE_ADDR     0x11

// control register bits
#define DS3231_A1IE     0x1
#define DS3231_A2IE     0x2
#define DS3231_INTCN    0x4

// status register bits
#define DS3231_A1F      0x1
#define DS3231_A2F      0x2
#define DS3231_OSF      0x80


struct ts {
    uint8_t sec; /* seconds */
    uint8_t min; /* minutes */
    uint8_t hour; /* hours */
    uint8_t mday; /* day of the month */
    uint8_t mon; /* month */
    int16_t year; /* year */
    uint8_t wday; /* day of the week */
    uint8_t yday; /* day in the year */
    uint8_t isdst; /* daylight saving time */
    uint8_t year_s; /* year in short notation*/
#ifdef CONFIG_UNIXTIME
uint32_t unixtime; /* seconds since 01.01.1970 00:00:00 UTC*/
#endif
};

typedef struct ts Rtc_Timestamp;

void Rtc_DS3231_init(const uint8_t creg);
void Rtc_DS3231_set(struct ts t);
void Rtc_DS3231_get(struct ts *t);

void Rtc_DS3231_set_addr(const uint8_t addr,
                         const uint8_t val);
uint8_t Rtc_DS3231_get_addr(const uint8_t addr);

// control/status register
void Rtc_DS3231_set_creg(const uint8_t val);
void Rtc_DS3231_set_sreg(const uint8_t val);
uint8_t Rtc_DS3231_get_sreg(void);

// aging offset register
void Rtc_DS3231_set_aging(const int8_t val);
int8_t Rtc_DS3231_get_aging(void);

// temperature register
float Rtc_DS3231_get_treg(void);

// alarms
void Rtc_DS3231_set_a1(const uint8_t s,
                       const uint8_t mi,
                       const uint8_t h,
                       const uint8_t d,
                       const uint8_t * flags);
void Rtc_DS3231_get_a1(char *buf,
                       const uint8_t len);
void Rtc_DS3231_clear_a1f(void);
uint8_t Rtc_DS3231_triggered_a1(void);

void Rtc_DS3231_set_a2(const uint8_t mi,
                       const uint8_t h,
                       const uint8_t d,
                       const uint8_t * flags);
void Rtc_DS3231_get_a2(char *buf,
                       const uint8_t len);
void Rtc_DS3231_clear_a2f(void);
uint8_t Rtc_DS3231_triggered_a2(void);

Rtc_Timestamp GetTimestampFromTm(struct tm tm);

#endif /* INC_RTC_DS3221_H_ */
