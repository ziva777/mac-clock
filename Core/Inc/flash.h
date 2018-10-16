/*
 * flash.h
 *
 *  Created on: 16 окт. 2018 г.
 *      Author: ziva
 */

#ifndef INC_FLASH_H_
#define INC_FLASH_H_

#include "stm32f4xx_hal.h"

//static const uint32_t tz_idx_addr = 0x080FA000;
//#define TZ_IDX_ADDR 	(0x080FFC00/*+0x80+0x40+0x20+0x0*/)
//#define TZ_IDX_ADDR 	(0x080FFF80/*+0x80+0x40+0x20+0x0*/)
//#define TZ_IDX_ADDR 	(0x080FFFF0-16)
//#define TZ_IDX_ADDR 	(0x080E0000)
#define TZ_IDX_ADDR 	(0x080FFF70)
#define LATITUDE_ADDR 	(0x080FFF80)
#define LONGITUDE_ADDR 	(0x080FFF90)

void 	FlashErase();
void 	FlashWrite(int tz_idx, double latitude, double longitude);
int 	FlashReadTZ();
double 	FlashReadLatitude();
double 	FlashReadLongitude();

#endif /* INC_FLASH_H_ */
