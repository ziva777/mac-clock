/*
 * flash.c
 *
 *  Created on: 16 окт. 2018 г.
 *      Author: ziva
 */

#include "flash.h"

void FlashErase()
{
	HAL_StatusTypeDef flash_ok;

	flash_ok = HAL_ERROR;
	while(flash_ok != HAL_OK) {
		flash_ok = HAL_FLASH_Unlock();
	}

	FLASH_Erase_Sector(FLASH_SECTOR_11, VOLTAGE_RANGE_3);

	flash_ok = HAL_ERROR;
	while(flash_ok != HAL_OK) {
		flash_ok = HAL_FLASH_Lock();
	}
}

void FlashWrite(int tz_idx,
				double latitude,
				double longitude,
				double p_correction)
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
			flash_ok = HAL_FLASH_Program(TYPEPROGRAM_WORD,
										 LONGITUDE_ADDR,
										 cast.u32[0]);
		}

		flash_ok = HAL_ERROR;
		while (flash_ok != HAL_OK) {
			flash_ok = HAL_FLASH_Program(TYPEPROGRAM_WORD,
										 LONGITUDE_ADDR + sizeof(int),
										 cast.u32[1]);
		}
		/* ~ long */

		/* p_correction */
		cast.d = p_correction;
		flash_ok = HAL_ERROR;
		while (flash_ok != HAL_OK) {
			flash_ok = HAL_FLASH_Program(TYPEPROGRAM_WORD,
										 PRESSURE_CORRECTION_ADDR,
										 cast.u32[0]);
		}

		flash_ok = HAL_ERROR;
		while (flash_ok != HAL_OK) {
			flash_ok = HAL_FLASH_Program(TYPEPROGRAM_WORD,
										 PRESSURE_CORRECTION_ADDR + sizeof(int),
										 cast.u32[1]);
		}
		/* ~p_correction */

		flash_ok = HAL_ERROR;
		while (flash_ok != HAL_OK) {
			flash_ok = HAL_FLASH_Lock();
		}
	}
}

int FlashReadTZ()
{
	int32_t idx = *(volatile int *)(TZ_IDX_ADDR);
	return idx;
}

double FlashReadLatitude()
{
	double v = *(volatile double *)(LATITUDE_ADDR);
	return v;
}

double FlashReadLongitude()
{
	double v = *(volatile double *)(LONGITUDE_ADDR);
	return v;
}

double	FlashReadPCorrection()
{
	double v = *(volatile double *)(PRESSURE_CORRECTION_ADDR);
	return v;
}
