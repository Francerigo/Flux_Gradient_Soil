/*
 * fatfs_sd.h
 */

#ifndef __FATFS_SD_H
#define __FATFS_SD_H

#include "stm32wlxx_hal.h"
#include "diskio.h"
#include "main.h"

/* --- НАСТРОЙКИ СВЯЗИ --- */
extern SPI_HandleTypeDef hspi1;
#define HSPI_SDCARD hspi1

/* Пин Chip Select (CS) */
#define CS_SD_PORT CS_SD_GPIO_Port
#define CS_SD_PIN  CS_SD_Pin

#define CS_SD_LOW()     HAL_GPIO_WritePin(CS_SD_PORT, CS_SD_Pin, GPIO_PIN_RESET)
#define CS_SD_HIGH()    HAL_GPIO_WritePin(CS_SD_PORT, CS_SD_Pin, GPIO_PIN_SET)
/* ------------------------ */

DSTATUS SD_disk_initialize (BYTE pdrv);
DSTATUS SD_disk_status (BYTE pdrv);
DRESULT SD_disk_read (BYTE pdrv, BYTE* buff, DWORD sector, UINT count);
DRESULT SD_disk_write (BYTE pdrv, const BYTE* buff, DWORD sector, UINT count);
DRESULT SD_disk_ioctl (BYTE pdrv, BYTE cmd, void* buff);

#endif
