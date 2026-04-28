/*
 * sd_logger.h
 *
 *  Created on: Feb 25, 2026
 *      Author: kamae
 */

#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include "main.h"
#include "ff.h" // Библиотека FatFS

// Универсальная функция для записи любой строки в любой файл
FRESULT SD_Log_String(const char* filename, const char* data_string);

#endif /* SD_LOGGER_H */
