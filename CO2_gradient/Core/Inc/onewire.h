/*
 * onewire.h
 *
 *  Created on: Apr 24, 2026
 *      Author: frrig
 */

#ifndef __ONEWIRE_H
#define __ONEWIRE_H

#include "main.h"

// Comandi DS18B20
#define DS18B20_CMD_SKIPROM          0xCC
#define DS18B20_CMD_CONVERTT         0x44
#define DS18B20_CMD_READSCRATCHPAD   0xBE

// Funzioni di sistema e delay
void DWT_Init(void);
void delay_us(uint32_t us);

// Funzioni core 1-Wire
uint8_t OneWire_Reset(void);
void OneWire_WriteByte(uint8_t byte);
uint8_t OneWire_ReadByte(void);

// Funzione specifica per temperatura (già scalata * 100)
int16_t Read_Soil_Temp_Scaled(void);

#endif /* __ONEWIRE_H */
