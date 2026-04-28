/*
 * onewire.c
 *
 *  Created on: Apr 24, 2026
 *      Author: frrig
 */


#include "onewire.h"

// Inizializza il contatore DWT per i microsecondi
void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// Ritardo preciso in microsecondi
void delay_us(uint32_t us) {
    uint32_t startTick = DWT->CYCCNT;
    uint32_t delayTicks = us * (SystemCoreClock / 1000000);
    while (DWT->CYCCNT - startTick < delayTicks);
}

// Reset del bus e Presence Pulse
uint8_t OneWire_Reset(void) {
    uint8_t presence = 0;
    // Porta il bus basso per 480us
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
    delay_us(480);
    // Rilascia il bus
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
    delay_us(70);
    // Se il sensore risponde, porta il bus basso
    if (!(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2))) presence = 1;
    delay_us(410);
    return presence;
}

// Scrittura di un singolo bit
void OneWire_WriteBit(uint8_t bit) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
    delay_us(bit ? 10 : 65);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
    delay_us(bit ? 55 : 5);
}

// Lettura di un singolo bit
uint8_t OneWire_ReadBit(void) {
    uint8_t bit = 0;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
    delay_us(3);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
    delay_us(10);
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2)) bit = 1;
    delay_us(50);
    return bit;
}

// Scrittura di un byte
void OneWire_WriteByte(uint8_t byte) {
    for (uint8_t i = 0; i < 8; i++) {
        OneWire_WriteBit(byte & (1 << i));
    }
}

// Lettura di un byte
uint8_t OneWire_ReadByte(void) {
    uint8_t byte = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (OneWire_ReadBit()) byte |= (1 << i);
    }
    return byte;
}

// Funzione finale per la temperatura del suolo
int16_t Read_Soil_Temp_Scaled(void) {
    uint8_t data[2];
    int16_t raw;

    if (!OneWire_Reset()) return -9999; // Errore sensore

    OneWire_WriteByte(DS18B20_CMD_SKIPROM);
    OneWire_WriteByte(DS18B20_CMD_CONVERTT);

    // Aspetta la conversione (750ms per 12-bit)
    HAL_Delay(750);

    OneWire_Reset();
    OneWire_WriteByte(DS18B20_CMD_SKIPROM);
    OneWire_WriteByte(DS18B20_CMD_READSCRATCHPAD);

    data[0] = OneWire_ReadByte(); // LSB
    data[1] = OneWire_ReadByte(); // MSB

    raw = (int16_t)((data[1] << 8) | data[0]);

    // Conversione da sedicesimi di grado a gradi * 100
    // (raw / 16.0) * 100.0 => raw * 6.25
    return (int16_t)(raw * 6.25f);
}
