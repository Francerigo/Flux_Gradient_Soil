/*
 * onewire.c
 *
 * Created on: Apr 24, 2026
 * Author: frrig
 */

#include "onewire.h"
#include "usart.h"
#include "timer_if.h"
#include "stdio.h"
#include "string.h"

// Macro rapide per cambiare modalità al pin PB2 (Port B, Pin 2)
// PB2 usa i bit 4 e 5 del registro MODER (pin * 2)
#define SET_PIN_OUTPUT() { GPIOB->MODER &= ~(3UL << 4); GPIOB->MODER |= (1UL << 4); }
#define SET_PIN_INPUT()  { GPIOB->MODER &= ~(3UL << 4); }

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

// Reset del bus e Presence Pulse (Tutorial Logic)
uint8_t OneWire_Reset(void) {
    uint8_t response = 0;

    SET_PIN_OUTPUT();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); // Pull Low
    delay_us(480);

    SET_PIN_INPUT(); // Release bus and set as Input
    delay_us(80);    // Wait for sensor to pull it low (Presence)

    if (!(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2))) {
        response = 1; // Presence detected
    }

    delay_us(400); // Complete the 480us total slot
    return response;
}

// Scrittura di un singolo bit (Tutorial Logic)
void OneWire_WriteBit(uint8_t bit) {
    if (bit) {
        // Write 1
        SET_PIN_OUTPUT();
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); // Pull Low
        delay_us(1); // Wait only 1us
        SET_PIN_INPUT(); // Release immediately
        delay_us(60);
    } else {
        // Write 0
        SET_PIN_OUTPUT();
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); // Pull Low
        delay_us(60); // Hold low for 60us
        SET_PIN_INPUT(); // Release
        delay_us(1);
    }
}

// Lettura di un singolo bit (Tutorial Logic)
uint8_t OneWire_ReadBit(void) {
    uint8_t bit = 0;

    SET_PIN_OUTPUT();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); // Pull Low
    delay_us(2); // Wait 2us

    SET_PIN_INPUT(); // Set as input to read

    // Piccolo delay di 2-5us per permettere alla pull-up di vincere
    // la capacità del MUX se il bit è 1
    delay_us(5);

    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2)) {
        bit = 1;
    }

    delay_us(55); // Complete the 60us slot
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
        if (OneWire_ReadBit()) {
            byte |= (1 << i);
        }
    }
    return byte;
}

int16_t Read_Soil_Temp_Scaled(void) {
    int16_t temperature = 0;
    uint8_t low, high;
    char uartBuf[128];
    uint16_t mSeconds;
    uint32_t totalSeconds = TIMER_IF_GetTime(&mSeconds);

    // 1. Inizio Conversione
    if (OneWire_Reset()) {
        OneWire_WriteByte(DS18B20_CMD_SKIPROM);
        OneWire_WriteByte(DS18B20_CMD_CONVERTT);

        // Attendere la conversione (750ms per 12-bit)
        HAL_Delay(750);

        // 2. Lettura dello Scratchpad
        if (OneWire_Reset()) {
            OneWire_WriteByte(DS18B20_CMD_SKIPROM);
            OneWire_WriteByte(DS18B20_CMD_READSCRATCHPAD);

            low = OneWire_ReadByte();
            high = OneWire_ReadByte();

            // Debug Grezzo dei Byte (Fondamentale!)
            char debug[64];
            snprintf(debug, sizeof(debug), "RAW BYTES: L=0x%02X H=0x%02X\r\n", low, high);
            HAL_UART_Transmit(&huart2, (uint8_t*)debug, strlen(debug), 100);

            // Calcolo temperatura reale
            int16_t raw = (high << 8) | low;
            float temp_f = raw / 16.0f;
            temperature = (int16_t)(temp_f * 100);

            // Stampa finale su UART2
            int len = snprintf(uartBuf, sizeof(uartBuf),
                               "[%lu.%03u] DS18B20 OK | Mux Ch 7 | Temp: %.2f C\r\n",
                               totalSeconds, mSeconds, temp_f);
            HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, len, 100);
        }
    } else {
        // Errore: Nessun sensore risponde al primo reset
        int len = snprintf(uartBuf, sizeof(uartBuf),
                           "[%lu.%03u] DS18B20 ERROR: No presence on Mux Ch 7\r\n",
                           totalSeconds, mSeconds);
        HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, len, 100);
        temperature = 0;
    }

    return temperature;
}
