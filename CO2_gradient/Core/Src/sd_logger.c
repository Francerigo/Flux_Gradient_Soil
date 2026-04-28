#include "sd_logger.h"
#include "main.h"    // Serve per conoscere i tipi HAL e le define dei pin
#include <string.h>
#include <stdio.h>    // Risolve l'errore di snprintf

extern UART_HandleTypeDef huart2;

static FATFS fs; // Dichiarata static per non appesantire lo stack

FRESULT SD_Log_String(const char* filename, const char* data_string) {
    FIL fil;
    FRESULT fres;
    UINT bytesWrote;

    // Forza lo stato NOINIT se hai accesso alla variabile Stat (opzionale ma consigliato)
    // extern DSTATUS Stat; Stat = STA_NOINIT;

    fres = f_mount(&fs, "", 1);
    if (fres != FR_OK) {
        // Se qui ti dà errore 3 (FR_NOT_READY), è un problema di velocità SPI o di Stat
        char errMsg[32];
        snprintf(errMsg, sizeof(errMsg), "Mount Err: %d\r\n", (int)fres);
        HAL_UART_Transmit(&huart2, (uint8_t*)errMsg, strlen(errMsg), 100);
        return fres;
    }

    // Un piccolo delay dopo il mount aiuta alcune schede a finire l'auto-init
    HAL_Delay(20);

    fres = f_open(&fil, filename, FA_WRITE | FA_OPEN_APPEND);
    if (fres == FR_OK) {
        fres = f_write(&fil, data_string, strlen(data_string), &bytesWrote);
        f_close(&fil);
    } else {
        char errMsg[32];
        snprintf(errMsg, sizeof(errMsg), "Open Err: %d\r\n", (int)fres);
        HAL_UART_Transmit(&huart2, (uint8_t*)errMsg, strlen(errMsg), 100);
    }

    // Smonta sempre se spegni il MOS subito dopo
    f_mount(NULL, "", 0);

    return fres;
}
