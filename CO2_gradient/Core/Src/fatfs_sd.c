#include "fatfs_sd.h"
#include "main.h"
#include "sys_app.h"

// Привязываем hspi1
extern SPI_HandleTypeDef hspi1;
#define HSPI_SDCARD hspi1

// --- КОМАНДЫ SD-КАРТЫ ---
#define CMD0    (0x40+0)
#define CMD1    (0x40+1)
#define CMD8    (0x40+8)
#define CMD17   (0x40+17)
#define CMD24   (0x40+24)
#define CMD55   (0x40+55)
#define CMD58   (0x40+58)
#define ACMD41  (0x80 | 0x40 | 41)

static volatile DSTATUS Stat = STA_NOINIT;
static uint8_t CardType;

// --- ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ SPI ---

// Надежная функция: читает байт, обязательно отправляя 0xFF
static uint8_t SPI_RxByte(void) {
    uint8_t dummy = 0xFF, data = 0xFF;
    HAL_SPI_TransmitReceive(&HSPI_SDCARD, &dummy, &data, 1, 10);
    return data;
}

static uint8_t SD_ReadyWait(void) {
    uint8_t res;
    uint32_t timeout = HAL_GetTick() + 500;
    do {
        res = SPI_RxByte();
    } while ((res != 0xFF) && (HAL_GetTick() < timeout));
    return res;
}

static uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg) {
    uint8_t n, res;

    if (cmd & 0x80) {
        cmd &= 0x7F;
        res = SD_SendCmd(CMD55, 0);
        if (res > 1) return res;
    }

    if (cmd != CMD0 && SD_ReadyWait() != 0xFF) return 0xFF;

    // Отправляем команду
    uint8_t cmd_packet[6];
    cmd_packet[0] = cmd;
    cmd_packet[1] = (uint8_t)(arg >> 24);
    cmd_packet[2] = (uint8_t)(arg >> 16);
    cmd_packet[3] = (uint8_t)(arg >> 8);
    cmd_packet[4] = (uint8_t)arg;
    cmd_packet[5] = 0x01; // CRC по умолчанию
    if (cmd == CMD0) cmd_packet[5] = 0x95;
    if (cmd == CMD8) cmd_packet[5] = 0x87;

    for(int i=0; i<6; i++) {
        uint8_t dummy = 0xFF;
        HAL_SPI_TransmitReceive(&HSPI_SDCARD, &cmd_packet[i], &dummy, 1, 10);
    }

    n = 10;
    do {
        res = SPI_RxByte();
    } while ((res & 0x80) && --n);

    return res;
}

// --- ОСНОВНЫЕ ФУНКЦИИ ДРАЙВЕРА ---

DSTATUS SD_disk_initialize(BYTE pdrv) {
    uint8_t n, ocr[4];
    uint32_t timeout;

    if (pdrv) return STA_NOINIT;

    Stat = STA_NOINIT;
    CardType = 0;

    CS_SD_HIGH();
    HAL_Delay(150);

    for (n = 0; n < 10; n++) SPI_RxByte(); // Протряска 0xFF

    CS_SD_LOW();

    uint8_t cmd0_res = SD_SendCmd(CMD0, 0);
    APP_LOG(TS_ON, VLEVEL_L, "DEBUG: CMD0 = 0x%02X\r\n", cmd0_res);

    if (cmd0_res == 1) {
        timeout = HAL_GetTick() + 1000;
        uint8_t cmd8_res = SD_SendCmd(CMD8, 0x1AA);

        if (cmd8_res == 1) {
            for (n = 0; n < 4; n++) ocr[n] = SPI_RxByte();
            if (ocr[2] == 0x01 && ocr[3] == 0xAA) {
                while (HAL_GetTick() < timeout && SD_SendCmd(ACMD41, 1UL << 30)) {
                    HAL_Delay(10);
                }
                if (HAL_GetTick() < timeout && SD_SendCmd(CMD58, 0) == 0) {
                    for (n = 0; n < 4; n++) ocr[n] = SPI_RxByte();
                    CardType = (ocr[0] & 0x40) ? 6 : 2;
                }
            }
        } else {
            uint8_t cmd = (SD_SendCmd(ACMD41, 0) <= 1) ? ACMD41 : CMD1;
            while (HAL_GetTick() < timeout && SD_SendCmd(cmd, 0)) {
                HAL_Delay(10);
            }
            CardType = (cmd == ACMD41) ? 2 : 1;
        }
    }

    CS_SD_HIGH();
    SPI_RxByte();

    if (CardType) {
        Stat &= ~STA_NOINIT;
        APP_LOG(TS_ON, VLEVEL_L, "DEBUG: Init OK. Type = %d\r\n", CardType);
    } else {
        APP_LOG(TS_ON, VLEVEL_L, "DEBUG: Init FAILED!\r\n");
    }

    return Stat;
}

DSTATUS SD_disk_status(BYTE pdrv) {
    if (pdrv) return STA_NOINIT;
    return Stat;
}

DRESULT SD_disk_read(BYTE pdrv, BYTE* buff, DWORD sector, UINT count) {
    if (pdrv || !count) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    if (!(CardType & 4)) sector *= 512;

    CS_SD_LOW();
    SPI_RxByte();

    DRESULT res = RES_ERROR;

    if (count == 1) {
        if (SD_SendCmd(CMD17, sector) == 0) {
            uint32_t t = HAL_GetTick() + 2000;
            uint8_t token;
            do {
                token = SPI_RxByte();
            } while (token == 0xFF && HAL_GetTick() < t);

            if (token == 0xFE) {
                // ИСПРАВЛЕНИЕ: Читаем строго побайтово, отправляя 0xFF
                for (UINT i = 0; i < 512; i++) {
                    buff[i] = SPI_RxByte();
                }
                SPI_RxByte(); SPI_RxByte(); // Пропуск CRC
                res = RES_OK;
            } else {
                APP_LOG(TS_ON, VLEVEL_L, "DEBUG: Read token err = 0x%02X\r\n", token);
            }
        }
    }

    CS_SD_HIGH();
    SPI_RxByte();
    return res;
}

DRESULT SD_disk_write(BYTE pdrv, const BYTE* buff, DWORD sector, UINT count) {
    if (pdrv || !count) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    if (!(CardType & 4)) sector *= 512;

    CS_SD_LOW();
    SPI_RxByte();

    DRESULT res = RES_ERROR;

    if (count == 1) {
        if ((SD_SendCmd(CMD24, sector) == 0) && (SD_ReadyWait() == 0xFF)) {

            // Стартовый токен данных
            uint8_t start_tok = 0xFE;
            uint8_t dummy = 0xFF;
            HAL_SPI_TransmitReceive(&HSPI_SDCARD, &start_tok, &dummy, 1, 10);

            // Пишем строго побайтово
            for (UINT i = 0; i < 512; i++) {
                HAL_SPI_TransmitReceive(&HSPI_SDCARD, (uint8_t*)&buff[i], &dummy, 1, 10);
            }

            // CRC
            HAL_SPI_TransmitReceive(&HSPI_SDCARD, &dummy, &dummy, 1, 10);
            HAL_SPI_TransmitReceive(&HSPI_SDCARD, &dummy, &dummy, 1, 10);

            uint8_t dataRes = SPI_RxByte();
            if ((dataRes & 0x1F) == 0x05) {
                if (SD_ReadyWait() == 0xFF) res = RES_OK;
            } else {
                APP_LOG(TS_ON, VLEVEL_L, "DEBUG: Write err = 0x%02X\r\n", dataRes);
            }
        }
    }

    CS_SD_HIGH();
    SPI_RxByte();
    return res;
}

DRESULT SD_disk_ioctl(BYTE pdrv, BYTE cmd, void* buff) {
    if (pdrv) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    DRESULT res = RES_ERROR;
    CS_SD_LOW();

    switch (cmd) {
        case CTRL_SYNC:
            if (SD_ReadyWait() == 0xFF) res = RES_OK;
            break;
        default:
            res = RES_PARERR;
    }

    CS_SD_HIGH();
    return res;
}
