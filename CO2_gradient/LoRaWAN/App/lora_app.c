/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    lora_app.c
  * @author  MCD Application Team
  * @brief   Application of the LRWAN Middleware
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "platform.h"
#include "sys_app.h"
#include "lora_app.h"
#include "stm32_seq.h"
#include "stm32_timer.h"
#include "utilities_def.h"
#include "app_version.h"
#include "lorawan_version.h"
#include "subghz_phy_version.h"
#include "lora_info.h"
#include "LmHandler.h"
#include "adc_if.h"
#include "CayenneLpp.h"
#include "sys_sensors.h"
#include "flash_if.h"

/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "stdbool.h"
#include "usart.h"
#include "stm32_seq.h"    // for UTIL_SEQ_SetTask / Sleep routines
#include "stm32_lpm.h"    // for UTIL_LPM_EnterLowPower()
#include "stm32_lpm_if.h"  // or whichever header contains the extern
#include "ff.h"
#include "sd_logger.h"
#include "onewire.h"
#include "timer_if.h"
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "app_fatfs.h"
#include "sht3x.h"


/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */
extern SUBGHZ_HandleTypeDef hsubghz;
extern void SD_Force_Reinit(void);

//#define RELAY_BUF_SIZE 64
#define BUFFSIZE 64
#define ASCII_TX_MAX 256
#define ADC_DRY 0      // Valore ADC con sensore all'aria
#define ADC_WET 3723   // Valore ADC con sensore in acqua (3.0V / 3.3V * 4095)
// Definizione dei comandi 1-Wire
#define DS18B20_CMD_SKIPROM          0xCC
#define DS18B20_CMD_CONVERTT         0x44
#define DS18B20_CMD_READSCRATCHPAD   0xBE

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern uint8_t rx_buff[1];
extern SPI_HandleTypeDef hspi1;
extern ADC_HandleTypeDef hadc;


uint8_t readings = 30;
uint16_t buf_co2[100];
uint16_t avg_co2 = 0;
uint8_t i = 0;
uint8_t TxBufferFilteredReading[3]="Z\r\n";
uint8_t RdBuffer[BUFFSIZE];    // Buffer circolare per dati dal sensore
uint8_t RdPCBuffer[BUFFSIZE];  // Buffer lineare per comandi dal PC
uint16_t InS = 0, PCPtr = 0;
uint8_t data[] = "Received\r\n";
uint8_t mode;
char out[80];
char outm[80];
uint16_t readf;
uint16_t readnf;
int len;
uint8_t lora = 1;
uint8_t count;
uint8_t buffer_index;
uint8_t selection = 2;
uint8_t period;
static uint8_t asciiTxBuf[ASCII_TX_MAX];
static volatile uint16_t asciiTxLen = 0;
volatile uint8_t tx_in_progress = 0;
uint16_t readingInterval = 500;
bool finished;
uint8_t sensornum;

sht3x_handle_t sht30 = {
    .i2c_handle = &hi2c2,
    .device_address = SHT3X_I2C_DEVICE_ADDRESS_ADDR_PIN_LOW
};

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/**
  * @brief LoRa State Machine states
  */
typedef enum TxEventType_e
{
  /**
    * @brief Appdata Transmission issue based on timer every TxDutyCycleTime
    */
  TX_ON_TIMER,
  /**
    * @brief Appdata Transmission external event plugged on OnSendEvent( )
    */
  TX_ON_EVENT
  /* USER CODE BEGIN TxEventType_t */

  /* USER CODE END TxEventType_t */
} TxEventType_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/**
  * LEDs period value of the timer in ms
  */
#define LED_PERIOD_TIME 500

/**
  * Join switch period value of the timer in ms
  */
#define JOIN_TIME 2000

/*---------------------------------------------------------------------------*/
/*                             LoRaWAN NVM configuration                     */
/*---------------------------------------------------------------------------*/
/**
  * @brief LoRaWAN NVM Flash address
  * @note last 2 sector of a 128kBytes device
  */
#define LORAWAN_NVM_BASE_ADDRESS                    ((void *)0x0803F000UL)

/* USER CODE BEGIN PD */
static const char *slotStrings[] = { "1", "2", "C", "C_MC", "P", "P_MC" };
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  LoRa End Node send request
  */
static void SendTxData(void);

/**
  * @brief  TX timer callback function
  * @param  context ptr of timer context
  */
static void OnTxTimerEvent(void *context);

/**
  * @brief  join event callback function
  * @param  joinParams status of join
  */
static void OnJoinRequest(LmHandlerJoinParams_t *joinParams);

/**
  * @brief callback when LoRaWAN application has sent a frame
  * @brief  tx event callback function
  * @param  params status of last Tx
  */
static void OnTxData(LmHandlerTxParams_t *params);

/**
  * @brief callback when LoRaWAN application has received a frame
  * @param appData data received in the last Rx
  * @param params status of last Rx
  */
static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params);

/**
  * @brief callback when LoRaWAN Beacon status is updated
  * @param params status of Last Beacon
  */
static void OnBeaconStatusChange(LmHandlerBeaconParams_t *params);

/**
  * @brief callback when system time has been updated
  */
static void OnSysTimeUpdate(void);

/**
  * @brief callback when LoRaWAN application Class is changed
  * @param deviceClass new class
  */
static void OnClassChange(DeviceClass_t deviceClass);

/**
  * @brief  LoRa store context in Non Volatile Memory
  */
static void StoreContext(void);

/**
  * @brief  stop current LoRa execution to switch into non default Activation mode
  */
static void StopJoin(void);

/**
  * @brief  Join switch timer callback function
  * @param  context ptr of Join switch context
  */
static void OnStopJoinTimerEvent(void *context);

/**
  * @brief  Notifies the upper layer that the NVM context has changed
  * @param  state Indicates if we are storing (true) or restoring (false) the NVM context
  */
static void OnNvmDataChange(LmHandlerNvmContextStates_t state);

/**
  * @brief  Store the NVM Data context to the Flash
  * @param  nvm ptr on nvm structure
  * @param  nvm_size number of data bytes which were stored
  */
static void OnStoreContextRequest(void *nvm, uint32_t nvm_size);

/**
  * @brief  Restore the NVM Data context from the Flash
  * @param  nvm ptr on nvm structure
  * @param  nvm_size number of data bytes which were restored
  */
static void OnRestoreContextRequest(void *nvm, uint32_t nvm_size);

/**
  * Will be called each time a Radio IRQ is handled by the MAC layer
  *
  */
static void OnMacProcessNotify(void);

/**
  * @brief Change the periodicity of the uplink frames
  * @param periodicity uplink frames period in ms
  * @note Compliance test protocol callbacks
  */
static void OnTxPeriodicityChanged(uint32_t periodicity);

/**
  * @brief Change the confirmation control of the uplink frames
  * @param isTxConfirmed Indicates if the uplink requires an acknowledgement
  * @note Compliance test protocol callbacks
  */
static void OnTxFrameCtrlChanged(LmHandlerMsgTypes_t isTxConfirmed);

/**
  * @brief Change the periodicity of the ping slot frames
  * @param pingSlotPeriodicity ping slot frames period in ms
  * @note Compliance test protocol callbacks
  */
static void OnPingSlotPeriodicityChanged(uint8_t pingSlotPeriodicity);

/**
  * @brief Will be called to reset the system
  * @note Compliance test protocol callbacks
  */
static void OnSystemReset(void);

/* USER CODE BEGIN PFP */
static void LogToSDCard(void);
static void readCO2(void);
uint16_t average_u16_int(const uint16_t *arr, size_t len, uint8_t index);
static void commUsart1(void);
static void commUsart2(void);
static void printOnUart(void);
static void OnReadTimerEvent(void *context);
uint16_t Read_ADC_Value(void);

/* USER CODE END PFP */

/* Private variables ---------------------------------------------------------*/
/**
  * @brief LoRaWAN default activation type
  */
static ActivationType_t ActivationType = LORAWAN_DEFAULT_ACTIVATION_TYPE;

/**
  * @brief LoRaWAN force rejoin even if the NVM context is restored
  */
static bool ForceRejoin = LORAWAN_FORCE_REJOIN_AT_BOOT;

/**
  * @brief LoRaWAN handler Callbacks
  */
static LmHandlerCallbacks_t LmHandlerCallbacks =
{
  .GetBatteryLevel =              GetBatteryLevel,
  .GetTemperature =               GetTemperatureLevel,
  .GetUniqueId =                  GetUniqueId,
  .GetDevAddr =                   GetDevAddr,
  .OnRestoreContextRequest =      OnRestoreContextRequest,
  .OnStoreContextRequest =        OnStoreContextRequest,
  .OnMacProcess =                 OnMacProcessNotify,
  .OnNvmDataChange =              OnNvmDataChange,
  .OnJoinRequest =                OnJoinRequest,
  .OnTxData =                     OnTxData,
  .OnRxData =                     OnRxData,
  .OnBeaconStatusChange =         OnBeaconStatusChange,
  .OnSysTimeUpdate =              OnSysTimeUpdate,
  .OnClassChange =                OnClassChange,
  .OnTxPeriodicityChanged =       OnTxPeriodicityChanged,
  .OnTxFrameCtrlChanged =         OnTxFrameCtrlChanged,
  .OnPingSlotPeriodicityChanged = OnPingSlotPeriodicityChanged,
  .OnSystemReset =                OnSystemReset,
};

/**
  * @brief LoRaWAN handler parameters
  */
static LmHandlerParams_t LmHandlerParams =
{
  .ActiveRegion =             ACTIVE_REGION,
  .DefaultClass =             LORAWAN_DEFAULT_CLASS,
  .AdrEnable =                LORAWAN_ADR_STATE,
  .IsTxConfirmed =            LORAWAN_DEFAULT_CONFIRMED_MSG_STATE,
  .TxDatarate =               LORAWAN_DEFAULT_DATA_RATE,
  .TxPower =                  LORAWAN_DEFAULT_TX_POWER,
  .PingSlotPeriodicity =      LORAWAN_DEFAULT_PING_SLOT_PERIODICITY,
  .RxBCTimeout =              LORAWAN_DEFAULT_CLASS_B_C_RESP_TIMEOUT
};

/**
  * @brief Type of Event to generate application Tx
  */
static TxEventType_t EventType = TX_ON_TIMER;

/**
  * @brief Timer to handle the application Tx
  */
static UTIL_TIMER_Object_t TxTimer;

/**
  * @brief Tx Timer period
  */
static UTIL_TIMER_Time_t TxPeriodicity = APP_TX_DUTYCYCLE;

/**
  * @brief Join Timer period
  */
static UTIL_TIMER_Object_t StopJoinTimer;

/* USER CODE BEGIN PV */

static UTIL_TIMER_Object_t ReadTimer;

/**
  * @brief User application buffer
  */
static uint8_t AppDataBuffer[LORAWAN_APP_DATA_BUFFER_MAX_SIZE];

/**
  * @brief User application data structure
  */
static LmHandlerAppData_t AppData = { 0, 0, AppDataBuffer };

/**
  * @brief Specifies the state of the application LED
  */
static uint8_t AppLedStateOn = RESET;

/* USER CODE END PV */

/* Exported functions ---------------------------------------------------------*/
/* USER CODE BEGIN EF */

/* USER CODE END EF */

void LoRaWAN_Init(void)
{
  /* USER CODE BEGIN LoRaWAN_Init_LV */
  uint32_t feature_version = 0UL;
  /* USER CODE END LoRaWAN_Init_LV */

  /* USER CODE BEGIN LoRaWAN_Init_1 */

  /* Get LoRaWAN APP version*/
  APP_LOG(TS_OFF, VLEVEL_M, "APPLICATION_VERSION: V%X.%X.%X\r\n",
          (uint8_t)(APP_VERSION_MAIN),
          (uint8_t)(APP_VERSION_SUB1),
          (uint8_t)(APP_VERSION_SUB2));

  /* Get MW LoRaWAN info */
  APP_LOG(TS_OFF, VLEVEL_M, "MW_LORAWAN_VERSION:  V%X.%X.%X\r\n",
          (uint8_t)(LORAWAN_VERSION_MAIN),
          (uint8_t)(LORAWAN_VERSION_SUB1),
          (uint8_t)(LORAWAN_VERSION_SUB2));

  /* Get MW SubGhz_Phy info */
  APP_LOG(TS_OFF, VLEVEL_M, "MW_RADIO_VERSION:    V%X.%X.%X\r\n",
          (uint8_t)(SUBGHZ_PHY_VERSION_MAIN),
          (uint8_t)(SUBGHZ_PHY_VERSION_SUB1),
          (uint8_t)(SUBGHZ_PHY_VERSION_SUB2));

  /* Get LoRaWAN Link Layer info */
  LmHandlerGetVersion(LORAMAC_HANDLER_L2_VERSION, &feature_version);
  APP_LOG(TS_OFF, VLEVEL_M, "L2_SPEC_VERSION:     V%X.%X.%X\r\n",
          (uint8_t)(feature_version >> 24),
          (uint8_t)(feature_version >> 16),
          (uint8_t)(feature_version >> 8));

  /* Get LoRaWAN Regional Parameters info */
  LmHandlerGetVersion(LORAMAC_HANDLER_REGION_VERSION, &feature_version);
  APP_LOG(TS_OFF, VLEVEL_M, "RP_SPEC_VERSION:     V%X-%X.%X.%X\r\n",
          (uint8_t)(feature_version >> 24),
          (uint8_t)(feature_version >> 16),
          (uint8_t)(feature_version >> 8),
          (uint8_t)(feature_version));

  if (FLASH_IF_Init(NULL) != FLASH_IF_OK)
  {
    Error_Handler();
  }

  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_ReadFilteredCO2), UTIL_SEQ_RFU, readCO2);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_USART1), UTIL_SEQ_RFU, commUsart1);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_USART2), UTIL_SEQ_RFU, commUsart2);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_SDLogEvent), UTIL_SEQ_RFU, LogToSDCard);

  mode = 2;

  /* USER CODE END LoRaWAN_Init_1 */

  UTIL_TIMER_Create(&StopJoinTimer, JOIN_TIME, UTIL_TIMER_ONESHOT, OnStopJoinTimerEvent, NULL);

  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LmHandlerProcess), UTIL_SEQ_RFU, LmHandlerProcess);

  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), UTIL_SEQ_RFU, SendTxData);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaStoreContextEvent), UTIL_SEQ_RFU, StoreContext);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaStopJoinEvent), UTIL_SEQ_RFU, StopJoin);

  /* Init Info table used by LmHandler*/
  LoraInfo_Init();

  /* Init the Lora Stack*/
  LmHandlerInit(&LmHandlerCallbacks, APP_VERSION);

  LmHandlerConfigure(&LmHandlerParams);

  /* USER CODE BEGIN LoRaWAN_Init_2 */
  //UTIL_TIMER_Start(&JoinLedTimer);

  /* USER CODE END LoRaWAN_Init_2 */

  LmHandlerJoin(ActivationType, ForceRejoin);

  if (EventType == TX_ON_TIMER)
  {
    /* send every time timer elapses */
    UTIL_TIMER_Create(&TxTimer, TxPeriodicity, UTIL_TIMER_ONESHOT, OnTxTimerEvent, NULL);
    UTIL_TIMER_Start(&TxTimer);
  }
  else
  {
    /* USER CODE BEGIN LoRaWAN_Init_3 */

    /* USER CODE END LoRaWAN_Init_3 */
  }

  /* USER CODE BEGIN LoRaWAN_Init_Last */
  UTIL_TIMER_Create(&ReadTimer, readingInterval, UTIL_TIMER_ONESHOT, OnReadTimerEvent, NULL);
  DWT_Init();
  /* USER CODE END LoRaWAN_Init_Last */
}

/* USER CODE BEGIN PB_Callbacks */

/*void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        tx_in_progress = 0;
    }
}
*/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if (huart->Instance == USART1){
		UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_USART1), CFG_SEQ_Prio_0);
    }



	if (huart->Instance == USART2){
		 UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_USART2), CFG_SEQ_Prio_0);
	 }

}


uint16_t average_u16_int(const uint16_t *arr, size_t len, uint8_t index) {
    if (len == 0) {
        return 0;
    }
    uint32_t sum = 0;

	for (size_t i = len*(index-1); i < len*index; i++) {
		sum += arr[i];
	}

    // Integer division will truncate toward zero.
    return (uint16_t)(sum / len);
}

void readCO2(void)
{
    // 3) Richiedo il dato “Z\r\n”
    HAL_UART_Transmit(&huart1, TxBufferFilteredReading, 3, 0xFFFF);
    //HAL_Delay(30);
}

static void LogToSDCard(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 1) Sblocca pull-up/pull-down dopo stop
    HAL_PWREx_DisablePullUpPullDownConfig();

    // 2) CS alto PRIMA di tutto
    HAL_GPIO_WritePin(GPIOA, CS_SD_Pin, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = CS_SD_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(CS_SD_GPIO_Port, &GPIO_InitStruct);

    // 3) Accendi la SD
    HAL_GPIO_WritePin(GPIOA, MOS_SD_ONOFF_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);

    // 4) Reimposta i pin SPI in modo corretto
    //    (meglio se lo fa MX_SPI1_Init tramite HAL_SPI_MspInit)
    MX_SPI1_Init();
    MX_FATFS_Init();

    // 5) Clock dummy con CS alto
    uint8_t dummy = 0xFF;
    for (int i = 0; i < 10; i++) {
        HAL_SPI_Transmit(&hspi1, &dummy, 1, 10);
    }

    // 6) Ora mount
    char logBuffer[256];
    int pos = 0;
    uint16_t mSeconds;
    uint32_t totalSeconds;
    char uartBuf[50];

    totalSeconds = TIMER_IF_GetTime(&mSeconds);

    // Aggiungiamo il timestamp iniziale
    pos += snprintf(logBuffer + pos, sizeof(logBuffer) - pos, "%lu", totalSeconds);

    // Formattiamo la stringa: il %lu serve per i numeri "long unsigned" (32 bit)
    int len = snprintf(uartBuf, sizeof(uartBuf), "Timestamp RTC: %lu.%03u s\r\n", totalSeconds, mSeconds);
    HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, len, 100);

    // Cicliamo su AppDataBuffer a passi di 2 (dato che ogni valore è uint16_t)
    for (uint8_t j = 0; j + 1 < buffer_index; j += 2)
    {
        uint16_t val = (uint16_t)AppDataBuffer[j] | ((uint16_t)AppDataBuffer[j + 1] << 8);
        pos += snprintf(logBuffer + pos, sizeof(logBuffer) - pos, ",%u", val);

        // Protezione per non eccedere la dimensione del logBuffer
        if (pos >= sizeof(logBuffer) - 10) break;
    }

    strcat(logBuffer, "\n");

    FRESULT fr = SD_Log_String("log.csv", logBuffer);
    if (fr == FR_OK) {
        HAL_UART_Transmit(&huart2, (uint8_t*)"SD success\r\n", 12, 100);
    } else {
        char msg[40];
        snprintf(msg, sizeof(msg), "SD err: %d\r\n", (int)fr);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
    }

    MX_FATFS_deInit();
    // 7) Spegni SPI e rimetti i pin in analog
    HAL_SPI_DeInit(&hspi1);

    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;

    GPIO_InitStruct.Pin = GPIO_PIN_5;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_6 | CS_SD_Pin;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 8) Spegni la SD
    HAL_GPIO_WritePin(GPIOA, MOS_SD_ONOFF_Pin, GPIO_PIN_SET);

    HAL_PWREx_EnablePullUpPullDownConfig();
    HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_A, MOS_SD_ONOFF_Pin);

}

uint16_t Read_ADC_Value(void)
{
	uint32_t adc_val = 0;
	    uint32_t percentage_scaled = 0;

	    if (HAL_ADC_Start(&hadc) != HAL_OK) return 0;

	    if (HAL_ADC_PollForConversion(&hadc, 10) == HAL_OK)
	    {
	        adc_val = HAL_ADC_GetValue(&hadc);

	        // Applichiamo il clamping per evitare valori fuori range
	        if (adc_val <= ADC_DRY) {
	            percentage_scaled = 0;
	        }
	        else if (adc_val >= ADC_WET) {
	            percentage_scaled = 10000; // Rappresenta il 100.00%
	        }
	        else {
	            // Formula di mappatura: ((val - min) * 10000) / (max - min)
	            percentage_scaled = ((adc_val - ADC_DRY) * 10000) / (ADC_WET - ADC_DRY);
	        }
	    }
	    HAL_ADC_Stop(&hadc);

	    return (uint16_t)percentage_scaled;
}

void printOnUart(void){
	// 1) Reset tx buffer length (important!)
	int pos = 0;
	asciiTxLen = 0;

	// 2) Header
	pos += snprintf((char*)asciiTxBuf + pos, ASCII_TX_MAX - pos, "AppDataBuffer (%u bytes): ", (unsigned)buffer_index);

	// 3) Convert each LSB/MSB pair to a decimal number separated by commas
	for (int bi_local = 0; bi_local + 1 < buffer_index && pos < (ASCII_TX_MAX - 16); bi_local += 2)
	{
		uint16_t val = (uint16_t)AppDataBuffer[bi_local] | ((uint16_t)AppDataBuffer[bi_local+1] << 8);
		pos += snprintf((char*)asciiTxBuf + pos, ASCII_TX_MAX - pos, "%u", (unsigned)val);
		if (bi_local + 2 < buffer_index)
		{
			pos += snprintf((char*)asciiTxBuf + pos, ASCII_TX_MAX - pos, ",");
		}
	}

	// 4) Terminatore di riga
	pos += snprintf((char*)asciiTxBuf + pos, ASCII_TX_MAX - pos, "\r\n");
	if (pos < ASCII_TX_MAX) asciiTxBuf[pos] = '\0';
	asciiTxLen = (uint16_t)pos;

	// 5) Avvia trasmissione non-bloccante solo se non ce n'è già una in corso
	if (!tx_in_progress)
	{
		tx_in_progress = 1;
		if (HAL_UART_Transmit(&huart2, asciiTxBuf, asciiTxLen,100) != HAL_OK)
		{
			// errore nel lancio: libera la flag così possiamo provare dopo
			tx_in_progress = 0;
		}
	}
}

void commUsart1(void)
{
	///////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////
	// Modalità Manuale
	// Attesa dei comandi da parte dell'utente
	// Di default modalità 2 (no continua stampa su uart)
	///////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////


	if (selection == 1){

		HAL_UART_Receive_IT(&huart1, rx_buff, 1);
		RdBuffer[InS] = rx_buff[0];
		if (RdBuffer[InS++] == '\n' || InS == BUFFSIZE){
			if (RdBuffer[1] == 'Z'){
				if (mode == 1){
					readf = atoi((char*)RdBuffer + 3);
					readnf = atoi((char*)RdBuffer + 11);
					//float t = HAL_GetTick() / 1000.0f;
					len = sprintf(out, ",%d,%d\r\n", readf, readnf);
					//len = sprintf(out, "Filtered CO2 concentration: %d ppm\r\nNon filtered CO2 concentration: %d ppm\r\n\n", readf, readnf);
					HAL_UART_Transmit(&huart2, (uint8_t*)out, len, 100);
					InS = 0;
				}
				else{
					readf = atoi((char*)RdBuffer + 3);
					len = sprintf(out, "Filtered CO2 concentration: %d ppm\r\n", readf);
					HAL_UART_Transmit(&huart2, (uint8_t*)out, len, 100);
					InS = 0;
				}
			}
			else if (RdBuffer[1] == 'z'){
				readnf = atoi((char*)RdBuffer + 3);
				len = sprintf(out, "Raw CO2 concentration: %d ppm\r\n", readnf);
				HAL_UART_Transmit(&huart2, (uint8_t*)out, len, 100);
				InS = 0;
			}
			else if (RdBuffer[1] == 'K'){
				readf = atoi((char*)RdBuffer + 3);
				mode = readf;
				len = sprintf(out, "Inserted mode %d \r\n", readf);
				HAL_UART_Transmit(&huart2, (uint8_t*)out, len, 100);
				InS = 0;
			}
			else{
				HAL_UART_Transmit(&huart2, RdBuffer, InS, 100);
				InS = 0;
			}
		}
	}


	///////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////
	// Modalità Automatica
	// Raccolta di 10 misure, media, accumulo della media su un buffer
	// Trasmissione del buffer delle medie quando è pieno
	///////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////



	else if (selection == 2){
		// 1) Re‐armo subito per la prossima singola lettura
		HAL_UART_Receive_IT(&huart1, rx_buff, 1);

		// 2) Metto il byte ricevuto in RdBuffer[InS]
		RdBuffer[InS] = rx_buff[0];

		// 3) Se ho chiuso la riga ("\n") o ho saturato il buffer:
		if (RdBuffer[InS++] == '\n' || InS == BUFFSIZE)
		{
			InS = 0;

			//  3a) Se è una riga di risposta CO2 (secondo carattere = 'Z')
			if (RdBuffer[1] == 'Z' && mode == 2)
			{
				// Estraggo il valore numerico
				uint16_t newValue = (uint16_t)atoi((char*)RdBuffer + 3);
				//len = sprintf(out, "value %d ppm\r\n", newValue);
				//HAL_UART_Transmit(&huart2, (uint8_t*)out, len, 100);


				// Accumulo su buf_co2 e conto fino a 10 letture
				buf_co2[i++] = newValue;
				if(i <= readings) sensornum = 1;
				else if(i>readings && i<=readings*2) sensornum = 2;
				else sensornum = 3;

				//readings is the number of readings we take from each sensor, which are to be averaged
				if (i == readings || i == 2*readings || i == 3*readings)
				{

					avg_co2 = average_u16_int(buf_co2, readings, sensornum);

					len = sprintf(out, "%d samples average for sensor %d: %d ppm\r\n", readings, sensornum, avg_co2);
					HAL_UART_Transmit(&huart2, (uint8_t*)out, len, 100);

					// Metto i due byte di avg_co2 in AppDataBuffer
					AppDataBuffer[buffer_index++] = (uint8_t)(avg_co2 & 0xFF);
					AppDataBuffer[buffer_index++] = (uint8_t)((avg_co2 >> 8) & 0xFF);
					//AppData.BufferSize = buffer_index;
					if (sensornum == 3){
						finished = true;
						i = 0;
					}

					// sending the command to sleep to whatever sensor is selected
			        HAL_UART_Transmit(&huart1, (uint8_t*)("K 0\r\n"), 5, 0xFFFF);

					return;
				}

				UTIL_TIMER_Start(&ReadTimer);
				return;
			}

			//  3b) Se è un comando di cambio modalità (“K …”)
			else if (RdBuffer[1] == 'K')
			{

				uint16_t newMode = (uint16_t)atoi((char*)RdBuffer + 2);
				mode = (uint8_t)newMode;

				if(mode == 0){
					len = sprintf(out, "Mode 0 inserted for sensor %d\r\n", sensornum);
					HAL_UART_Transmit(&huart2, (uint8_t*)out, len, 100);
					if (sensornum == 1){
						//switch to sensor 2

						//CO2 sensor x      (bitmap A2 A1 A0)
						//CO2 sensor CO2_1  (bitmap 0  0  1)
						//CO2 sensor CO2_2  (bitmap 0  1  1)
						//CO2 sensor CO2_3  (bitmap 0  0  0)

						HAL_GPIO_WritePin(GPIOA, MUX_A0_Pin, GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOB, MUX_A1_Pin, GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOB, MUX_A2_Pin, GPIO_PIN_RESET);
						sensornum = 2;
			        	// insert the measurement mode to the new sensor selected,
			        	//which will trigger again the ReadTimer timer
			        	HAL_UART_Transmit(&huart1, (uint8_t*)("K 2\r\n"), 5, 0xFFFF);
					}
					else if (sensornum == 2){
					 	//switch to sensor 3

					   	//CO2 sensor x      (bitmap A2 A1 A0)
					   	//CO2 sensor CO2_1  (bitmap 0  0  1)
					  	//CO2 sensor CO2_2  (bitmap 0  1  1)
					   	//CO2 sensor CO2_3  (bitmap 0  0  0)

			        	HAL_GPIO_WritePin(GPIOA, MUX_A0_Pin, GPIO_PIN_RESET);
			        	HAL_GPIO_WritePin(GPIOB, MUX_A1_Pin, GPIO_PIN_RESET);
			        	HAL_GPIO_WritePin(GPIOB, MUX_A2_Pin, GPIO_PIN_RESET);
			        	sensornum = 3;
			        	// insert the measurement mode to the new sensor selected,
			        	//which will trigger again the ReadTimer timer
			        	HAL_UART_Transmit(&huart1, (uint8_t*)("K 2\r\n"), 5, 0xFFFF);
			        }
			        else{
			        	// you have finished to read all the three sensors
						if (!sht3x_init(&sht30)) {
							HAL_UART_Transmit(&huart2, (uint8_t*)"SHT3x access fail\r\n", 19, 100);
						}
						float temperature, humidity;
						sht3x_read_temperature_and_humidity(&sht30, &temperature, &humidity);

						int16_t temp_scaled = (int16_t)(temperature * 100);
						uint16_t hum_scaled = (uint16_t)(humidity * 100);

						// Inserting temp in AppDataBuffer (2 bytes, LSB first)
						AppDataBuffer[buffer_index++] = (uint8_t)(temp_scaled & 0xFF);
						AppDataBuffer[buffer_index++] = (uint8_t)((temp_scaled >> 8) & 0xFF);

				        // Inserting RH in AppDataBuffer (2 bytes, LSB first)
				        AppDataBuffer[buffer_index++] = (uint8_t)(hum_scaled & 0xFF);
				        AppDataBuffer[buffer_index++] = (uint8_t)((hum_scaled >> 8) & 0xFF);

				        len = sprintf(out, "Temp: %.2fC, Hum: %.2f%%RH\r\n", temperature, humidity);
						HAL_UART_Transmit(&huart2, (uint8_t*)out, len, 100);

                        ///////////////////////////////////////////////////////////////////////////////////////////////
						///////////////////////////////////////////////////////////////////////////////////////////////
						///////////////////////////////////////////////////////////////////////////////////////////////
/*
						// 2. Lettura DS18B20 (Temperatura Suolo - Canali MUX 0, 1, 2)
					    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET); // MUXADC_EN ON

					    // Switch PB2 a modalità GPIO per protocollo 1-Wire
					    GPIO_InitTypeDef GPIO_InitStruct = {0};
					    GPIO_InitStruct.Pin = GPIO_PIN_2; // PB2
					    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD; // Open Drain per 1-Wire
					    GPIO_InitStruct.Pull = GPIO_PULLUP;
					    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
					    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

					    for (uint8_t ch = 0; ch < 3; ch++) {
					        // Selezione Canale MUX
					    	// ch will vary between 0 and 2
					    	// when ch = 0:
					    	// ch & 0x01 = 000 & 001 = 000 = FALSE
					    	// ch & 0x02 = 000 & 010 = 000 = FALSE
					    	// ch & 0x04 = 000 & 100 = 000 = FALSE
					    	// when ch = 1:
					    	// ch & 0x01 = 001 & 001 = 001 = TRUE (because different from 000)
					    	// ch & 0x02 = 001 & 010 = 000 = FALSE
					    	// ch & 0x04 = 001 & 100 = 000 = FALSE
					    	// when ch = 2:
					    	// ch & 0x01 = 010 & 001 = 000 = FALSE
					    	// ch & 0x02 = 010 & 010 = 010 = TRUE
					    	// ch & 0x04 = 010 & 100 = 000 = FALSE

					        HAL_GPIO_WritePin(GPIOA, MUX_A0_Pin, (ch & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
					        HAL_GPIO_WritePin(GPIOB, MUX_A1_Pin, (ch & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
					        HAL_GPIO_WritePin(GPIOB, MUX_A2_Pin, (ch & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
					        HAL_Delay(10);

					        // Chiamata alla funzione helper per DS18B20 (che include il delay di conversione)
					        int16_t soil_temp = Read_Soil_Temp_Scaled();

					        AppDataBuffer[buffer_index++] = (uint8_t)(soil_temp & 0xFF);
					        AppDataBuffer[buffer_index++] = (uint8_t)((soil_temp >> 8) & 0xFF);
					    }
					    // 3. Lettura SoilWatch 10 (Umidità Suolo - Canali MUX 3, 4, 5)
					    // Riporta PB2 in modalità ADC
					    MX_ADC_Init(); // Re-inizializza l'ADC per configurare PB2 come ADC_IN4

					    for (uint8_t ch = 3; ch < 6; ch++) {
					        HAL_GPIO_WritePin(GPIOA, MUX_A0_Pin, (ch & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
					        HAL_GPIO_WritePin(GPIOB, MUX_A1_Pin, (ch & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
					        HAL_GPIO_WritePin(GPIOB, MUX_A2_Pin, (ch & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
					        HAL_Delay(5);
					        uint16_t adc_val = Read_ADC_Value(); // Funzione helper ADC
					        AppDataBuffer[buffer_index++] = (uint8_t)(adc_val & 0xFF);
					        AppDataBuffer[buffer_index++] = (uint8_t)((adc_val >> 8) & 0xFF);
					    }

					    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET); // MUXADC_EN OFF
*/
					    ///////////////////////////////////////////////////////////////////////////////////////////////
					    ///////////////////////////////////////////////////////////////////////////////////////////////
					    ///////////////////////////////////////////////////////////////////////////////////////////////

						LogToSDCard();

						// Se ho raccolto x medie invio tramite lora (se lora == 1) o tramite UART (se lora == 0)
				        // Consider that I have 2 bytes per average (with three sensors it means 6 bytes per reading)
				        // I also have 4 bytes for T + RH, which makes the total increase to 10 bytes
						// I also have 2 bytes per soil sensor, which means 12 bytes
						// In total, it comes to 22 bytes of data
						if (buffer_index >= 50)
						{
							if(lora){
							    AppData.BufferSize = buffer_index;
							    LmHandlerSetDutyCycleEnable(false);

							    //if the appdata buffersize is equal to 10 multiplied by the number of sensors (in total 3)
							    // it means we acquired 10 bytes for each sensor, which means 5 averages from each
							    // modifying the 10, allows to set the bytes for each sensor before transmitting via lora
							    UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), CFG_SEQ_Prio_0);
							}

							else{
								printOnUart();

								UTIL_TIMER_Start(&TxTimer);
							}

							// 6) Resetta il buffer di raccolta così possiamo continuare a popolarlo
							buffer_index = 0;
						}

						// inversely, if we have finished the acquisition of the averages and not reached the wanted
						// buffersize, we set Stop2
						else UTIL_LPM_SetStopMode((1 << CFG_LPM_APPLI_Id), UTIL_LPM_ENABLE);
			        }
				}
				else if (mode == 2){
					len = sprintf(out, "Mode 2 inserted for sensor %d\r\n", sensornum);
					HAL_UART_Transmit(&huart2, (uint8_t*)out, len, 100);
					UTIL_TIMER_Start(&ReadTimer);
				}

				//InS = 0;
				return;
			}

			//  3c) Se arriva qualsiasi altra cosa
			else
			{
				return;
			}
		}
    }
}

void commUsart2(void)
{
    HAL_UART_Receive_IT(&huart2, rx_buff, 1);
    RdPCBuffer[PCPtr] = rx_buff[0];

    // Command 'c' to toggle between Manual and Automatic mode
    if (RdPCBuffer[PCPtr] == 'c') {
        if (selection == 1) {
            selection = 2;
            HAL_UART_Transmit(&huart2, (uint8_t*)"Inserted automatic mode\r\n", 24, 100);
            UTIL_TIMER_Start(&TxTimer);
        } else {
            selection = 1;
            HAL_UART_Transmit(&huart2, (uint8_t*)"Inserted manual mode\r\n", 21, 100);
        }
        PCPtr = 0;
        return;
    }

    // Command 'l' to toggle LoRa/UART transmission
    if (RdPCBuffer[PCPtr] == 'l') {
        lora = !lora;
        if (lora) {
            HAL_UART_Transmit(&huart2, (uint8_t*)"Inserted LoRaWAN transmission mode\r\n", 36, 100);
        } else {
            HAL_UART_Transmit(&huart2, (uint8_t*)"Inserted UART transmission mode\r\n", 33, 100);
        }
        PCPtr = 0;
        return;
    }

    // New Command 'q' to cycle through CO2 sensors in Manual Mode
    if (RdPCBuffer[PCPtr] == 'q') {
        // Ensure the MUX is enabled
        HAL_GPIO_WritePin(GPIOA, MUXCO2_EN_Pin, GPIO_PIN_SET);

        // Increment and wrap around (1 -> 2 -> 3 -> 1)
        sensornum = (sensornum % 3) + 1;

        // Apply MUX mapping based on your bitmap:
        // CO2_1: A2=0, A1=0, A0=1
        // CO2_2: A2=0, A1=1, A0=1
        // CO2_3: A2=0, A1=0, A0=0
        if (sensornum == 1) {
            HAL_GPIO_WritePin(GPIOA, MUX_A0_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, MUX_A1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, MUX_A2_Pin, GPIO_PIN_RESET);
        } else if (sensornum == 2) {
            HAL_GPIO_WritePin(GPIOA, MUX_A0_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, MUX_A1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, MUX_A2_Pin, GPIO_PIN_RESET);
        } else { // sensornum == 3
            HAL_GPIO_WritePin(GPIOA, MUX_A0_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, MUX_A1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, MUX_A2_Pin, GPIO_PIN_RESET);
        }

        len = sprintf(out, "Manual selection: CO2 Sensor %d active\r\n", sensornum);
        HAL_UART_Transmit(&huart2, (uint8_t*)out, len, 100);

        PCPtr = 0;
        return;
    }

    // Handling multi-character commands ending in '\n'
    if (RdPCBuffer[PCPtr++] == '\n' || PCPtr == BUFFSIZE) {
        if (RdPCBuffer[0] == 't') {
            period = atoi((char*)RdPCBuffer + 1);
            OnTxPeriodicityChanged(period * 1000);
            len = sprintf(out, "Periodicity set to %lu ms\r\n", TxPeriodicity);
            HAL_UART_Transmit(&huart2, (uint8_t*)out, len, 100);
        } else if (RdPCBuffer[0] == 'n') {
            readings = atoi((char*)RdPCBuffer + 1);
            len = sprintf(out, "Now you will read %d data every cycle\r\n", readings);
            HAL_UART_Transmit(&huart2, (uint8_t*)out, len, 100);
        } else {
            HAL_UART_Transmit(&huart1, RdPCBuffer, PCPtr, 100);
        }
        PCPtr = 0;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

}

/* USER CODE END PB_Callbacks */

/* Private functions ---------------------------------------------------------*/
/* USER CODE BEGIN PrFD */
static void OnReadTimerEvent(void *context)
{
	  if(selection == 2){
		  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_ReadFilteredCO2), CFG_SEQ_Prio_0);
	  }

	  //UTIL_TIMER_Start(&ReadTimer);
}
/* USER CODE END PrFD */

static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params)
{
  /* USER CODE BEGIN OnRxData_1 */
  uint8_t RxPort = 0;

  if (params != NULL)
  {

    if (params->IsMcpsIndication)
    {
      if (appData != NULL)
      {
        RxPort = appData->Port;
        if (appData->Buffer != NULL)
        {
          switch (appData->Port)
          {
            case LORAWAN_SWITCH_CLASS_PORT:
              /*this port switches the class*/
              if (appData->BufferSize == 1)
              {
                switch (appData->Buffer[0])
                {
                  case 0:
                  {
                    LmHandlerRequestClass(CLASS_A);
                    break;
                  }
                  case 1:
                  {
                    LmHandlerRequestClass(CLASS_B);
                    break;
                  }
                  case 2:
                  {
                    LmHandlerRequestClass(CLASS_C);
                    break;
                  }
                  default:
                    break;
                }
              }
              break;
            case LORAWAN_USER_APP_PORT:
              if (appData->BufferSize == 1)
              {
                AppLedStateOn = appData->Buffer[0] & 0x01;
                if (AppLedStateOn == RESET)
                {
                  APP_LOG(TS_OFF, VLEVEL_H, "LED OFF\r\n");
                }
                else
                {
                  APP_LOG(TS_OFF, VLEVEL_H, "LED ON\r\n");
                }
              }
              break;

            default:

              break;
          }
        }
      }
    }
    if (params->RxSlot < RX_SLOT_NONE)
    {
      APP_LOG(TS_OFF, VLEVEL_H, "###### D/L FRAME:%04d | PORT:%d | DR:%d | SLOT:%s | RSSI:%d | SNR:%d\r\n",
              params->DownlinkCounter, RxPort, params->Datarate, slotStrings[params->RxSlot],
              params->Rssi, params->Snr);
    }
  }
  /* USER CODE END OnRxData_1 */
}

static void SendTxData(void)
{
  /* USER CODE BEGIN SendTxData_1 */
  LmHandlerErrorStatus_t status = LORAMAC_HANDLER_ERROR;
  UTIL_TIMER_Time_t nextTxIn = 0;

  if (LmHandlerIsBusy() == false)
  {
	//HAL_UART_Transmit(&huart2, (uint8_t*)"LmHandler is not busy\r\n", 23, 100);
    AppData.Port = LORAWAN_USER_APP_PORT;

    status = LmHandlerSend(&AppData, LmHandlerParams.IsTxConfirmed, false);
    if (LORAMAC_HANDLER_SUCCESS == status)
    {
      APP_LOG(TS_ON, VLEVEL_L, "SEND REQUEST\r\n");
    }
    else if (LORAMAC_HANDLER_BUSY_ERROR == status)
    {
      APP_LOG(TS_ON, VLEVEL_L, "busy error\r\n");
    }
    else if (LORAMAC_HANDLER_NO_NETWORK_JOINED == status)
    {
      APP_LOG(TS_ON, VLEVEL_L, "no network joined\r\n");
    }
    else if (LORAMAC_HANDLER_COMPLIANCE_RUNNING == status)
    {
      APP_LOG(TS_ON, VLEVEL_L, "compliance running\r\n");
    }
    else if (LORAMAC_HANDLER_CRYPTO_ERROR == status)
    {
      APP_LOG(TS_ON, VLEVEL_L, "crypto error\r\n");
    }
    else if (LORAMAC_HANDLER_DUTYCYCLE_RESTRICTED == status)
    {
    	APP_LOG(TS_ON, VLEVEL_L, "dutycycle restricted\r\n");
      //nextTxIn = LmHandlerGetDutyCycleWaitTime();
     // if (nextTxIn > 0)
     // {
     //   APP_LOG(TS_ON, VLEVEL_L, "Next Tx in  : ~%d second(s)\r\n", (nextTxIn / 1000));
     // }
    }
  }

  if (EventType == TX_ON_TIMER)
  {
    UTIL_TIMER_Stop(&TxTimer);
    UTIL_TIMER_SetPeriod(&TxTimer, MAX(nextTxIn, TxPeriodicity));
    UTIL_TIMER_Start(&TxTimer);
  }

  AppData.BufferSize = 0;

  /* USER CODE END SendTxData_1 */
}

static void OnTxTimerEvent(void *context)
{
  /* USER CODE BEGIN OnTxTimerEvent_1 */
	finished = false;
	//Enable the MUX
	HAL_GPIO_WritePin(GPIOA, MUXCO2_EN_Pin, GPIO_PIN_SET);

	//CO2 sensor x      (bitmap A2 A1 A0)
	//CO2 sensor CO2_1  (bitmap 0  0  1)
	//CO2 sensor CO2_2  (bitmap 0  1  1)
	//CO2 sensor CO2_3  (bitmap 0  0  0)


	HAL_GPIO_WritePin(GPIOA, MUX_A0_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, MUX_A1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, MUX_A2_Pin, GPIO_PIN_RESET);

	sensornum = 1;
	  //UTIL_SEQ_ENTER_CRITICAL_SECTION();
	  if(selection == 2){
		  UTIL_LPM_SetStopMode((1 << CFG_LPM_APPLI_Id), UTIL_LPM_DISABLE);
		  HAL_UART_Receive_IT(&huart1, rx_buff, 1);

		  InS = 0;

		  //HAL_UART_Transmit(&huart2, (uint8_t *)"tim\r\n", 5, 0xFFFF); // arriva senza problemi

		  //if (mode != 2)
		  //{
			  //HAL_UART_Transmit(&huart2, (uint8_t*)("Changing mode to 2\r\n"), 20, 0xFFFF);
			  HAL_UART_Transmit(&huart1, (uint8_t*)("K 2\r\n"), 5, 0xFFFF);
		  //}


		  //UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_ReadFilteredCO2), CFG_SEQ_Prio_0);
		  //UTIL_TIMER_Start(&ReadTimer);
	  }


  /* USER CODE END OnTxTimerEvent_1 */

  /*Wait for next tx slot*/
  UTIL_TIMER_Start(&TxTimer);
  /* USER CODE BEGIN OnTxTimerEvent_2 */

  /* USER CODE END OnTxTimerEvent_2 */
}

/* USER CODE BEGIN PrFD_LedEvents */


/* USER CODE END PrFD_LedEvents */

static void OnTxData(LmHandlerTxParams_t *params)
{
  /* USER CODE BEGIN OnTxData_1 */
  if ((params != NULL))
  {
    /* Process Tx event only if its a mcps response to prevent some internal events (mlme) */
    if (params->IsMcpsConfirm != 0)
    {
      //UTIL_TIMER_Start(&TxLedTimer);

      APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### ========== MCPS-Confirm =============\r\n");
      APP_LOG(TS_OFF, VLEVEL_M, "###### U/L FRAME:%04d | PORT:%d | DR:%d | PWR:%d", params->UplinkCounter,
              params->AppData.Port, params->Datarate, params->TxPower);

      APP_LOG(TS_OFF, VLEVEL_M, " | MSG TYPE:");
      if (params->MsgType == LORAMAC_HANDLER_CONFIRMED_MSG)
      {
        APP_LOG(TS_OFF, VLEVEL_M, "CONFIRMED [%s]\r\n", (params->AckReceived != 0) ? "ACK" : "NACK");
      }
      else
      {
        APP_LOG(TS_OFF, VLEVEL_M, "UNCONFIRMED\r\n");
      }
    }
  }
  UTIL_LPM_SetStopMode((1 << CFG_LPM_APPLI_Id), UTIL_LPM_ENABLE);


  /* USER CODE END OnTxData_1 */
}

static void OnJoinRequest(LmHandlerJoinParams_t *joinParams)
{
  /* USER CODE BEGIN OnJoinRequest_1 */
  if (joinParams != NULL)
  {
    if (joinParams->Status == LORAMAC_HANDLER_SUCCESS)
    {
      APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### = JOINED = ");
      if (joinParams->Mode == ACTIVATION_TYPE_ABP)
      {
        APP_LOG(TS_OFF, VLEVEL_M, "ABP ======================\r\n");
      }
      else
      {
        APP_LOG(TS_OFF, VLEVEL_M, "OTAA =====================\r\n");
      }
    }
    else
    {
      APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### = JOIN FAILED\r\n");
    }

    APP_LOG(TS_OFF, VLEVEL_H, "###### U/L FRAME:JOIN | DR:%d | PWR:%d\r\n", joinParams->Datarate, joinParams->TxPower);
  }
  /* USER CODE END OnJoinRequest_1 */
}

static void OnBeaconStatusChange(LmHandlerBeaconParams_t *params)
{
  /* USER CODE BEGIN OnBeaconStatusChange_1 */
  if (params != NULL)
  {
    switch (params->State)
    {
      default:
      case LORAMAC_HANDLER_BEACON_LOST:
      {
        APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### BEACON LOST\r\n");
        break;
      }
      case LORAMAC_HANDLER_BEACON_RX:
      {
        APP_LOG(TS_OFF, VLEVEL_M,
                "\r\n###### BEACON RECEIVED | DR:%d | RSSI:%d | SNR:%d | FQ:%d | TIME:%d | DESC:%d | "
                "INFO:02X%02X%02X %02X%02X%02X\r\n",
                params->Info.Datarate, params->Info.Rssi, params->Info.Snr, params->Info.Frequency,
                params->Info.Time.Seconds, params->Info.GwSpecific.InfoDesc,
                params->Info.GwSpecific.Info[0], params->Info.GwSpecific.Info[1],
                params->Info.GwSpecific.Info[2], params->Info.GwSpecific.Info[3],
                params->Info.GwSpecific.Info[4], params->Info.GwSpecific.Info[5]);
        break;
      }
      case LORAMAC_HANDLER_BEACON_NRX:
      {
        APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### BEACON NOT RECEIVED\r\n");
        break;
      }
    }
  }
  /* USER CODE END OnBeaconStatusChange_1 */
}

static void OnSysTimeUpdate(void)
{
  /* USER CODE BEGIN OnSysTimeUpdate_1 */

  /* USER CODE END OnSysTimeUpdate_1 */
}

static void OnClassChange(DeviceClass_t deviceClass)
{
  /* USER CODE BEGIN OnClassChange_1 */
  APP_LOG(TS_OFF, VLEVEL_M, "Switch to Class %c done\r\n", "ABC"[deviceClass]);
  /* USER CODE END OnClassChange_1 */
}

static void OnMacProcessNotify(void)
{
  /* USER CODE BEGIN OnMacProcessNotify_1 */

  /* USER CODE END OnMacProcessNotify_1 */
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LmHandlerProcess), CFG_SEQ_Prio_0);

  /* USER CODE BEGIN OnMacProcessNotify_2 */

  /* USER CODE END OnMacProcessNotify_2 */
}

static void OnTxPeriodicityChanged(uint32_t periodicity)
{
  /* USER CODE BEGIN OnTxPeriodicityChanged_1 */

  /* USER CODE END OnTxPeriodicityChanged_1 */
  TxPeriodicity = periodicity;

  if (TxPeriodicity == 0)
  {
    /* Revert to application default periodicity */
    TxPeriodicity = APP_TX_DUTYCYCLE;
  }

  /* Update timer periodicity */
  UTIL_TIMER_Stop(&TxTimer);
  UTIL_TIMER_SetPeriod(&TxTimer, TxPeriodicity);
  UTIL_TIMER_Start(&TxTimer);
  /* USER CODE BEGIN OnTxPeriodicityChanged_2 */

  /* USER CODE END OnTxPeriodicityChanged_2 */
}

static void OnTxFrameCtrlChanged(LmHandlerMsgTypes_t isTxConfirmed)
{
  /* USER CODE BEGIN OnTxFrameCtrlChanged_1 */

  /* USER CODE END OnTxFrameCtrlChanged_1 */
  LmHandlerParams.IsTxConfirmed = isTxConfirmed;
  /* USER CODE BEGIN OnTxFrameCtrlChanged_2 */

  /* USER CODE END OnTxFrameCtrlChanged_2 */
}

static void OnPingSlotPeriodicityChanged(uint8_t pingSlotPeriodicity)
{
  /* USER CODE BEGIN OnPingSlotPeriodicityChanged_1 */

  /* USER CODE END OnPingSlotPeriodicityChanged_1 */
  LmHandlerParams.PingSlotPeriodicity = pingSlotPeriodicity;
  /* USER CODE BEGIN OnPingSlotPeriodicityChanged_2 */

  /* USER CODE END OnPingSlotPeriodicityChanged_2 */
}

static void OnSystemReset(void)
{
  /* USER CODE BEGIN OnSystemReset_1 */

  /* USER CODE END OnSystemReset_1 */
  if ((LORAMAC_HANDLER_SUCCESS == LmHandlerHalt()) && (LmHandlerJoinStatus() == LORAMAC_HANDLER_SET))
  {
    NVIC_SystemReset();
  }
  /* USER CODE BEGIN OnSystemReset_Last */

  /* USER CODE END OnSystemReset_Last */
}

static void StopJoin(void)
{
  /* USER CODE BEGIN StopJoin_1 */
  /* USER CODE END StopJoin_1 */

  UTIL_TIMER_Stop(&TxTimer);

  if (LORAMAC_HANDLER_SUCCESS != LmHandlerStop())
  {
    APP_LOG(TS_OFF, VLEVEL_M, "LmHandler Stop on going ...\r\n");
  }
  else
  {
    APP_LOG(TS_OFF, VLEVEL_M, "LmHandler Stopped\r\n");
    if (LORAWAN_DEFAULT_ACTIVATION_TYPE == ACTIVATION_TYPE_ABP)
    {
      ActivationType = ACTIVATION_TYPE_OTAA;
      APP_LOG(TS_OFF, VLEVEL_M, "LmHandler switch to OTAA mode\r\n");
    }
    else
    {
      ActivationType = ACTIVATION_TYPE_ABP;
      APP_LOG(TS_OFF, VLEVEL_M, "LmHandler switch to ABP mode\r\n");
    }
    LmHandlerConfigure(&LmHandlerParams);
    LmHandlerJoin(ActivationType, true);
    UTIL_TIMER_Start(&TxTimer);
  }
  UTIL_TIMER_Start(&StopJoinTimer);
  /* USER CODE BEGIN StopJoin_Last */

  /* USER CODE END StopJoin_Last */
}

static void OnStopJoinTimerEvent(void *context)
{
  /* USER CODE BEGIN OnStopJoinTimerEvent_1 */

  /* USER CODE END OnStopJoinTimerEvent_1 */
  if (ActivationType == LORAWAN_DEFAULT_ACTIVATION_TYPE)
  {
    UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaStopJoinEvent), CFG_SEQ_Prio_0);
  }
  /* USER CODE BEGIN OnStopJoinTimerEvent_Last */
  /* USER CODE END OnStopJoinTimerEvent_Last */
}

static void StoreContext(void)
{
  LmHandlerErrorStatus_t status = LORAMAC_HANDLER_ERROR;

  /* USER CODE BEGIN StoreContext_1 */

  /* USER CODE END StoreContext_1 */
  status = LmHandlerNvmDataStore();

  if (status == LORAMAC_HANDLER_NVM_DATA_UP_TO_DATE)
  {
    APP_LOG(TS_OFF, VLEVEL_M, "NVM DATA UP TO DATE\r\n");
  }
  else if (status == LORAMAC_HANDLER_ERROR)
  {
    APP_LOG(TS_OFF, VLEVEL_M, "NVM DATA STORE FAILED\r\n");
  }
  /* USER CODE BEGIN StoreContext_Last */

  /* USER CODE END StoreContext_Last */
}

static void OnNvmDataChange(LmHandlerNvmContextStates_t state)
{
  /* USER CODE BEGIN OnNvmDataChange_1 */

  /* USER CODE END OnNvmDataChange_1 */
  if (state == LORAMAC_HANDLER_NVM_STORE)
  {
    APP_LOG(TS_OFF, VLEVEL_M, "NVM DATA STORED\r\n");
  }
  else
  {
    APP_LOG(TS_OFF, VLEVEL_M, "NVM DATA RESTORED\r\n");
  }
  /* USER CODE BEGIN OnNvmDataChange_Last */

  /* USER CODE END OnNvmDataChange_Last */
}

static void OnStoreContextRequest(void *nvm, uint32_t nvm_size)
{
  /* USER CODE BEGIN OnStoreContextRequest_1 */

  /* USER CODE END OnStoreContextRequest_1 */
  /* store nvm in flash */
  if (FLASH_IF_Erase(LORAWAN_NVM_BASE_ADDRESS, FLASH_PAGE_SIZE) == FLASH_IF_OK)
  {
    FLASH_IF_Write(LORAWAN_NVM_BASE_ADDRESS, (const void *)nvm, nvm_size);
  }
  /* USER CODE BEGIN OnStoreContextRequest_Last */

  /* USER CODE END OnStoreContextRequest_Last */
}

static void OnRestoreContextRequest(void *nvm, uint32_t nvm_size)
{
  /* USER CODE BEGIN OnRestoreContextRequest_1 */

  /* USER CODE END OnRestoreContextRequest_1 */
  FLASH_IF_Read(nvm, LORAWAN_NVM_BASE_ADDRESS, nvm_size);
  /* USER CODE BEGIN OnRestoreContextRequest_Last */

  /* USER CODE END OnRestoreContextRequest_Last */
}

