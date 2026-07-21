/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE BEGIN First_Private_Defines */
#define NUM_ANALOG  4U
#define NUM_INPUTS  11U
#define NUM_OUTPUTS 4U

/* ---- Addon Ekspansi Input (PCF8574, dibaca & dikirim oleh ESP32) ----
 * Konvensi bit gabungan (disepakati dengan firmware ESP32):
 *   bit 0..10  -> I1..I11   (native GPIO STM32, lihat g_inputs[])
 *   bit 11..18 -> I12..I19  (addon PCF8574, dikirim ESP32 via FRAME_TYPE_ADDON)
 */
#define NUM_ADDON_INPUTS       8U
#define ADDON_INPUT_BIT_OFFSET 11U
#define NUM_INPUTS_TOTAL       (ADDON_INPUT_BIT_OFFSET + NUM_ADDON_INPUTS)  /* = 19 */
/* USER CODE END First_Private_Defines */


/* DS3231 RTC Time Structure */
typedef struct
{
    uint8_t  seconds;
    uint8_t  minutes;
    uint8_t  hours;
    uint8_t  dayOfWeek;
    uint8_t  day;
    uint8_t  month;
    uint16_t year;
} DS3231_TimeTypeDef;

/* GPIO Pin Definition */
typedef struct {
    GPIO_TypeDef *Port;
    uint16_t      Pin;
} IO_PinDef_t;

/* Input Index Enum */
typedef enum {
    IN_1 = 0, IN_2, IN_3, IN_4, IN_5,
    IN_6, IN_7, IN_8, IN_9, IN_10, IN_11
} InputId_t;

/* Output Index Enum */
typedef enum {
    OUT_1 = 0, OUT_2, OUT_3, OUT_4
} OutputId_t;

/* Rule Logic Enum */
typedef enum {
    RULE_LOGIC_OFF = 0,
    RULE_LOGIC_OR,
    RULE_LOGIC_AND,
    RULE_LOGIC_NOT
} RuleLogic_t;

/* Rule Definition */
typedef struct {
    RuleLogic_t logic;
    uint8_t     inputs[NUM_INPUTS_TOTAL];  /* referensi input bisa I1..I20 */
    uint8_t     inputCount;
} RuleDef_t;

/* Measurement Structure */
typedef struct {
    uint32_t           seq;
    DS3231_TimeTypeDef rtc;
    int16_t            temp;        		// °C × 10
    int16_t            hum;        			// % × 10
    uint32_t           dig_in;      		// bitmask, bit0..19 = I1..I20 (lihat NUM_INPUTS_TOTAL)
    uint16_t           an_in[NUM_ANALOG];   // 0-255
    uint8_t            q;           		// output bitmask
} Measurement_t;

/* TX Context */
typedef struct {
    uint32_t       seq;
    char           jsonLine[320];
    uint16_t       jsonLen;
    uint8_t        retryCount;
    uint32_t       ackDeadlineMs;
    Measurement_t  measurement;   /* simpan data penuh utk retry TLV identik */
} TxContext_t;

/* Application State */
typedef enum {
    APP_STATE_INIT = 0,
    APP_STATE_SD_MOUNT,
    APP_STATE_LOG_RECOVER,
    APP_STATE_RUN
} AppState_t;

/* TX State */
typedef enum {
    TX_STATE_IDLE = 0,
    TX_STATE_SEND_FRAME,
    TX_STATE_WAIT_TX_COMPLETE,
    TX_STATE_WAIT_ACK,
    TX_STATE_ACK_OK,
    TX_STATE_RETRY_OR_FAIL,
    TX_STATE_MOVE_TO_ERROR
} TxState_t;

typedef struct
{
    uint8_t sensor;
    uint8_t acquisition;
    uint8_t ack;
    uint8_t tx;
    uint8_t sd;
} WatchdogHealth_t;

static WatchdogHealth_t wd;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ==================== TLV FORMAT (UART only) ==================== */
#define TLV_SOF             0x7E

#define TLV_TAG_SEQ         0x01    // uint32 BE
#define TLV_TAG_TEMP        0x02    // int16 BE (°C × 10)
#define TLV_TAG_HUM         0x03    // int16 BE (% × 10)
#define TLV_TAG_DIG_IN      0x04    // uint32 BE (bitmask, bit0..19 = I1..I20 - lihat NUM_INPUTS_TOTAL). BREAKING CHANGE: sebelumnya uint16 (2 byte), sekarang 4 byte - ESP32 wajib update decode-nya juga.
#define TLV_TAG_AN_IN       0x05    // uint16[4] BE
#define TLV_TAG_Q           0x06    // uint8 (bitmask)
#define TLV_TAG_TIMESTAMP   0x07    // uint32 BE

/* Frame types for UART communication */
#define FRAME_TYPE_DATA     0x01
#define FRAME_TYPE_ACK      0x02
#define FRAME_TYPE_RULES    0x03
#define FRAME_TYPE_RTC      0x04
#define FRAME_TYPE_ERROR    0x05   // arah STM32 -> ESP32 (laporan error)
#define FRAME_TYPE_ADDON    0x06   // arah ESP32 -> STM32 (status input addon PCF8574)

/* TLV tags from ESP32 */
#define TAG_RULE            0x10
#define TAG_RTC_TS          0x20
#define TAG_ADDON_MASK      0x40   // payload 1 byte: bitmask 8 input PCF8574 (aktif=1), lihat FRAME_TYPE_ADDON

/* TLV tags for error report STM32 -> ESP */
#define TLV_TAG_ERR_MODULE  0x30   // u8
#define TLV_TAG_ERR_CODE    0x31   // u8
#define TLV_TAG_ERR_ACTIVE  0x32   // u8 (1 = error, 0 = clear)

/* I2C Addresses */
#define DS3231_ADDR         (0x68U << 1)
#define ADS1115_ADDR        (0x48U << 1)

/* ADS1115 */
#define ADS_FS_VOLT         4.096f
#define ADC_INPUT_MAX_VOLT  3.3f
#define ADS_READ_PERIOD_MS  100U

/* mid: module id */
#define MOD_RTC   1
#define MOD_DHT   2
#define MOD_ADS   3
#define MOD_SD    4

/* st: status */
#define ST_OK     0
#define ST_ERR    1

/* ec: error code */
#define ERR_NONE      0
#define ERR_TIMEOUT   1
#define ERR_NOT_FOUND 2
#define ERR_CRC       3

/* Buffers */
#define RX_BUF_SIZE             540U
#define UART_FRAME_MAX_LEN      540U
#define JSON_LINE_MAX           320U

/* Timing */
#define INPUT_CHANGE_TIME_MS    500U
#define DELIVERY_TIME_INTERVAL  60000U
#define TX_MAX_RETRY            3U
#define ACK_TIMEOUT_MS          2000U

/* DHT error reporting delay */
#define DHT_ERROR_ENABLE_DELAY_MS   5000U   // 5 detik setelah boot

/* SD status monitor*/
#define SD_STATUS_INTERVAL_MS   8000U

/* Rules */
#define RULE_MAX_PAYLOAD        240U
#define RULE_STR_MAX_LEN        32U

/* File names*/
#define LOG_FILE_NAME           "log.json"
#define STATUS_FILE_NAME        "status.json"

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* Sensor data */
static float     g_temp = 0.0f;
static float     g_hum  = 0.0f;
static char      g_debugBuf[384];
static uint32_t  g_bootMs = 0;

/* Input state */
static volatile uint32_t g_inputMaskStable = 0;  /* gabungan I1..I20 (native | addon<<12), dipakai rule engine & dig_in */
static volatile uint16_t g_nativeInputMask = 0;  /* komponen native GPIO saja (I1..I11), didebounce di App_BackgroundSensors */
static volatile uint8_t  g_addonInputMask  = 0;  /* komponen addon PCF8574 (I13..I20), diterima via FRAME_TYPE_ADDON, sudah didebounce di ESP32 */
static volatile uint8_t  g_ruleUpdated     = 0;

/* ADS1115 */
static int16_t  g_anRaw[NUM_ANALOG];
static uint8_t  g_anCache[NUM_ANALOG];
static uint32_t g_adsTick = 0;

/* UART RX - ESP32 */
static uint8_t  g_rxByte;
static uint8_t  g_frameBuf[RX_BUF_SIZE];
static uint16_t g_frameIdx      = 0;
static int16_t  g_expectedLen   = -1;

/* UART RX - RS485/Debug */
static uint8_t  g_rs485RxByte;
static uint8_t  g_rs485Buf[128];
static uint16_t g_rs485Idx = 0;

/* Application state */
static AppState_t g_appState = APP_STATE_SD_MOUNT;

/* SD Card */
static uint8_t  g_sd_mounted = 0;
static uint32_t g_nextSeq    = 1;

/* Flags */
volatile uint8_t g_io_changed_flag = 0;

/* ACK from ESP32 */
volatile uint8_t  g_ack_pending = 0;
volatile uint32_t g_ack_seq     = 0;

/* TX state machine */
static TxState_t   g_txState = TX_STATE_IDLE;
static TxContext_t g_txCtx;

/* Error flags */
static uint8_t g_err_rtc = 0;
static uint8_t g_err_dht = 0;
static uint8_t g_err_ads = 0;
static uint8_t g_err_sd  = 0;

/* Input mapping */
static const IO_PinDef_t g_inputs[NUM_INPUTS] =
{
    [IN_1]  = { INPUT_1_GPIO_Port,  INPUT_1_Pin  },
    [IN_2]  = { INPUT_2_GPIO_Port,  INPUT_2_Pin  },
    [IN_3]  = { INPUT_3_GPIO_Port,  INPUT_3_Pin  },
    [IN_4]  = { INPUT_4_GPIO_Port,  INPUT_4_Pin  },
    [IN_5]  = { INPUT_5_GPIO_Port,  INPUT_5_Pin  },
    [IN_6]  = { INPUT_6_GPIO_Port,  INPUT_6_Pin  },
    [IN_7]  = { INPUT_7_GPIO_Port,  INPUT_7_Pin  },
    [IN_8]  = { INPUT_8_GPIO_Port,  INPUT_8_Pin  },
    [IN_9]  = { INPUT_9_GPIO_Port,  INPUT_9_Pin  },
    [IN_10] = { INPUT_10_GPIO_Port, INPUT_10_Pin },
    [IN_11] = { INPUT_11_GPIO_Port, INPUT_11_Pin }
};

/* Output mapping */
static const IO_PinDef_t g_outputs[NUM_OUTPUTS] = {
    [OUT_1] = { OUTPUT_1_GPIO_Port, OUTPUT_1_Pin },
    [OUT_2] = { OUTPUT_2_GPIO_Port, OUTPUT_2_Pin },
    [OUT_3] = { OUTPUT_3_GPIO_Port, OUTPUT_3_Pin },
    [OUT_4] = { OUTPUT_4_GPIO_Port, OUTPUT_4_Pin }
};

/* Rules */
static RuleDef_t g_rules[NUM_OUTPUTS];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

/* Utility */
static void UART_Print(const char* str);

/* DHT22 */
void DWT_Delay_Init(void);
void delay_us(uint32_t us);
void DHT_SetOutput(void);
void DHT_SetInput(void);
uint8_t DHT_Start(void);
uint8_t DHT_ReadBit(void);
uint8_t DHT_ReadByte(void);
uint8_t DHT22_Read(float *temperature, float *humidity);

/* DS3231 RTC */
static uint8_t bcd2bin(uint8_t val);
static uint8_t bin2bcd(uint8_t val);
uint8_t DS3231_ReadTime(DS3231_TimeTypeDef *t);
uint8_t DS3231_SetTime(const DS3231_TimeTypeDef *t);
static uint8_t IsLeapYear(uint16_t year);
static uint32_t RTC_ToEpochSeconds(const DS3231_TimeTypeDef *t);
static void EpochToRTC(uint32_t epoch, DS3231_TimeTypeDef *t);

/* ADS1115 */
static int16_t ADS1115_ReadRaw(uint8_t ch);
static float ADS1115_ConvertRawToVolt(int16_t raw);
static void ADS_Task(void);
static uint8_t VoltageToPWM_0_255(float voltage);

/* CRC */
static uint16_t CRC16_Modbus(const uint8_t *data, uint16_t len);

/* TLV Encoder (for UART TX to ESP32) */
static size_t TLV_PutU8(uint8_t *buf, uint8_t tag, uint8_t val);
static size_t TLV_PutU16(uint8_t *buf, uint8_t tag, uint16_t val);
static size_t TLV_PutI16(uint8_t *buf, uint8_t tag, int16_t val);
static size_t TLV_PutU32(uint8_t *buf, uint8_t tag, uint32_t val);
static size_t TLV_PutU16Array4(uint8_t *buf, uint8_t tag, const uint16_t arr[4]);
static size_t TLV_EncodeMeasurement(const Measurement_t *m, uint8_t *out, size_t maxLen);
static void SendErrorTLV(uint8_t mid, uint8_t st, uint8_t ec);

/* TLV Parser (from ESP32) */
static uint32_t TLV_ReadU32BE(const uint8_t *p);

/* JSON Encoder/Parser (for SD Card log & debug) */
static size_t JSON_EncodeMeasurement(const Measurement_t *m, char *out, size_t maxLen);
static bool JSON_ParseSeqFromLine(const char *line, uint32_t *outSeq);

/* Rules & Frame RX */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
static void Rules_Reset(void);
static void ParseOneRuleString(const char *ruleStr);
static void ParseRulesTLV(const uint8_t *tlv, size_t len);
static void ParseRTCTLV(const uint8_t *tlv, size_t len);
static void ParseACKTLV(const uint8_t *tlv, size_t len);
static void ParseAddonTLV(const uint8_t *tlv, size_t len);
static void RecomputeCombinedInputMask(void);
static uint8_t EvalOutputState(int qIdx, uint32_t inputMask);
static void ApplyRules(uint32_t inputMask);
static void PrintRuleStatus(void);
static void OnFrameReceived(uint8_t *payload, uint16_t len, uint8_t frameType);

/* Logging / ACK helper */
static bool Log_RemoveSeq(uint32_t seqToRemove);
static bool SendMeasurementTLV(const Measurement_t *m);
static void App_HandleAck(void);

/* TX state machine */
static void TX_StartNew(const Measurement_t *m);
static void TX_Process(uint32_t now);

/* SD Card & Log (JSON format) */
static void Log_InitRuntime(void);
static bool SD_MountTask(void);
static bool Log_RecoverTask(void);
static bool Log_AppendJSON(const char *jsonLine);
static void SD_StatusTask(uint32_t now);

/* Application */
static void App_BackgroundSensors(uint32_t now);
static void App_HandleAcquisition(uint32_t now);
static void Acquire_FillMeasurement(Measurement_t *m, const DS3231_TimeTypeDef *rtc);

/* Debug */
static void Debug_ErrorJson(uint8_t mid, uint8_t st, uint8_t ec);
static void Debug_PrintMeasurementJSON(const Measurement_t *m, bool byIo);
static void Debug_DumpTLV(const char *prefix, const uint8_t *tlv, size_t len);

static void WD_Service(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ==================== UTILITY ==================== */
static void UART_Print(const char* str)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 100);
}

static inline void WD_SetSensor(void)
{
    wd.sensor = 1;
}

static inline void WD_SetAcquisition(void)
{
    wd.acquisition = 1;
}

static inline void WD_SetAck(void)
{
    wd.ack = 1;
}

static inline void WD_SetTX(void)
{
    wd.tx = 1;
}

static inline void WD_SetSD(void)
{
    wd.sd = 1;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  DWT_Delay_Init();
  g_bootMs = HAL_GetTick();   // simpan waktu boot untuk referensi DHT
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  UART_Print("\r\n");
  UART_Print("============================================\r\n");
  UART_Print("  STM32 IoT Node - TLV/JSON Hybrid v1.0\r\n");
  UART_Print("  UART Comms : TLV Binary\r\n");
  UART_Print("  SD Card    : JSON (Human-readable)\r\n");
  UART_Print("  Debug      : JSON (Human-readable)\r\n");
  UART_Print("============================================\r\n");

  Log_InitRuntime();
  g_appState = APP_STATE_SD_MOUNT;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      uint32_t now = HAL_GetTick();

      switch (g_appState)
      {
      case APP_STATE_SD_MOUNT:
          {
              uint8_t tries = 0;
              while (!g_sd_mounted && tries < 3) {
                  SD_MountTask();
                  HAL_Delay(200); // jeda 200ms antar percobaan awal
                  tries++;
              }
              g_appState = APP_STATE_LOG_RECOVER;
          }
          break;

      case APP_STATE_LOG_RECOVER:
          // Kalau SD sudah ter-mount, lakukan recovery; kalau tidak, skip saja
          if (g_sd_mounted) {
              Log_RecoverTask();
          }
          g_appState = APP_STATE_RUN;
          UART_Print("[APP] Entering RUN state\r\n");
          break;

      case APP_STATE_RUN:
          // Di RUN, kalau belum ada SD, coba mount berkala (user colok saat jalan)
          if (!g_sd_mounted) {
              SD_MountTask();
          }

          App_BackgroundSensors(now);
          WD_SetSensor();

          App_HandleAcquisition(now);
          WD_SetAcquisition();

          App_HandleAck();
          WD_SetAck();

          TX_Process(now);
          WD_SetTX();

          SD_StatusTask(now);
          WD_SetSD();

          WD_Service();

          break;

      default:
          g_appState = APP_STATE_SD_MOUNT;
          break;
      }

      HAL_Delay(1);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 999;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  HAL_UART_Receive_IT(&huart1, &g_rs485RxByte, 1);

  /* Priority lebih rendah dari USART2 (yg di-set 5,0) - RS485/debug
     tidak boleh starvasi link data kritis ke ESP32 saat dua interrupt
     datang berdekatan. Angka NVIC lebih besar = priority eksekusi
     lebih rendah di Cortex-M. */
  HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  	HAL_UART_Receive_IT(&huart2, &g_rxByte, 1);

  	HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
  	HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SUHU_SENSOR_GPIO_Port, SUHU_SENSOR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OUTPUT_2_Pin|OUTPUT_1_Pin|OUTPUT_3_Pin|OUTPUT_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : INPUT_11_Pin INPUT_3_Pin INPUT_4_Pin INPUT_5_Pin
                           INPUT_6_Pin */
  GPIO_InitStruct.Pin = INPUT_11_Pin|INPUT_3_Pin|INPUT_4_Pin|INPUT_5_Pin
                          |INPUT_6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SUHU_SENSOR_Pin */
  GPIO_InitStruct.Pin = SUHU_SENSOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SUHU_SENSOR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OUTPUT_2_Pin OUTPUT_1_Pin OUTPUT_3_Pin OUTPUT_4_Pin */
  GPIO_InitStruct.Pin = OUTPUT_2_Pin|OUTPUT_1_Pin|OUTPUT_3_Pin|OUTPUT_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : INPUT_1_Pin INPUT_2_Pin INPUT_7_Pin INPUT_8_Pin
                           INPUT_9_Pin INPUT_10_Pin */
  GPIO_InitStruct.Pin = INPUT_1_Pin|INPUT_2_Pin|INPUT_7_Pin|INPUT_8_Pin
                          |INPUT_9_Pin|INPUT_10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* ==================== DWT DELAY ==================== */
void DWT_Delay_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint32_t us)
{
    uint32_t cycles = us * (SystemCoreClock / 1000000);
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles);
}

/* ==================== DHT22 DRIVER ==================== */
void DHT_SetOutput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = SUHU_SENSOR_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SUHU_SENSOR_GPIO_Port, &GPIO_InitStruct);
}

void DHT_SetInput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = SUHU_SENSOR_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(SUHU_SENSOR_GPIO_Port, &GPIO_InitStruct);
}

uint8_t DHT_Start(void)
{
    uint32_t timeout;

    DHT_SetOutput();
    HAL_GPIO_WritePin(SUHU_SENSOR_GPIO_Port, SUHU_SENSOR_Pin, GPIO_PIN_RESET);
    HAL_Delay(2);
    HAL_GPIO_WritePin(SUHU_SENSOR_GPIO_Port, SUHU_SENSOR_Pin, GPIO_PIN_SET);
    delay_us(30);
    DHT_SetInput();

    timeout = 0;
    while (HAL_GPIO_ReadPin(SUHU_SENSOR_GPIO_Port, SUHU_SENSOR_Pin) == GPIO_PIN_SET)
    {
        delay_us(1);
        if (++timeout > 200) return 0;
    }

    timeout = 0;
    while (HAL_GPIO_ReadPin(SUHU_SENSOR_GPIO_Port, SUHU_SENSOR_Pin) == GPIO_PIN_RESET)
    {
        delay_us(1);
        if (++timeout > 200) return 0;
    }

    timeout = 0;
    while (HAL_GPIO_ReadPin(SUHU_SENSOR_GPIO_Port, SUHU_SENSOR_Pin) == GPIO_PIN_SET)
    {
        delay_us(1);
        if (++timeout > 200) return 0;
    }

    return 1;
}

uint8_t DHT_ReadBit(void)
{
    uint8_t bit = 0;
    uint32_t timeout = 0;

    // Tunggu naik dari LOW ke HIGH, tapi dengan timeout
    while (HAL_GPIO_ReadPin(SUHU_SENSOR_GPIO_Port, SUHU_SENSOR_Pin) == GPIO_PIN_RESET)
    {
        delay_us(1);
        if (++timeout > 200) {
            // timeout ~200 us, anggap bit = 0 dan keluar
            return 0;
        }
    }

    delay_us(40);
    if (HAL_GPIO_ReadPin(SUHU_SENSOR_GPIO_Port, SUHU_SENSOR_Pin) == GPIO_PIN_SET)
        bit = 1;

    // Tunggu turun kembali ke LOW, juga dengan timeout
    timeout = 0;
    while (HAL_GPIO_ReadPin(SUHU_SENSOR_GPIO_Port, SUHU_SENSOR_Pin) == GPIO_PIN_SET)
    {
        delay_us(1);
        if (++timeout > 200) {
            break;  // paksa keluar supaya tidak hang
        }
    }

    return bit;
}

uint8_t DHT_ReadByte(void)
{
    uint8_t i, data = 0;
    for (i = 0; i < 8; i++)
    {
        data <<= 1;
        data |= DHT_ReadBit();
    }
    return data;
}

uint8_t DHT22_Read(float *temperature, float *humidity)
{
    uint8_t data[5];

    if (!DHT_Start()) return 0;

    data[0] = DHT_ReadByte();
    data[1] = DHT_ReadByte();
    data[2] = DHT_ReadByte();
    data[3] = DHT_ReadByte();
    data[4] = DHT_ReadByte();

    uint8_t sum = data[0] + data[1] + data[2] + data[3];
    if (sum != data[4]) return 0;

    uint16_t rawHum  = (data[0] << 8) | data[1];
    int16_t  rawTemp = (data[2] << 8) | data[3];

    *humidity = rawHum / 10.0f;

    if (rawTemp & 0x8000)
    {
        rawTemp &= 0x7FFF;
        *temperature = -rawTemp / 10.0f;
    }
    else
    {
        *temperature = rawTemp / 10.0f;
    }

    if (*humidity < 0.0f || *humidity > 100.0f) return 0;
    if (*temperature < -40.0f || *temperature > 80.0f) return 0;

    return 1;
}

/* ==================== I2C RECOVERY ==================== */
static void I2C1_RecoverBus(void)
{
    HAL_I2C_DeInit(&hi2c1);
    MX_I2C1_Init();    // re-init peripheral
}

static void I2C2_RecoverBus(void)
{
    HAL_I2C_DeInit(&hi2c2);
    MX_I2C2_Init();    // re-init peripheral
}

/* Map HAL I2C status -> error code */
static uint8_t I2C_StatusToErrCode(HAL_StatusTypeDef st)
{
    if (st == HAL_TIMEOUT) {
        return ERR_TIMEOUT;
    }
    // HAL_ERROR atau HAL_BUSY -> treat as NOT_FOUND (device NACK / hilang)
    return ERR_NOT_FOUND;
}

/* ==================== DS3231 RTC ==================== */
static uint8_t bcd2bin(uint8_t val)
{
    return (uint8_t)((val >> 4) * 10 + (val & 0x0F));
}

static uint8_t bin2bcd(uint8_t val)
{
    return (uint8_t)(((val / 10) << 4) | (val % 10));
}

uint8_t DS3231_ReadTime(DS3231_TimeTypeDef *t)
{
    uint8_t buf[7];
    HAL_StatusTypeDef st;

    st = HAL_I2C_Mem_Read(&hi2c1, DS3231_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT,
                          buf, 7, 100);
    if (st != HAL_OK) {
        // Coba recovery I2C1 sekali
        I2C1_RecoverBus();
        st = HAL_I2C_Mem_Read(&hi2c1, DS3231_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT,
                              buf, 7, 100);
        if (st != HAL_OK) {
            // Biarkan App_HandleAcquisition yang meng-set Debug_ErrorJson/SendErrorTLV
            return 0;
        }
    }

    t->seconds   = bcd2bin(buf[0] & 0x7F);
    t->minutes   = bcd2bin(buf[1] & 0x7F);
    t->hours     = bcd2bin(buf[2] & 0x3F);
    t->dayOfWeek = bcd2bin(buf[3] & 0x07);
    t->day       = bcd2bin(buf[4] & 0x3F);
    t->month     = bcd2bin(buf[5] & 0x1F);
    t->year      = 2000 + bcd2bin(buf[6]);

    return 1;
}

uint8_t DS3231_SetTime(const DS3231_TimeTypeDef *t)
{
    uint8_t buf[7];

    buf[0] = bin2bcd(t->seconds);
    buf[1] = bin2bcd(t->minutes);
    buf[2] = bin2bcd(t->hours);
    buf[3] = bin2bcd(t->dayOfWeek);
    buf[4] = bin2bcd(t->day);
    buf[5] = bin2bcd(t->month);
    buf[6] = bin2bcd((uint8_t)(t->year - 2000));

    if (HAL_I2C_Mem_Write(&hi2c1, DS3231_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT,
                          buf, 7, 100) != HAL_OK)
    {
        return 0;
    }
    return 1;
}

static uint8_t IsLeapYear(uint16_t year)
{
    return (year % 4U == 0U);
}

static uint32_t RTC_ToEpochSeconds(const DS3231_TimeTypeDef *t)
{
    static const uint8_t days_in_month[12] =
        {31,28,31,30,31,30,31,31,30,31,30,31};

    uint32_t days = 0;

    for (uint16_t y = 2000; y < t->year; y++) {
        days += 365U + (IsLeapYear(y) ? 1U : 0U);
    }

    for (uint8_t m = 1; m < t->month; m++) {
        days += days_in_month[m-1];
        if (m == 2 && IsLeapYear(t->year)) {
            days += 1U;
        }
    }

    days += (uint32_t)(t->day - 1U);

    uint32_t seconds =
        days * 86400UL +
        (uint32_t)t->hours   * 3600UL +
        (uint32_t)t->minutes * 60UL  +
        (uint32_t)t->seconds;

    return seconds;
}

static void EpochToRTC(uint32_t epoch, DS3231_TimeTypeDef *t)
{
    static const uint8_t daysInMonth[12] =
        {31,28,31,30,31,30,31,31,30,31,30,31};

    uint32_t days = epoch / 86400;
    uint32_t secs = epoch % 86400;

    t->hours   = secs / 3600;
    t->minutes = (secs % 3600) / 60;
    t->seconds = secs % 60;

    t->year = 2000;
    while (1) {
        uint16_t daysInYear = 365 + (IsLeapYear(t->year) ? 1 : 0);
        if (days < daysInYear) break;
        days -= daysInYear;
        t->year++;
    }

    t->month = 1;
    while (1) {
        uint8_t dim = daysInMonth[t->month - 1];
        if (t->month == 2 && IsLeapYear(t->year)) dim = 29;
        if (days < dim) break;
        days -= dim;
        t->month++;
    }

    t->day = days + 1;
    t->dayOfWeek = 1;
}

/* ==================== ADS1115 DRIVER ==================== */
static int16_t ADS1115_ReadRaw(uint8_t ch)
{
    uint8_t  cfg[3];
    uint8_t  rx[2];
    uint16_t config;
    HAL_StatusTypeDef st;
    uint8_t ec;   // error code

    config  = 0x8000U;
    config |= (uint16_t)((0x04U + ch) << 12);
    config |= 0x0200U;
    config |= 0x0100U;
    config |= 0x0080U;
    config |= 0x0003U;

    cfg[0] = 0x01;
    cfg[1] = (uint8_t)(config >> 8);
    cfg[2] = (uint8_t)(config & 0xFF);

    // TX config
    st = HAL_I2C_Master_Transmit(&hi2c2, ADS1115_ADDR, cfg, 3, 100);
    if (st != HAL_OK) {
        I2C2_RecoverBus();
        st = HAL_I2C_Master_Transmit(&hi2c2, ADS1115_ADDR, cfg, 3, 100);
    }
    if (st != HAL_OK) {
        if (!g_err_ads) {
            g_err_ads = 1;
            ec = I2C_StatusToErrCode(st);   // ERR_TIMEOUT atau ERR_NOT_FOUND
            Debug_ErrorJson(MOD_ADS, ST_ERR, ec);
            SendErrorTLV(MOD_ADS, ST_ERR, ec);
        }
        return 0;
    }

    HAL_Delay(8);

    // Set register pointer ke 0x00
    cfg[0] = 0x00;
    st = HAL_I2C_Master_Transmit(&hi2c2, ADS1115_ADDR, cfg, 1, 100);
    if (st != HAL_OK) {
        I2C2_RecoverBus();
        st = HAL_I2C_Master_Transmit(&hi2c2, ADS1115_ADDR, cfg, 1, 100);
    }
    if (st != HAL_OK) {
        if (!g_err_ads) {
            g_err_ads = 1;
            ec = I2C_StatusToErrCode(st);
            Debug_ErrorJson(MOD_ADS, ST_ERR, ec);
            SendErrorTLV(MOD_ADS, ST_ERR, ec);
        }
        return 0;
    }

    // Read conversion
    st = HAL_I2C_Master_Receive(&hi2c2, ADS1115_ADDR, rx, 2, 100);
    if (st != HAL_OK) {
        I2C2_RecoverBus();
        st = HAL_I2C_Master_Receive(&hi2c2, ADS1115_ADDR, rx, 2, 100);
    }
    if (st != HAL_OK) {
        if (!g_err_ads) {
            g_err_ads = 1;
            ec = I2C_StatusToErrCode(st);
            Debug_ErrorJson(MOD_ADS, ST_ERR, ec);
            SendErrorTLV(MOD_ADS, ST_ERR, ec);
        }
        return 0;
    }

    // Sampai sini semua OK, kalau sebelumnya error, kirim status OK
    if (g_err_ads) {
        g_err_ads = 0;
        Debug_ErrorJson(MOD_ADS, ST_OK, ERR_NONE);
        SendErrorTLV(MOD_ADS, ST_OK, ERR_NONE);
    }

    return (int16_t)((rx[0] << 8) | rx[1]);
}

static uint8_t VoltageToPWM_0_255(float voltage)
{
    // Pastikan dalam range 0 .. ADC_INPUT_MAX_VOLT
    if (voltage < 0.0f)            voltage = 0.0f;
    if (voltage > ADC_INPUT_MAX_VOLT) voltage = ADC_INPUT_MAX_VOLT;

    // Mapping linier 0..ADC_INPUT_MAX_VOLT -> 0..255
    float scale = voltage / ADC_INPUT_MAX_VOLT;
    uint8_t pwm = (uint8_t)(scale * 255.0f + 0.5f);  // +0.5 untuk pembulatan

    return pwm;
}

static float ADS1115_ConvertRawToVolt(int16_t raw)
{
    float v = (float)raw * ADS_FS_VOLT / 32768.0f;

    // Clamp ke 0 .. ADC_INPUT_MAX_VOLT
    if (v < 0.0f)            v = 0.0f;
    if (v > ADC_INPUT_MAX_VOLT) v = ADC_INPUT_MAX_VOLT;

    return v;
}

static void ADS_Task(void)
{
    uint32_t now = HAL_GetTick();
    if (now - g_adsTick < ADS_READ_PERIOD_MS)
        return;

    g_adsTick = now;

    for (uint8_t ch = 0; ch < NUM_ANALOG; ch++)
    {
        g_anRaw[ch]   = ADS1115_ReadRaw(ch);
        float volt    = ADS1115_ConvertRawToVolt(g_anRaw[ch]);
        g_anCache[ch] = VoltageToPWM_0_255(volt);
    }
}

/* ==================== CRC16 MODBUS ==================== */
static uint16_t CRC16_Modbus(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

/* ==================== TLV ENCODER (for UART) ==================== */
static size_t TLV_PutU8(uint8_t *buf, uint8_t tag, uint8_t val)
{
    buf[0] = tag;
    buf[1] = 1;
    buf[2] = val;
    return 3;
}

static size_t TLV_PutU16(uint8_t *buf, uint8_t tag, uint16_t val)
{
    buf[0] = tag;
    buf[1] = 2;
    buf[2] = (uint8_t)(val >> 8);
    buf[3] = (uint8_t)(val & 0xFF);
    return 4;
}

static size_t TLV_PutI16(uint8_t *buf, uint8_t tag, int16_t val)
{
    buf[0] = tag;
    buf[1] = 2;
    buf[2] = (uint8_t)((uint16_t)val >> 8);
    buf[3] = (uint8_t)((uint16_t)val & 0xFF);
    return 4;
}

static size_t TLV_PutU32(uint8_t *buf, uint8_t tag, uint32_t val)
{
    buf[0] = tag;
    buf[1] = 4;
    buf[2] = (uint8_t)(val >> 24);
    buf[3] = (uint8_t)(val >> 16);
    buf[4] = (uint8_t)(val >> 8);
    buf[5] = (uint8_t)(val & 0xFF);
    return 6;
}

static size_t TLV_PutU16Array4(uint8_t *buf, uint8_t tag, const uint16_t arr[4])
{
    buf[0] = tag;
    buf[1] = 8;
    for (int i = 0; i < 4; i++) {
        buf[2 + i*2]     = (uint8_t)(arr[i] >> 8);
        buf[2 + i*2 + 1] = (uint8_t)(arr[i] & 0xFF);
    }
    return 10;
}

static size_t TLV_EncodeMeasurement(const Measurement_t *m, uint8_t *out, size_t maxLen)
{
    if (maxLen < 48) return 0;

    size_t idx = 0;

    // Calculate timestamp epoch from RTC
    uint32_t timestamp = RTC_ToEpochSeconds(&m->rtc);

    idx += TLV_PutU32(&out[idx], TLV_TAG_SEQ, m->seq);
    idx += TLV_PutU32(&out[idx], TLV_TAG_TIMESTAMP, timestamp);
    idx += TLV_PutI16(&out[idx], TLV_TAG_TEMP, m->temp);
    idx += TLV_PutI16(&out[idx], TLV_TAG_HUM, m->hum);
    idx += TLV_PutU32(&out[idx], TLV_TAG_DIG_IN, m->dig_in);  // sekarang 4 byte (I1..I20), lihat catatan TLV_TAG_DIG_IN
    idx += TLV_PutU16Array4(&out[idx], TLV_TAG_AN_IN, m->an_in);
    idx += TLV_PutU8(&out[idx], TLV_TAG_Q, m->q);

    return idx;
}

static void SendErrorTLV(uint8_t mid, uint8_t st, uint8_t ec)
{
    uint8_t payload[16];
    size_t idx = 0;

    // mid -> TLV_TAG_ERR_MODULE
    payload[idx++] = TLV_TAG_ERR_MODULE;
    payload[idx++] = 1;
    payload[idx++] = mid;

    // st -> TLV_TAG_ERR_ACTIVE (pakai tag lama untuk "status")
    payload[idx++] = TLV_TAG_ERR_ACTIVE;
    payload[idx++] = 1;
    payload[idx++] = st;

    // ec -> TLV_TAG_ERR_CODE
    payload[idx++] = TLV_TAG_ERR_CODE;
    payload[idx++] = 1;
    payload[idx++] = ec;

    uint8_t frame[32];
    uint16_t fidx = 0;

    frame[fidx++] = TLV_SOF;
    frame[fidx++] = FRAME_TYPE_ERROR;
    frame[fidx++] = (idx >> 8) & 0xFF;
    frame[fidx++] = idx & 0xFF;

    memcpy(&frame[fidx], payload, idx);
    fidx += idx;

    uint16_t crc = CRC16_Modbus(frame, fidx);
    frame[fidx++] = (crc >> 8) & 0xFF;
    frame[fidx++] = crc & 0xFF;

    HAL_UART_Transmit(&huart2, frame, fidx, 100);
}

static bool SendMeasurementTLV(const Measurement_t *m)
{
    uint8_t tlvPayload[64];
    size_t tlvLen = TLV_EncodeMeasurement(m, tlvPayload, sizeof(tlvPayload));
    if (tlvLen == 0) {
        UART_Print("[TX] TLV encode failed\r\n");
        return false;
    }

    uint8_t frame[UART_FRAME_MAX_LEN];
    uint16_t idx = 0;

    frame[idx++] = TLV_SOF;
    frame[idx++] = FRAME_TYPE_DATA;
    frame[idx++] = (tlvLen >> 8) & 0xFF;
    frame[idx++] = tlvLen & 0xFF;

    memcpy(&frame[idx], tlvPayload, tlvLen);
    idx += tlvLen;

    uint16_t crc = CRC16_Modbus(frame, idx);
    frame[idx++] = (crc >> 8) & 0xFF;
    frame[idx++] = crc & 0xFF;

    if (HAL_UART_Transmit(&huart2, frame, idx, 200) != HAL_OK) {
        UART_Print("[TX] UART transmit failed\r\n");
        return false;
    }

    snprintf(g_debugBuf, sizeof(g_debugBuf),
             "[TX] Sent TLV seq=%lu len=%u\r\n",
             (unsigned long)m->seq, (unsigned)tlvLen);
    UART_Print(g_debugBuf);

    return true;
}

/* ==================== TLV PARSER (from ESP32) ==================== */
static uint32_t TLV_ReadU32BE(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) |
           ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  |
           p[3];
}

/* ==================== JSON ENCODER (for SD & Debug) ==================== */
static size_t JSON_EncodeMeasurement(const Measurement_t *m, char *out, size_t maxLen)
{
    char ts[32];
    char digArr[64];
    char anArr[32];
    char qArr[24];
    char *p;
    int n;

    // 1) Build timestamp string "YYYY-MM-DDTHH:MM:SSZ"
    snprintf(ts, sizeof(ts), "%04u-%02u-%02uT%02u:%02u:%02uZ",
             m->rtc.year, m->rtc.month, m->rtc.day,
             m->rtc.hours, m->rtc.minutes, m->rtc.seconds);

    // 2) Build dig_in array string [1,0,1,0,...]
    p = digArr;
    *p++ = '[';
    for (uint8_t i = 0; i < NUM_INPUTS_TOTAL; i++) {
        uint8_t bit = (m->dig_in & (1U << i)) ? 1 : 0;
        n = snprintf(p, sizeof(digArr) - (p - digArr), "%u%s",
                     bit, (i < NUM_INPUTS_TOTAL - 1) ? "," : "");
        if (n > 0) p += n;
    }
    *p++ = ']';
    *p = '\0';

    // 3) Build an_in array string [255,255,255,255]
    snprintf(anArr, sizeof(anArr), "[%u,%u,%u,%u]",
             m->an_in[0], m->an_in[1], m->an_in[2], m->an_in[3]);

    // 4) Build q array string [0,1,0,1]
    p = qArr;
    *p++ = '[';
    for (uint8_t i = 0; i < NUM_OUTPUTS; i++) {
        uint8_t bit = (m->q & (1U << i)) ? 1 : 0;
        n = snprintf(p, sizeof(qArr) - (p - qArr), "%u%s",
                     bit, (i < NUM_OUTPUTS - 1) ? "," : "");
        if (n > 0) p += n;
    }
    *p++ = ']';
    *p = '\0';

    // 5) Build complete JSON
    int len = snprintf(out, maxLen,
        "{\"timestamp\":\"%s\","
        "\"env\":{\"temp\":%.1f,\"hum\":%.1f},"
        "\"dig_in\":%s,"
        "\"an_in\":%s,"
        "\"q\":%s,"
        "\"seq\":%lu}\n",
        ts,
        m->temp / 10.0f,
        m->hum / 10.0f,
        digArr,
        anArr,
        qArr,
        (unsigned long)m->seq
    );

    if (len < 0 || (size_t)len >= maxLen) return 0;
    return (size_t)len;
}

static bool JSON_ParseSeqFromLine(const char *line, uint32_t *outSeq)
{
    const char *p = strstr(line, "\"seq\":");
    if (!p) return false;

    p += 6;
    *outSeq = (uint32_t)strtoul(p, NULL, 10);
    return (*outSeq > 0);
}

/* ==================== UART CALLBACKS ==================== */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2)
    {
        uint8_t b = g_rxByte;

        if (g_frameIdx == 0)
        {
            if (b == TLV_SOF)
            {
                g_frameBuf[0] = b;
                g_frameIdx    = 1;
                g_expectedLen = -1;
            }
        }
        else
        {
            if (g_frameIdx < RX_BUF_SIZE)
            {
                g_frameBuf[g_frameIdx++] = b;
            }
            else
            {
                g_frameIdx    = 0;
                g_expectedLen = -1;
            }

            // After SOF + TYPE + LEN_H + LEN_L (4 bytes)
            if (g_frameIdx == 4)
            {
                uint16_t payloadLen = ((uint16_t)g_frameBuf[2] << 8) | g_frameBuf[3];
                if (payloadLen > RULE_MAX_PAYLOAD)
                {
                    g_frameIdx    = 0;
                    g_expectedLen = -1;
                }
                else
                {
                    g_expectedLen = 4 + payloadLen + 2;
                }
            }

            if (g_expectedLen > 0 && g_frameIdx == g_expectedLen)
            {
                uint16_t payloadLen = ((uint16_t)g_frameBuf[2] << 8) | g_frameBuf[3];

                uint16_t crcRecv = ((uint16_t)g_frameBuf[g_frameIdx-2] << 8) |
                                    g_frameBuf[g_frameIdx-1];
                uint16_t crcCalc = CRC16_Modbus(g_frameBuf, g_frameIdx-2);

                if (crcRecv == crcCalc)
                {
                    uint8_t frameType = g_frameBuf[1];
                    OnFrameReceived(&g_frameBuf[4], payloadLen, frameType);
                }

                g_frameIdx    = 0;
                g_expectedLen = -1;
            }
        }

        HAL_UART_Receive_IT(&huart2, &g_rxByte, 1);
    }

    if (huart == &huart1)
    {
        uint8_t b = g_rs485RxByte;

        if (g_rs485Idx < sizeof(g_rs485Buf) - 1)
        {
            g_rs485Buf[g_rs485Idx++] = b;
        }

        if (b == '\n')
        {
            g_rs485Buf[g_rs485Idx] = '\0';
            g_rs485Idx = 0;
        }

        HAL_UART_Receive_IT(&huart1, &g_rs485RxByte, 1);
    }
}

/* ==================== RULES ==================== */
static void Rules_Reset(void)
{
    for (int q = 0; q < NUM_OUTPUTS; q++)
    {
        g_rules[q].logic      = RULE_LOGIC_OFF;
        g_rules[q].inputCount = 0;
        memset(g_rules[q].inputs, 0, sizeof(g_rules[q].inputs));
    }
}

static void ParseOneRuleString(const char *ruleStr)
{
    if (ruleStr == NULL || ruleStr[0] != 'Q') return;

    int qNum = atoi(&ruleStr[1]);
    if (qNum < 1 || qNum > (int)NUM_OUTPUTS) return;
    int qIdx = qNum - 1;

    const char *p = strchr(ruleStr, ':');
    if (!p) return;
    p++;

    const char *p2 = strchr(p, ':');
    if (!p2) return;

    char logicStr[8] = {0};
    size_t logicLen = (size_t)(p2 - p);
    if (logicLen >= sizeof(logicStr))
        logicLen = sizeof(logicStr) - 1;
    memcpy(logicStr, p, logicLen);
    logicStr[logicLen] = '\0';

    RuleLogic_t logic = RULE_LOGIC_OFF;
    if      (strcmp(logicStr, "OR")  == 0) logic = RULE_LOGIC_OR;
    else if (strcmp(logicStr, "AND") == 0) logic = RULE_LOGIC_AND;
    else if (strcmp(logicStr, "NOT") == 0) logic = RULE_LOGIC_NOT;

    g_rules[qIdx].logic      = logic;
    g_rules[qIdx].inputCount = 0;

    const char *r = p2 + 1;
    while (*r != '\0')
    {
        if (*r == 'I')
        {
            int inNum = atoi(r + 1);
            /* I1..I11 = native GPIO, I12 = reserved (belum ada GPIO fisik),
             * I13..I20 = addon PCF8574 (lihat NUM_INPUTS_TOTAL) */
            if (inNum >= 1 && inNum <= (int)NUM_INPUTS_TOTAL)
            {
                if (g_rules[qIdx].inputCount < NUM_INPUTS_TOTAL)
                {
                    g_rules[qIdx].inputs[g_rules[qIdx].inputCount++] = (uint8_t)inNum;
                }
            }
            while (*r && *r != ',' && *r != '\0') r++;
        }
        else
        {
            r++;
        }
    }
}

static void ParseRulesTLV(const uint8_t *tlv, size_t len)
{
    Rules_Reset();

    size_t i = 0;
    while (i + 2 <= len) {
        uint8_t tag = tlv[i];
        uint8_t l   = tlv[i + 1];

        if (i + 2 + l > len) break;

        if (tag == TAG_RULE) {
            char ruleStr[RULE_STR_MAX_LEN];
            size_t copyLen = (l < RULE_STR_MAX_LEN - 1) ? l : (RULE_STR_MAX_LEN - 1);
            memcpy(ruleStr, &tlv[i + 2], copyLen);
            ruleStr[copyLen] = '\0';
            ParseOneRuleString(ruleStr);
        }

        i += 2 + l;
    }

    g_ruleUpdated = 1;
}

static void ParseRTCTLV(const uint8_t *tlv, size_t len)
{
    size_t i = 0;
    while (i + 2 <= len) {
        uint8_t tag = tlv[i];
        uint8_t l   = tlv[i + 1];

        if (i + 2 + l > len) break;

        if (tag == TAG_RTC_TS && l == 4) {
            uint32_t epoch = TLV_ReadU32BE(&tlv[i + 2]);
            DS3231_TimeTypeDef t;
            EpochToRTC(epoch, &t);

            if (DS3231_SetTime(&t)) {
                UART_Print("[RTC] Set from ESP32 OK\r\n");
            } else {
                UART_Print("[RTC] Set FAILED\r\n");
            }
        }

        i += 2 + l;
    }
}

static void ParseACKTLV(const uint8_t *tlv, size_t len)
{
    size_t i = 0;
    while (i + 2 <= len) {
        uint8_t tag = tlv[i];
        uint8_t l   = tlv[i + 1];

        if (i + 2 + l > len) break;

        if (tag == TLV_TAG_SEQ && l == 4) {
            g_ack_seq = TLV_ReadU32BE(&tlv[i + 2]);
            g_ack_pending = 1;

            snprintf(g_debugBuf, sizeof(g_debugBuf),
                     "[ACK] Received seq=%lu\r\n", (unsigned long)g_ack_seq);
            UART_Print(g_debugBuf);
        }

        i += 2 + l;
    }
}

static void RecomputeCombinedInputMask(void)
{
    uint32_t combined = (uint32_t)g_nativeInputMask |
                         ((uint32_t)g_addonInputMask << ADDON_INPUT_BIT_OFFSET);

    if (combined != g_inputMaskStable)
    {
        g_inputMaskStable = combined;
        g_io_changed_flag = 1;
    }
}

static void ParseAddonTLV(const uint8_t *tlv, size_t len)
{
    size_t i = 0;
    while (i + 2 <= len)
    {
        uint8_t tag = tlv[i];
        uint8_t l   = tlv[i + 1];

        if (i + 2 + l > len) break;

        if (tag == TAG_ADDON_MASK && l == 1)
        {
            uint8_t newAddonMask = tlv[i + 2];
            if (newAddonMask != g_addonInputMask)
            {
                g_addonInputMask = newAddonMask;
                RecomputeCombinedInputMask();

                snprintf(g_debugBuf, sizeof(g_debugBuf),
                         "[ADDON] mask updated from ESP32: 0x%02X\r\n",
                         newAddonMask);
                UART_Print(g_debugBuf);
            }
        }

        i += 2 + l;
    }
}

static uint8_t EvalOutputState(int qIdx, uint32_t inputMask)
{
    RuleDef_t *r = &g_rules[qIdx];

    if (r->logic == RULE_LOGIC_OFF || r->inputCount == 0)
        return 0;

    if (r->logic == RULE_LOGIC_NOT)
    {
        uint8_t in = r->inputs[0];
        uint8_t state = (inputMask & (1U << (in - 1))) ? 1 : 0;
        return !state;
    }

    if (r->logic == RULE_LOGIC_OR)
    {
        for (uint8_t i = 0; i < r->inputCount; i++)
        {
            uint8_t in = r->inputs[i];
            if (inputMask & (1U << (in - 1)))
                return 1;
        }
        return 0;
    }

    if (r->logic == RULE_LOGIC_AND)
    {
        for (uint8_t i = 0; i < r->inputCount; i++)
        {
            uint8_t in = r->inputs[i];
            if (!(inputMask & (1U << (in - 1))))
                return 0;
        }
        return 1;
    }

    return 0;
}

static void ApplyRules(uint32_t inputMask)
{
    for (int q = 0; q < NUM_OUTPUTS; q++)
    {
        uint8_t on = EvalOutputState(q, inputMask);
        HAL_GPIO_WritePin(g_outputs[q].Port, g_outputs[q].Pin,
                          on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

static void PrintRuleStatus(void)
{
    UART_Print("[RULE] Current configuration:\r\n");

    for (int q = 0; q < NUM_OUTPUTS; q++)
    {
        RuleDef_t *r = &g_rules[q];

        const char *logicName = "OFF";
        if      (r->logic == RULE_LOGIC_OR)  logicName = "OR";
        else if (r->logic == RULE_LOGIC_AND) logicName = "AND";
        else if (r->logic == RULE_LOGIC_NOT) logicName = "NOT";

        int len = snprintf(g_debugBuf, sizeof(g_debugBuf),
                           "  Q%d: %s (", q + 1, logicName);

        for (uint8_t i = 0; i < r->inputCount; i++)
        {
            len += snprintf(g_debugBuf + len, sizeof(g_debugBuf) - len,
                            "I%u%s", r->inputs[i],
                            (i == r->inputCount - 1) ? "" : ",");
        }
        snprintf(g_debugBuf + len, sizeof(g_debugBuf) - len, ")\r\n");
        UART_Print(g_debugBuf);
    }
}

static void OnFrameReceived(uint8_t *payload, uint16_t len, uint8_t frameType)
{
    snprintf(g_debugBuf, sizeof(g_debugBuf),
             "[RX] Frame type=0x%02X len=%u\r\n", frameType, len);
    UART_Print(g_debugBuf);

    switch (frameType) {
        case FRAME_TYPE_RULES:
//            Debug_DumpTLV("[RX][RULES]", payload, len);
            ParseRulesTLV(payload, len);
            break;

        case FRAME_TYPE_RTC:
//            Debug_DumpTLV("[RX][RTC]", payload, len);
            ParseRTCTLV(payload, len);
            break;

        case FRAME_TYPE_ACK:
//            Debug_DumpTLV("[RX][ACK]", payload, len);
            ParseACKTLV(payload, len);
            break;

        case FRAME_TYPE_ADDON:
//            Debug_DumpTLV("[RX][ADDON]", payload, len);
            ParseAddonTLV(payload, len);
            break;

        default:
            UART_Print("[RX] Unknown frame type\r\n");
            break;
    }
}

/* ==================== SD CARD & LOG (JSON format) ==================== */
static void Log_InitRuntime(void)
{
    g_sd_mounted = 0;
    g_nextSeq    = 1;
}

static bool SD_MountTask(void)
{
    static uint32_t lastTryMs   = 0;
    static FRESULT  lastFr      = FR_OK;

    if (g_sd_mounted) return true;

    uint32_t now = HAL_GetTick();
    if ((now - lastTryMs) < 1000U) {   // coba tiap 1 detik
        return false;
    }
    lastTryMs = now;

    // Paksa init ulang disk (akan memanggil SD_disk_initialize)
    disk_initialize(0);

    FRESULT fr = f_mount(&USERFatFS, (TCHAR const*)USERPath, 1);
    if (fr == FR_OK) {
        g_sd_mounted = 1;
        lastFr       = fr;

        if (g_err_sd) {
            g_err_sd = 0;
            Debug_ErrorJson(MOD_SD, ST_OK, ERR_NONE);
            SendErrorTLV(MOD_SD, ST_OK, ERR_NONE);
        }
        UART_Print("[SD] Mounted OK\r\n");
        return true;
    } else {
        if (!g_err_sd) {
            g_err_sd = 1;
            Debug_ErrorJson(MOD_SD, ST_ERR, ERR_NOT_FOUND);  // kartu / filesystem tidak ditemukan
            SendErrorTLV(MOD_SD, ST_ERR, ERR_NOT_FOUND);
        }

        // Hanya print kalau kode error berubah
        if (fr != lastFr) {
            lastFr = fr;
            snprintf(g_debugBuf, sizeof(g_debugBuf),
                     "[SD] Mount ERR=%d\r\n", fr);
            UART_Print(g_debugBuf);
        }
    }
    return false;
}

static bool Log_RecoverTask(void)
{
    static uint8_t done = 0;
    if (done) return true;
    if (!g_sd_mounted) return false;

    FIL file;
    FRESULT fr;

    // Pastikan log.json ada
    fr = f_open(&file, LOG_FILE_NAME, FA_READ | FA_OPEN_EXISTING);
    if (fr != FR_OK) {
        // Buat file kosong
        fr = f_open(&file, LOG_FILE_NAME, FA_WRITE | FA_CREATE_ALWAYS);
        if (fr != FR_OK) {
            UART_Print("[LOG] create log.json failed\r\n");
            return false;
        }
        f_close(&file);
        g_nextSeq = 1;
        done = 1;
        return true;
    }

    // Scan untuk mencari seq maksimum
    char line[JSON_LINE_MAX];
    uint32_t seq;
    uint32_t maxSeq = 0;

    while (f_gets(line, sizeof(line), &file) != NULL) {
        if (JSON_ParseSeqFromLine(line, &seq)) {
            if (seq > maxSeq) maxSeq = seq;
        }
    }

    f_close(&file);

    g_nextSeq = (maxSeq == 0) ? 1 : (maxSeq + 1);
    done = 1;

    snprintf(g_debugBuf, sizeof(g_debugBuf),
             "[LOG] Recovery done, nextSeq=%lu\r\n", (unsigned long)g_nextSeq);
    UART_Print(g_debugBuf);

    return true;
}

static bool Log_AppendJSON(const char *jsonLine)
{
    FIL f;
    FRESULT fr = f_open(&f, LOG_FILE_NAME, FA_WRITE | FA_OPEN_APPEND);
    if (fr != FR_OK) {
        UART_Print("[LOG] Open failed\r\n");
        if (!g_err_sd) {
            g_err_sd = 1;
            Debug_ErrorJson(MOD_SD, ST_ERR, ERR_NOT_FOUND);
            SendErrorTLV(MOD_SD, ST_ERR, ERR_NOT_FOUND);
        }
        f_close(&f);  // jaga-jaga
        f_mount(NULL, (TCHAR const*)USERPath, 0); // unmount
        g_sd_mounted = 0;
        return false;
    }

    UINT bw;
    size_t len = strlen(jsonLine);
    fr = f_write(&f, jsonLine, len, &bw);
    f_sync(&f);
    f_close(&f);

    if (fr != FR_OK || bw != len) {
        UART_Print("[LOG] Write failed\r\n");
        if (!g_err_sd) {
            g_err_sd = 1;
            Debug_ErrorJson(MOD_SD, ST_ERR, ERR_CRC);   // treat as data error
            SendErrorTLV(MOD_SD, ST_ERR, ERR_CRC);
        }
        f_close(&f);
        f_mount(NULL, (TCHAR const*)USERPath, 0);
        g_sd_mounted = 0;
        return false;
    }

    if (g_err_sd) {
        g_err_sd = 0;
        Debug_ErrorJson(MOD_SD, ST_OK, ERR_NONE);
        SendErrorTLV(MOD_SD, ST_OK, ERR_NONE);
    }

    return true;
}

static bool Log_RemoveSeq(uint32_t seqToRemove)
{
    FIL f;
    FRESULT fr = f_open(&f, LOG_FILE_NAME, FA_READ | FA_WRITE);
    if (fr != FR_OK) {
        UART_Print("[LOG] Open for remove failed\r\n");
        f_mount(NULL, (TCHAR const*)USERPath, 0);
        g_sd_mounted = 0;
        return false;
    }

    char line[JSON_LINE_MAX];
    uint32_t seq;
    UINT bw;
    DWORD readPos  = 0;
    DWORD writePos = 0;
    bool removed   = false;

    while (1) {
        // Posisikan ke offset baca
        fr = f_lseek(&f, readPos);
        if (fr != FR_OK) break;

        // Baca satu baris
        if (f_gets(line, sizeof(line), &f) == NULL) {
            // EOF
            break;
        }

        // Update posisi baca ke setelah baris ini
        readPos = f_tell(&f);

        // Ambil seq
        if (JSON_ParseSeqFromLine(line, &seq) && seq == seqToRemove) {
            // Skip baris ini (tidak ditulis kembali)
            removed = true;
            continue;
        }

        // Tulis ke posisi writePos
        fr = f_lseek(&f, writePos);
        if (fr != FR_OK) break;

        size_t len = strlen(line);
        fr = f_write(&f, line, len, &bw);
        if (fr != FR_OK || bw != len) {
            UART_Print("[LOG] write during remove failed\r\n");
            break;
        }

        writePos += bw;
    }

    // Jika ada baris dihapus, truncate file ke writePos
    if (removed) {
        fr = f_lseek(&f, writePos);
        if (fr == FR_OK) {
            f_truncate(&f);   // truncate di posisi writePos
        }
    }

    f_sync(&f);
    f_close(&f);

    return removed;
}

static void SD_StatusTask(uint32_t now)
{
    static uint32_t lastCheckMs = 0;

    // Jalan tiap SD_STATUS_INTERVAL_MS (default 3000 ms)
    if ((now - lastCheckMs) < SD_STATUS_INTERVAL_MS) {
        return;
    }
    lastCheckMs = now;

    // Kalau SD belum ter-mount, coba mount
    if (!g_sd_mounted) {
        SD_MountTask();
        return;
    }

    // Ambil waktu sekarang dari RTC
    DS3231_TimeTypeDef rtc;
    if (!DS3231_ReadTime(&rtc)) {
        // Kalau RTC gagal dibaca, tidak update status.json
        return;
    }

    // Bentuk string timestamp ISO: "YYYY-MM-DDTHH:MM:SSZ"
    char ts[32];
    snprintf(ts, sizeof(ts), "%04u-%02u-%02uT%02u:%02u:%02uZ",
             rtc.year, rtc.month, rtc.day,
             rtc.hours, rtc.minutes, rtc.seconds);

    // JSON 1 baris, berisi waktu terakhir online
    // Contoh: {"last_online":"2025-01-08T12:34:56Z"}
    char jsonLine[96];
    int len = snprintf(jsonLine, sizeof(jsonLine),
                       "{\"last_online\":\"%s\"}\r\n", ts);
    if (len <= 0 || len >= (int)sizeof(jsonLine)) {
        return;
    }

    FIL f;
    FRESULT fr = f_open(&f, STATUS_FILE_NAME, FA_WRITE | FA_CREATE_ALWAYS);
    if (fr != FR_OK) {
        UART_Print("[STATUS] Open status.json failed\r\n");
        if (!g_err_sd) {
            g_err_sd = 1;
            Debug_ErrorJson(MOD_SD, ST_ERR, ERR_NOT_FOUND);
            SendErrorTLV(MOD_SD, ST_ERR, ERR_NOT_FOUND);
        }
        // Unmount supaya nanti dicoba mount ulang
        f_mount(NULL, (TCHAR const*)USERPath, 0);
        g_sd_mounted = 0;
        return;
    }

    UINT bw;
    fr = f_write(&f, jsonLine, len, &bw);
    f_sync(&f);
    f_close(&f);

    if (fr != FR_OK || bw != (UINT)len) {
        UART_Print("[STATUS] Write status.json failed\r\n");
        if (!g_err_sd) {
            g_err_sd = 1;
            Debug_ErrorJson(MOD_SD, ST_ERR, ERR_CRC);
            SendErrorTLV(MOD_SD, ST_ERR, ERR_CRC);
        }
        f_mount(NULL, (TCHAR const*)USERPath, 0);
        g_sd_mounted = 0;
        return;
    }

    // Kalau sebelumnya SD error lalu sekarang sukses, kirim status OK
    if (g_err_sd) {
        g_err_sd = 0;
        Debug_ErrorJson(MOD_SD, ST_OK, ERR_NONE);
        SendErrorTLV(MOD_SD, ST_OK, ERR_NONE);
    }

    // (Opsional) debug
    // UART_Print("[STATUS] status.json updated\r\n");
}

/* ==================== APPLICATION ==================== */
static void App_BackgroundSensors(uint32_t now)
{
    static uint8_t  initialized   = 0;
    static uint32_t lastDhtMs     = 0;
    static uint16_t stableMask    = 0;
    static uint16_t lastRawMask   = 0;
    static uint32_t lastChangeMs  = 0;
    static uint32_t lastRtcCheckMs = 0;

    if (!initialized)
    {
        uint16_t mask0 = 0;
        for (uint8_t i = 0; i < NUM_INPUTS; i++) {
            GPIO_PinState s = HAL_GPIO_ReadPin(g_inputs[i].Port, g_inputs[i].Pin);
            if (s == GPIO_PIN_RESET) mask0 |= (1 << i);
        }
        stableMask        = mask0;
        lastRawMask       = mask0;
        g_nativeInputMask = mask0;
        RecomputeCombinedInputMask();
        lastChangeMs      = now;
        lastDhtMs         = now - 2000U;
        initialized       = 1;
    }

    // Update temp/hum every 2s
    if ((now - lastDhtMs) >= 2000U)
    {
        float t_tmp, h_tmp;
        uint8_t ok = DHT22_Read(&t_tmp, &h_tmp);

        if (ok) {
            // Baca SUKSES
            g_temp = t_tmp;
            g_hum  = h_tmp;

            // Jika sebelumnya error, sekarang recover
            if (g_err_dht) {
                g_err_dht = 0;
                Debug_ErrorJson(MOD_DHT, ST_OK, ERR_NONE);
                SendErrorTLV(MOD_DHT, ST_OK, ERR_NONE);
            }
        } else {
            // Baca GAGAL , tunda pelaporan selama 5 detik
            if ((now - g_bootMs) >= DHT_ERROR_ENABLE_DELAY_MS) {
                if (!g_err_dht) {
                    g_err_dht = 1;
                    Debug_ErrorJson(MOD_DHT, ST_ERR, ERR_TIMEOUT);   // baca DHT timeout / gagal
                    SendErrorTLV(MOD_DHT, ST_ERR, ERR_TIMEOUT);
                }
            }
        }

        lastDhtMs = now;
    }

    // ADS background
    ADS_Task();

    // Read RAW inputs
    uint16_t rawMask = 0;
    for (uint8_t i = 0; i < NUM_INPUTS; i++) {
        GPIO_PinState s = HAL_GPIO_ReadPin(g_inputs[i].Port, g_inputs[i].Pin);
        if (s == GPIO_PIN_RESET) rawMask |= (1 << i);
    }

    if (rawMask != lastRawMask) {
        lastRawMask  = rawMask;
        lastChangeMs = now;
    }

    // Debounce 500 ms
    if ((rawMask != stableMask) && ((now - lastChangeMs) >= INPUT_CHANGE_TIME_MS))
    {
        stableMask        = rawMask;
        g_nativeInputMask = stableMask;
        RecomputeCombinedInputMask();
    }

    static uint32_t lastAppliedMask = 0xFFFFFFFFUL;
    if (g_inputMaskStable != lastAppliedMask || g_ruleUpdated)
    {
        ApplyRules(g_inputMaskStable);
        lastAppliedMask = g_inputMaskStable;
    }

    if (g_ruleUpdated)
    {
        PrintRuleStatus();
        g_ruleUpdated = 0;
    }

    /*Update RTC*/
    if ((now - lastRtcCheckMs) >= 2000U) {
        lastRtcCheckMs = now;

        DS3231_TimeTypeDef tmp;
        if (!DS3231_ReadTime(&tmp)) {
            if (!g_err_rtc) {
                g_err_rtc = 1;
                Debug_ErrorJson(MOD_RTC, ST_ERR, ERR_TIMEOUT);
                SendErrorTLV(MOD_RTC, ST_ERR, ERR_TIMEOUT);
            }
        } else {
            if (g_err_rtc) {
                g_err_rtc = 0;
                Debug_ErrorJson(MOD_RTC, ST_OK, ERR_NONE);
                SendErrorTLV(MOD_RTC, ST_OK, ERR_NONE);
            }
        }
    }
}

static void Acquire_FillMeasurement(Measurement_t *m, const DS3231_TimeTypeDef *rtc)
{
    m->seq = g_nextSeq;
    m->rtc = *rtc;

    m->temp = (int16_t)(g_temp * 10.0f);
    m->hum  = (int16_t)(g_hum * 10.0f);

    m->dig_in = g_inputMaskStable;

    for (int i = 0; i < 4; i++) {
        m->an_in[i] = (uint16_t)g_anCache[i];
    }

    m->q = 0;
    for (int i = 0; i < NUM_OUTPUTS; i++) {
        if (HAL_GPIO_ReadPin(g_outputs[i].Port, g_outputs[i].Pin) == GPIO_PIN_SET) {
            m->q |= (1U << i);
        }
    }
}

static void App_HandleAcquisition(uint32_t now)
{
    static uint32_t lastLogTick = 0;
    static uint8_t  initialized = 0;

    bool needLog = false;
    bool byIo    = false;

    // Inisialisasi pertama kali: mulai hitung interval dari waktu boot
    if (!initialized) {
        lastLogTick = now;
        initialized = 1;
    }

    // 1) Trigger: perubahan input (langsung kirim, reset interval)
    if (g_io_changed_flag) {
        g_io_changed_flag = 0;
        needLog = true;
        byIo    = true;
    }

    // 2) Periodik: tiap DELIVERY_TIME_INTERVAL (misalnya 60000 ms)
    if (!needLog && (now - lastLogTick) >= DELIVERY_TIME_INTERVAL) {
        needLog = true;
        byIo    = false;
    }

    if (!needLog) return;

    // Reset base waktu interval ke saat pengiriman ini
    lastLogTick = now;

    // ======= Di bawah ini biarkan persis seperti semula =======

    // 3) Baca RTC
    DS3231_TimeTypeDef rtc;
    if (!DS3231_ReadTime(&rtc)) {
        if (!g_err_rtc) {
            g_err_rtc = 1;
            Debug_ErrorJson(MOD_RTC, ST_ERR, ERR_TIMEOUT);
            SendErrorTLV(MOD_RTC, ST_ERR, ERR_TIMEOUT);
        }
        memset(&rtc, 0, sizeof(rtc));
        rtc.year  = 2000;
        rtc.month = 1;
        rtc.day   = 1;
    } else {
        if (g_err_rtc) {
            g_err_rtc = 0;
            Debug_ErrorJson(MOD_RTC, ST_OK, ERR_NONE);
            SendErrorTLV(MOD_RTC, ST_OK, ERR_NONE);
        }
    }

    // 4) Isi measurement
    Measurement_t m;
    Acquire_FillMeasurement(&m, &rtc);

    // 5) Encode JSON (untuk SD & debug)
    char  jsonLine[JSON_LINE_MAX];
    size_t jsonLen = JSON_EncodeMeasurement(&m, jsonLine, sizeof(jsonLine));
    if (jsonLen == 0) {
        UART_Print("[ACQ] JSON encode failed\r\n");
        return;
    }

    // 6) Debug ke UART selalu (USART1)
    Debug_PrintMeasurementJSON(&m, byIo);

    // 7) Kalau SD ter-mount, simpan ke log.json
    if (g_sd_mounted) {
        if (!Log_AppendJSON(jsonLine)) {
            UART_Print("[ACQ] Log append failed\r\n");
            // Tetap lanjut kirim TLV
        }
    }

    // 8) Serahkan ke TX state machine (non-blocking, dgn retry otomatis)
    //    Data SUDAH aman di log.json dari langkah 7 di atas - kalau
    //    TX_StartNew menolak karena masih busy, tidak ada data hilang,
    //    hanya pengiriman ke ESP32 yang tertunda ke siklus berikutnya.
    TX_StartNew(&m);

    // 9) Naikkan seq
    g_nextSeq++;
}

static void App_HandleAck(void)
{
    if (!g_ack_pending) return;

    uint32_t seq = g_ack_seq;
    g_ack_pending = 0;

    if (!g_sd_mounted) {
        // Tidak ada log untuk dihapus, cukup info saja
        snprintf(g_debugBuf, sizeof(g_debugBuf),
                 "[ACK] seq=%lu (no SD, nothing to remove)\r\n",
                 (unsigned long)seq);
        UART_Print(g_debugBuf);
        return;
    }

    bool removed = Log_RemoveSeq(seq);

    snprintf(g_debugBuf, sizeof(g_debugBuf),
             "[ACK] seq=%lu, removed_from_log=%s\r\n",
             (unsigned long)seq, removed ? "YES" : "NO");
    UART_Print(g_debugBuf);
}

/* ----------------------------------------------------------------------
   TX_StartNew()
   Dipanggil dari App_HandleAcquisition() SEBAGAI PENGGANTI panggilan
   langsung ke SendMeasurementTLV(&m). Tidak langsung kirim - hanya
   menyiapkan context dan menyerahkan eksekusi ke TX_Process() yang
   berjalan non-blocking tiap iterasi main loop.

   Kalau TX sebelumnya masih berjalan (state != IDLE), measurement baru
   TIDAK ditembak (supaya tidak menimpa context yang sedang retry/wait-ack).
   Data measurement yang baru tetap sudah aman di log.json (sudah dipanggil
   Log_AppendJSON sebelum titik ini di App_HandleAcquisition), jadi tidak
   ada data yang hilang - hanya pengiriman TLV-nya yang ditunda ke siklus
   App_HandleAcquisition berikutnya.
   ---------------------------------------------------------------------- */
static void TX_StartNew(const Measurement_t *m)
{
    if (g_txState != TX_STATE_IDLE)
    {
        snprintf(g_debugBuf, sizeof(g_debugBuf),
                 "[TX] busy (state=%d), seq=%lu tetap di log.json, kirim ditunda\r\n",
                 (int)g_txState, (unsigned long)m->seq);
        UART_Print(g_debugBuf);
        return;
    }

    g_txCtx.seq         = m->seq;
    g_txCtx.measurement = *m;          /* copy penuh, dipakai ulang saat retry */
    g_txCtx.retryCount  = 0;
    g_txCtx.jsonLen     = 0;           /* tidak dipakai di jalur TLV, dibiarkan utk kompatibilitas struct */

    g_txState = TX_STATE_SEND_FRAME;
}

/* ----------------------------------------------------------------------
   TX_Process()
   Dipanggil SETIAP iterasi while(1) di main(), tanpa blocking dan tanpa
   HAL_Delay di dalamnya. Mengeksekusi satu state per pemanggilan.

   Alur:
     IDLE            -> menunggu TX_StartNew() dipanggil
     SEND_FRAME      -> kirim TLV via SendMeasurementTLV (re-encode dari
                        g_txCtx.measurement, jadi retry kirim ulang frame
                        yang identik)
     WAIT_TX_COMPLETE-> (di implementasi ini SendMeasurementTLV blocking
                        HAL_UART_Transmit dengan timeout 200ms, sudah
                        cukup singkat - state ini dilewati langsung ke
                        WAIT_ACK. Disediakan di enum utk pengembangan
                        DMA/IT based transmit di masa depan)
     WAIT_ACK        -> polling g_ack_pending (diisi ISR USART2) sampai
                        match seq atau timeout ACK_TIMEOUT_MS
     ACK_OK          -> hapus baris log.json utk seq ini, balik ke IDLE
     RETRY_OR_FAIL   -> increment retryCount, ulang SEND_FRAME kalau
                        masih di bawah TX_MAX_RETRY, kalau tidak pindah
                        ke MOVE_TO_ERROR
     MOVE_TO_ERROR   -> log kegagalan permanen, data TETAP ada di
                        log.json (tidak dihapus - itulah keamanannya),
                        balik ke IDLE supaya measurement berikutnya
                        tidak ikut macet
   ---------------------------------------------------------------------- */
static void TX_Process(uint32_t now)
{
    switch (g_txState)
    {
    case TX_STATE_IDLE:
        /* tidak ada TX aktif */
        break;

    case TX_STATE_SEND_FRAME:
        if (SendMeasurementTLV(&g_txCtx.measurement))
        {
            g_txCtx.ackDeadlineMs = now + ACK_TIMEOUT_MS;
            g_txState = TX_STATE_WAIT_ACK;
        }
        else
        {
            /* UART_Transmit gagal (HAL_BUSY/HAL_ERROR) - kemungkinan
               line sedang dipakai. Langsung ke retry, bukan ke error,
               karena ini biasanya transient. */
            g_txState = TX_STATE_RETRY_OR_FAIL;
        }
        break;

    case TX_STATE_WAIT_TX_COMPLETE:
        /* Tidak dipakai pada implementasi UART blocking saat ini.
           Disediakan utk migrasi ke HAL_UART_Transmit_IT/DMA nanti -
           pada saat itu state ini menunggu callback TxCpltCallback
           sebelum lanjut ke WAIT_ACK. */
        g_txState = TX_STATE_WAIT_ACK;
        break;

    case TX_STATE_WAIT_ACK:
        if (g_ack_pending && g_ack_seq == g_txCtx.seq)
        {
            g_ack_pending = 0;
            g_txState = TX_STATE_ACK_OK;
        }
        else if ((int32_t)(now - g_txCtx.ackDeadlineMs) >= 0)
        {
            snprintf(g_debugBuf, sizeof(g_debugBuf),
                     "[TX] ACK timeout seq=%lu\r\n",
                     (unsigned long)g_txCtx.seq);
            UART_Print(g_debugBuf);
            g_txState = TX_STATE_RETRY_OR_FAIL;
        }
        /* kalau ACK datang dengan seq BERBEDA dari yang ditunggu,
           dibiarkan saja di sini - kemungkinan ACK basi dari frame
           sebelumnya yang sudah lewat timeout-nya sendiri. Dibuang
           secara implisit pada iterasi berikutnya karena g_ack_pending
           akan ditimpa ACK baru atau tetap stale (tidak match selamanya
           sampai ada ACK baru yang benar). */
        break;

    case TX_STATE_ACK_OK:
        if (g_sd_mounted)
        {
            bool removed = Log_RemoveSeq(g_txCtx.seq);
            snprintf(g_debugBuf, sizeof(g_debugBuf),
                     "[TX] ACK OK seq=%lu, log_removed=%s\r\n",
                     (unsigned long)g_txCtx.seq, removed ? "YES" : "NO");
            UART_Print(g_debugBuf);
        }
        else
        {
            snprintf(g_debugBuf, sizeof(g_debugBuf),
                     "[TX] ACK OK seq=%lu (no SD mounted)\r\n",
                     (unsigned long)g_txCtx.seq);
            UART_Print(g_debugBuf);
        }
        g_txState = TX_STATE_IDLE;
        break;

    case TX_STATE_RETRY_OR_FAIL:
        g_txCtx.retryCount++;
        if (g_txCtx.retryCount < TX_MAX_RETRY)
        {
            snprintf(g_debugBuf, sizeof(g_debugBuf),
                     "[TX] retry %u/%u seq=%lu\r\n",
                     g_txCtx.retryCount, TX_MAX_RETRY,
                     (unsigned long)g_txCtx.seq);
            UART_Print(g_debugBuf);
            g_txState = TX_STATE_SEND_FRAME;
        }
        else
        {
            g_txState = TX_STATE_MOVE_TO_ERROR;
        }
        break;

    case TX_STATE_MOVE_TO_ERROR:
        /* Data TIDAK dihapus dari log.json - itu poin pentingnya.
           Kegagalan kirim TLV ke ESP32 tidak berarti data hilang;
           tetap tersimpan di SD utk dikirim manual/recovery nanti.
           Kembali ke IDLE supaya measurement BERIKUTNYA (siklus
           App_HandleAcquisition selanjutnya) tidak ikut tersumbat
           oleh kegagalan yang sudah permanen ini. */
        snprintf(g_debugBuf, sizeof(g_debugBuf),
                 "[TX] FAILED permanen seq=%lu setelah %u retry, data tetap di log.json\r\n",
                 (unsigned long)g_txCtx.seq, TX_MAX_RETRY);
        UART_Print(g_debugBuf);
        g_txState = TX_STATE_IDLE;
        break;

    default:
        g_txState = TX_STATE_IDLE;
        break;
    }
}

/* ==================== DEBUG ==================== */
static void Debug_ErrorJson(uint8_t mid, uint8_t st, uint8_t ec)
{
    // Format baru: {"mid":1,"st":1,"ec":2}
    snprintf(g_debugBuf, sizeof(g_debugBuf),
             "[ERR]{\"mid\":%u,\"st\":%u,\"ec\":%u}\r\n",
             mid, st, ec);
    UART_Print(g_debugBuf);
}

static void Debug_PrintMeasurementJSON(const Measurement_t *m, bool byIo)
{
    char ts[32];
    char digArr[64];
    char anArr[32];
    char qArr[24];
    char *p;
    int n;

    // Build timestamp
    snprintf(ts, sizeof(ts), "%04u-%02u-%02uT%02u:%02u:%02uZ",
             m->rtc.year, m->rtc.month, m->rtc.day,
             m->rtc.hours, m->rtc.minutes, m->rtc.seconds);

    // Build dig_in array
    p = digArr;
    *p++ = '[';
    for (uint8_t i = 0; i < NUM_INPUTS_TOTAL; i++) {
        uint8_t bit = (m->dig_in & (1U << i)) ? 1 : 0;
        n = snprintf(p, sizeof(digArr) - (p - digArr), "%u%s",
                     bit, (i < NUM_INPUTS_TOTAL - 1) ? "," : "");
        if (n > 0) p += n;
    }
    *p++ = ']';
    *p = '\0';

    // Build an_in array
    snprintf(anArr, sizeof(anArr), "[%u,%u,%u,%u]",
             m->an_in[0], m->an_in[1], m->an_in[2], m->an_in[3]);

    // Build q array
    p = qArr;
    *p++ = '[';
    for (uint8_t i = 0; i < NUM_OUTPUTS; i++) {
        uint8_t bit = (m->q & (1U << i)) ? 1 : 0;
        n = snprintf(p, sizeof(qArr) - (p - qArr), "%u%s",
                     bit, (i < NUM_OUTPUTS - 1) ? "," : "");
        if (n > 0) p += n;
    }
    *p++ = ']';
    *p = '\0';

    const char *src = byIo ? "[ACQ][IO] " : "[ACQ][TMR] ";

    snprintf(g_debugBuf, sizeof(g_debugBuf),
        "%s{\"timestamp\":\"%s\",\"env\":{\"temp\":%.1f,\"hum\":%.1f},"
        "\"dig_in\":%s,\"an_in\":%s,\"q\":%s,\"seq\":%lu}\r\n",
        src, ts,
        m->temp / 10.0f, m->hum / 10.0f,
        digArr, anArr, qArr,
        (unsigned long)m->seq
    );

    UART_Print(g_debugBuf);
}

/* ==================== TLV DEBUG DUMP ==================== */
static void Debug_DumpTLV(const char *prefix, const uint8_t *tlv, size_t len)
{
    size_t i = 0;

    snprintf(g_debugBuf, sizeof(g_debugBuf),
             "%s TLV dump, len=%u\r\n", prefix, (unsigned)len);
    UART_Print(g_debugBuf);

    while (i + 2 <= len)
    {
        uint8_t tag = tlv[i];
        uint8_t l   = tlv[i + 1];
        i += 2;

        if (i + l > len) {
            snprintf(g_debugBuf, sizeof(g_debugBuf),
                     "%s  !! malformed TLV (tag=0x%02X, len=%u, remaining=%u)\r\n",
                     prefix, tag, l, (unsigned)(len - (i - 2)));
            UART_Print(g_debugBuf);
            break;
        }

        int n = snprintf(g_debugBuf, sizeof(g_debugBuf),
                         "%s  tag=0x%02X len=%u val=", prefix, tag, l);
        for (uint8_t j = 0; j < l && n < (int)sizeof(g_debugBuf) - 3; j++) {
            n += snprintf(g_debugBuf + n, sizeof(g_debugBuf) - n,
                          "%02X ", tlv[i + j]);
        }
        snprintf(g_debugBuf + n, sizeof(g_debugBuf) - n, "\r\n");
        UART_Print(g_debugBuf);

        /* Interpretasi tag yang dikenal */
        if (tag == TLV_TAG_SEQ && l == 4) {
            uint32_t seq = TLV_ReadU32BE(&tlv[i]);
            snprintf(g_debugBuf, sizeof(g_debugBuf),
                     "%s    -> SEQ = %lu\r\n",
                     prefix, (unsigned long)seq);
            UART_Print(g_debugBuf);
        }
        else if (tag == TLV_TAG_TIMESTAMP && l == 4) {
            uint32_t ts = TLV_ReadU32BE(&tlv[i]);
            snprintf(g_debugBuf, sizeof(g_debugBuf),
                     "%s    -> TIMESTAMP = %lu (epoch)\r\n",
                     prefix, (unsigned long)ts);
            UART_Print(g_debugBuf);
        }
        else if (tag == TLV_TAG_TEMP && l == 2) {
            int16_t t = (int16_t)(((uint16_t)tlv[i] << 8) | tlv[i + 1]);
            snprintf(g_debugBuf, sizeof(g_debugBuf),
                     "%s    -> TEMP = %.1f C\r\n",
                     prefix, t / 10.0f);
            UART_Print(g_debugBuf);
        }
        else if (tag == TLV_TAG_HUM && l == 2) {
            int16_t h = (int16_t)(((uint16_t)tlv[i] << 8) | tlv[i + 1]);
            snprintf(g_debugBuf, sizeof(g_debugBuf),
                     "%s    -> HUM  = %.1f %%\r\n",
                     prefix, h / 10.0f);
            UART_Print(g_debugBuf);
        }
        else if (tag == TLV_TAG_DIG_IN && l == 2) {
            uint16_t dig = ((uint16_t)tlv[i] << 8) | tlv[i + 1];
            snprintf(g_debugBuf, sizeof(g_debugBuf),
                     "%s    -> DIG_IN mask = 0x%04X\r\n",
                     prefix, dig);
            UART_Print(g_debugBuf);
        }
        else if (tag == TLV_TAG_AN_IN && l == 8) {
            uint16_t an[4];
            for (int ch = 0; ch < 4; ch++) {
                an[ch] = ((uint16_t)tlv[i + ch*2] << 8) | tlv[i + ch*2 + 1];
            }
            snprintf(g_debugBuf, sizeof(g_debugBuf),
                     "%s    -> AN_IN = [%u,%u,%u,%u]\r\n",
                     prefix, an[0], an[1], an[2], an[3]);
            UART_Print(g_debugBuf);
        }
        else if (tag == TLV_TAG_Q && l == 1) {
            uint8_t q = tlv[i];
            snprintf(g_debugBuf, sizeof(g_debugBuf),
                     "%s    -> Q mask = 0x%02X\r\n",
                     prefix, q);
            UART_Print(g_debugBuf);
        }
        else if (tag == TAG_RULE) {   // dari ESP32
            snprintf(g_debugBuf, sizeof(g_debugBuf),
                     "%s    -> RULE string: \"%.*s\"\r\n",
                     prefix, l, (const char *)&tlv[i]);
            UART_Print(g_debugBuf);
        }
        else if (tag == TAG_RTC_TS && l == 4) {
            uint32_t epoch = TLV_ReadU32BE(&tlv[i]);
            snprintf(g_debugBuf, sizeof(g_debugBuf),
                     "%s    -> RTC EPOCH = %lu\r\n",
                     prefix, (unsigned long)epoch);
            UART_Print(g_debugBuf);
        }

        i += l;
    }
}

static void WD_Service(void)
{
    if (wd.sensor &&
        wd.acquisition &&
        wd.ack &&
        wd.tx &&
        wd.sd)
    {
        HAL_IWDG_Refresh(&hiwdg);

        wd.sensor = 0;
        wd.acquisition = 0;
        wd.ack = 0;
        wd.tx = 0;
        wd.sd = 0;
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
