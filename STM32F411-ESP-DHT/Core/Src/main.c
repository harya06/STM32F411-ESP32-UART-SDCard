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
#define NUM_ANALOG 4U
/* USER CODE END First_Private_Defines */

typedef struct
{
    uint8_t  seconds;
    uint8_t  minutes;
    uint8_t  hours;
    uint8_t  dayOfWeek;   // 1=Sunday .. 7=Saturday
    uint8_t  day;         // 1..31
    uint8_t  month;       // 1..12
    uint16_t year;        // mis: 2025
} DS3231_TimeTypeDef;

/*Struct Input/Output fisik*/
typedef struct {
    GPIO_TypeDef *Port;
    uint16_t      Pin;
} IO_PinDef_t;

/*Enum index input/output logis*/
typedef enum {
    IN_1 = 0,
    IN_2,
    IN_3,
    IN_4,
    IN_5,
    IN_6,
    IN_7,
    IN_8,
    IN_9,
    IN_10,
    IN_11,
    NUM_INPUTS
} InputId_t;

typedef enum {
    OUT_1 = 0,
    OUT_2,
    OUT_3,
    OUT_4,
    NUM_OUTPUTS
} OutputId_t;

/*Struct Rules*/
typedef enum {
    RULE_LOGIC_OFF = 0,
    RULE_LOGIC_OR,
    RULE_LOGIC_AND,
    RULE_LOGIC_NOT
} RuleLogic_t;

typedef struct {
    RuleLogic_t logic;               // OFF / OR / AND / NOT
    uint8_t     inputs[NUM_INPUTS];  // daftar input 1..11
    uint8_t     inputCount;
} RuleDef_t;

typedef struct {
    uint32_t seq;                 // seq of record currently being sent
    uint8_t  cbor_buf[256];       // CBOR payload of this record
    uint16_t cbor_len;
    uint8_t  retry_count;
    uint32_t ack_deadline_ms;
    uint8_t  waiting_ack;         // flag used alongside state
} TxContext_t;

typedef enum {
    APP_STATE_INIT = 0,
    APP_STATE_SD_MOUNT,
    APP_STATE_LOG_RECOVER,
    APP_STATE_RUN
} AppState_t;

typedef enum {
    TX_STATE_IDLE = 0,
    TX_STATE_LOAD_HEAD,
    TX_STATE_SEND_FRAME,
    TX_STATE_WAIT_TX_COMPLETE,
    TX_STATE_WAIT_ACK,
    TX_STATE_ACK_OK,
    TX_STATE_RETRY_OR_FAIL,
    TX_STATE_MOVE_TO_ERROR
} TxState_t;

typedef enum {
    RX_STATE_WAIT_SOF = 0,
    RX_STATE_TYPE,
    RX_STATE_CMD,
    RX_STATE_LEN_H,
    RX_STATE_LEN_L,
    RX_STATE_PAYLOAD,
    RX_STATE_CRC_H,
    RX_STATE_CRC_L
} RxState_t;

// Satu sampel measurement
typedef struct {
    uint32_t seq;
    DS3231_TimeTypeDef rtc;
    float temp;
    float hum;
    uint8_t dig_in[NUM_INPUTS];
    uint8_t an_in[NUM_ANALOG];
    uint8_t q[NUM_OUTPUTS];
} Measurement_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SOF_BYTE                0x7E
#define DS3231_ADDR             (0x68U << 1)
#define ADS1115_ADDR            (0x48U << 1)

#define ADS_FS_VOLT             4.096f
#define ADS_READ_PERIOD_MS      100U

#define RX_BUF_SIZE             256U
#define INPUT_CHANGE_TIME_MS    500U
#define DELIVERY_TIME_INTERVAL  60000U
#define RULE_MAX_PAYLOAD        240U
#define RULE_STR_MAX_LEN        32U

#define LOG_FILE_NAME      "log.cbor"
#define ERROR_FILE_NAME    "error.cbor"

#define TX_MAX_RETRY       3
#define ACK_TIMEOUT_MS     2000U
#define UART_FRAME_MAX_LEN 300

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */


static float     temp = 0.0f;
static float     hum  = 0.0f;
static char      buf[256];

static volatile uint16_t g_inputMaskStable = 0;
static volatile uint8_t  g_ruleUpdated     = 0;

static int16_t  g_anRaw[NUM_ANALOG];    // RAW ADS1115
static uint8_t  g_anCache[NUM_ANALOG];  // 0..255 scaled
static uint32_t g_adsTick = 0;

static uint8_t  rxByte;
static uint8_t           ruleFrameBuf[RX_BUF_SIZE];
static uint16_t          ruleFrameIdx    = 0;
static int16_t           ruleExpectedLen = -1;
static char              ruleJsonRaw[RX_BUF_SIZE];

static uint8_t rs485RxByte;
static uint8_t          rs485Buf[128];
static uint16_t         rs485Idx = 0;

static AppState_t g_appState = APP_STATE_SD_MOUNT;
static TxState_t  g_txState  = TX_STATE_IDLE;

static uint8_t  g_sd_mounted = 0;
static uint32_t g_nextSeq    = 1;

// Flag dari ISR
volatile uint8_t g_acq_timer_flag = 0;
volatile uint8_t g_io_changed_flag = 0;

// ACK dari ESP32
volatile uint8_t  g_ack_pending = 0;
volatile uint32_t g_ack_seq     = 0;

// UART2 TX (non-blocking)
static uint8_t  g_uart2_tx_done = 1;
static uint8_t  g_uart2_tx_buf[UART_FRAME_MAX_LEN];
static uint16_t g_uart2_tx_len  = 0;
static TxContext_t g_tx;

// State penerima ACK
static RxState_t g_rxState;

/* Mapping input I1..I11 */
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

/* Mapping output Q1..Q4 */
static const IO_PinDef_t g_outputs[NUM_OUTPUTS] = {
    [OUT_1] = { OUTPUT_1_GPIO_Port, OUTPUT_1_Pin },
    [OUT_2] = { OUTPUT_2_GPIO_Port, OUTPUT_2_Pin },
    [OUT_3] = { OUTPUT_3_GPIO_Port, OUTPUT_3_Pin },
    [OUT_4] = { OUTPUT_4_GPIO_Port, OUTPUT_4_Pin }
};

/* Rules aktif untuk Q1..Q4 */
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
/* USER CODE BEGIN PFP */

/*Fungsi Sensor Suhu*/
void DWT_Delay_Init(void);
void delay_us(uint32_t us);
void DHT_SetOutput(void);
void DHT_SetInput(void);
uint8_t DHT_Start(void);
uint8_t DHT_ReadBit(void);
uint8_t DHT_ReadByte(void);
uint8_t DHT22_Read(float *temperature, float *humidity);

/*Fungsi RTC*/
static uint8_t bcd2bin(uint8_t val);
static uint8_t bin2bcd(uint8_t val);
uint8_t DS3231_ReadTime(DS3231_TimeTypeDef *t);
uint8_t DS3231_SetTime(const DS3231_TimeTypeDef *t);
static uint32_t RTC_ToEpochSeconds(const DS3231_TimeTypeDef *t);

/* Fungsi ADS1115 */
static int16_t ADS1115_ReadRaw(uint8_t ch);    // channel 0..3
static float ADS1115_ConvertRawToVolt(int16_t raw);
static void ADS_Task(void);
uint8_t VoltageToPWM_0_255(float voltage);


/*Callback ESP*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
static void Rules_Reset(void);
static void ParseOneRuleString(const char *ruleStr);
void ParseRuleJSON(char *json);
static uint8_t EvalOutputState(int qIdx, uint16_t inputMask);
void ApplyRules(uint16_t inputMask);
void PrintRuleStatus(void);
void ParseRTCJSON(const char *json);
void OnRuleFrameReceived(uint8_t *payload, uint16_t len);

void Log_InitRuntime(void);
bool SD_MountTask(void);
bool Log_RecoverTask(void);

void App_BackgroundSensors(uint32_t now);
void App_HandleAcquisition(uint32_t now);

void Tx_InitRuntime(TxContext_t *ctx);
void Tx_StateMachine_Run(TxState_t *state, TxContext_t *ctx, uint32_t now);

uint16_t CRC16_Modbus(uint8_t *data, uint16_t len);
size_t CBOR_EncodeMeasurement(const Measurement_t *m, uint8_t *out, size_t max_len);

bool Log_AppendRecord(const uint8_t *cbor, size_t len);
bool Log_AppendErrorRecord(const uint8_t *cbor, size_t len);
bool Log_ReadHeadRecord(uint32_t *out_seq, uint8_t *out_cbor, uint16_t *out_len);
bool Log_TruncateUpToSeq(uint32_t upto_seq);
bool Log_ReadNextRecordFromFile(FIL *pf, uint32_t *out_seq, uint8_t *out_cbor, uint16_t *out_len);

/*Debug log*/
static void Debug_PrintMeasurementJSON(const Measurement_t *m, bool trigByIo);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void UART_Print(char* str)
{
    HAL_UART_Transmit(&huart1, (uint8_t *) str, strlen(str), 100);
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
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  Log_InitRuntime();
  Tx_InitRuntime(&g_tx);

  g_appState = APP_STATE_SD_MOUNT;
  g_txState  = TX_STATE_IDLE;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint32_t now = HAL_GetTick();

	  switch (g_appState)
	  {
	  case APP_STATE_SD_MOUNT:
		  if (SD_MountTask()) {
			  g_appState = APP_STATE_LOG_RECOVER;
	      }
	      break;

	  case APP_STATE_LOG_RECOVER:
	      if (Log_RecoverTask()) {
	          g_appState = APP_STATE_RUN;
	      }
	      break;

	  case APP_STATE_RUN:
	      App_BackgroundSensors(now);             // update cache DHT, ADS, input, rules
	      App_HandleAcquisition(now);             // buat measurement & append ke log.cbor jika trigger
	      Tx_StateMachine_Run(&g_txState, &g_tx, now); // kirim record tertua, handle ACK/retry
	      break;

	  default:
	      g_appState = APP_STATE_SD_MOUNT;
	      break;
	  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
  HAL_UART_Receive_IT(&huart1, &rs485RxByte, 1);

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
  	HAL_UART_Receive_IT(&huart2, &rxByte, 1);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SUHU_SENSOR_Pin|SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OUTPUT_4_Pin|OUTPUT_3_Pin|OUTPUT_2_Pin|OUTPUT_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : INPUT_1_Pin INPUT_8_Pin INPUT_9_Pin INPUT_10_Pin
                           INPUT_11_Pin */
  GPIO_InitStruct.Pin = INPUT_1_Pin|INPUT_8_Pin|INPUT_9_Pin|INPUT_10_Pin
                          |INPUT_11_Pin;
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

  /*Configure GPIO pins : INPUT_2_Pin INPUT_3_Pin INPUT_4_Pin INPUT_5_Pin
                           INPUT_6_Pin INPUT_7_Pin */
  GPIO_InitStruct.Pin = INPUT_2_Pin|INPUT_3_Pin|INPUT_4_Pin|INPUT_5_Pin
                          |INPUT_6_Pin|INPUT_7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : OUTPUT_4_Pin OUTPUT_3_Pin OUTPUT_2_Pin OUTPUT_1_Pin */
  GPIO_InitStruct.Pin = OUTPUT_4_Pin|OUTPUT_3_Pin|OUTPUT_2_Pin|OUTPUT_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* ===================== DWT DELAY ===================== */
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


/* ===================== GPIO MODE ===================== */
void DHT_SetOutput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = SUHU_SENSOR_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;   // PUSH-PULL
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SUHU_SENSOR_GPIO_Port, &GPIO_InitStruct);
}

void DHT_SetInput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = SUHU_SENSOR_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;           // pakai pull-up + resistor 10k eksternal
    HAL_GPIO_Init(SUHU_SENSOR_GPIO_Port, &GPIO_InitStruct);
}

/* ===================== DHT22  DRIVER ===================== */

uint8_t DHT_Start(void)
{
    uint32_t timeout;

    // Host pull low minimal 1ms
    DHT_SetOutput();
    HAL_GPIO_WritePin(SUHU_SENSOR_GPIO_Port, SUHU_SENSOR_Pin, GPIO_PIN_RESET);
    HAL_Delay(2);                         // 2ms
    HAL_GPIO_WritePin(SUHU_SENSOR_GPIO_Port, SUHU_SENSOR_Pin, GPIO_PIN_SET);
    delay_us(30);                         // 20-40us
    DHT_SetInput();

    // Sensor response: 80us LOW
    timeout = 0;
    while (HAL_GPIO_ReadPin(SUHU_SENSOR_GPIO_Port, SUHU_SENSOR_Pin) == GPIO_PIN_SET)
    {
        delay_us(1);
        if (++timeout > 200)              // ~200us timeout
            return 0;                     // tidak respon
    }

    // 80us HIGH
    timeout = 0;
    while (HAL_GPIO_ReadPin(SUHU_SENSOR_GPIO_Port, SUHU_SENSOR_Pin) == GPIO_PIN_RESET)
    {
        delay_us(1);
        if (++timeout > 200)
            return 0;                     // timing salah
    }

    // Tunggu LOW pertama data (awal bit pertama)
    timeout = 0;
    while (HAL_GPIO_ReadPin(SUHU_SENSOR_GPIO_Port, SUHU_SENSOR_Pin) == GPIO_PIN_SET)
    {
        delay_us(1);
        if (++timeout > 200)
            return 0;
    }

    // Sekarang jalur dalam keadaan LOW (awal bit pertama)
    return 1;
}

uint8_t DHT_ReadBit(void)
{
    uint8_t bit = 0;

    // Saat fungsi dipanggil, jalur sedang LOW (50us).
    // Tunggu naik (awal HIGH bit).
    while (HAL_GPIO_ReadPin(SUHU_SENSOR_GPIO_Port, SUHU_SENSOR_Pin) == GPIO_PIN_RESET);

    // Setelah naik, tunggu ~40us lalu sampling
    delay_us(40);

    if (HAL_GPIO_ReadPin(SUHU_SENSOR_GPIO_Port, SUHU_SENSOR_Pin) == GPIO_PIN_SET)
        bit = 1;

    // Tunggu sampai jalur LOW lagi (akhir HIGH)
    while (HAL_GPIO_ReadPin(SUHU_SENSOR_GPIO_Port, SUHU_SENSOR_Pin) == GPIO_PIN_SET);

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

    if (!DHT_Start())
        return 0;

    // Baca 5 byte (40 bit)
    data[0] = DHT_ReadByte();
    data[1] = DHT_ReadByte();
    data[2] = DHT_ReadByte();
    data[3] = DHT_ReadByte();
    data[4] = DHT_ReadByte();

    // Checksum
    uint8_t sum = data[0] + data[1] + data[2] + data[3];
    if (sum != data[4])
        return 0;

    // Konversi (format DHT22)
    uint16_t rawHum  = (data[0] << 8) | data[1];
    int16_t  rawTemp = (data[2] << 8) | data[3];

    *humidity = rawHum / 10.0f;

    if (rawTemp & 0x8000)           // bit sign
    {
        rawTemp &= 0x7FFF;
        *temperature = -rawTemp / 10.0f;
    }
    else
    {
        *temperature = rawTemp / 10.0f;
    }

    // Filter nilai tidak masuk akal
    if (*humidity < 0.0f || *humidity > 100.0f) return 0;
    if (*temperature < -40.0f || *temperature > 80.0f) return 0;

    return 1;
}


/*=================================== Fungsi RTC ==================================*/
static uint8_t bcd2bin(uint8_t val)
{
    return (uint8_t)((val >> 4) * 10 + (val & 0x0F));
}

static uint8_t bin2bcd(uint8_t val)
{
    return (uint8_t)(((val / 10) << 4) | (val % 10));
}

/* Baca waktu dari DS3231 */
uint8_t DS3231_ReadTime(DS3231_TimeTypeDef *t)
{
    uint8_t buf[7];

    if (HAL_I2C_Mem_Read(&hi2c1,
                         DS3231_ADDR,
                         0x00,                        // register detik
                         I2C_MEMADD_SIZE_8BIT,
                         buf,
                         7,
                         HAL_MAX_DELAY) != HAL_OK)
    {
        return 0;   // gagal I2C
    }

    t->seconds   = bcd2bin(buf[0] & 0x7F);
    t->minutes   = bcd2bin(buf[1] & 0x7F);

    // jam: bit 6 = 12/24h, asumsikan 24h mode
    t->hours     = bcd2bin(buf[2] & 0x3F);

    t->dayOfWeek = bcd2bin(buf[3] & 0x07);
    t->day       = bcd2bin(buf[4] & 0x3F);

    // month: bit7 = century
    t->month     = bcd2bin(buf[5] & 0x1F);

    t->year      = 2000 + bcd2bin(buf[6]);   // DS3231 menyimpan 00..99 -> 2000..2099

    return 1;   // sukses
}

/* Set waktu ke DS3231 (panggil sekali untuk set awal) */
uint8_t DS3231_SetTime(const DS3231_TimeTypeDef *t)
{
    uint8_t buf[7];

    buf[0] = bin2bcd(t->seconds);
    buf[1] = bin2bcd(t->minutes);
    buf[2] = bin2bcd(t->hours);          // 24h mode
    buf[3] = bin2bcd(t->dayOfWeek);
    buf[4] = bin2bcd(t->day);
    buf[5] = bin2bcd(t->month);          // century bit 0
    buf[6] = bin2bcd((uint8_t)(t->year - 2000));

    if (HAL_I2C_Mem_Write(&hi2c1,
                          DS3231_ADDR,
                          0x00,
                          I2C_MEMADD_SIZE_8BIT,
                          buf,
                          7,
                          HAL_MAX_DELAY) != HAL_OK)
    {
        return 0;
    }
    return 1;
}

static uint8_t IsLeapYear(uint16_t year)
{
    // Untuk rentang 2000..2099 cukup year%4==0
    return (year % 4U == 0U);
}

static uint32_t RTC_ToEpochSeconds(const DS3231_TimeTypeDef *t)
{
    // Hitung detik sejak 2000-01-01 00:00:00 (basis arbitrer, konsisten saja)
    static const uint8_t days_in_month[12] =
        {31,28,31,30,31,30,31,31,30,31,30,31};

    uint32_t days = 0;

    // Tambah hari untuk tahun-tahun penuh sejak 2000
    for (uint16_t y = 2000; y < t->year; y++) {
        days += 365U + (IsLeapYear(y) ? 1U : 0U);
    }

    // Tambah hari untuk bulan-bulan penuh di tahun berjalan
    for (uint8_t m = 1; m < t->month; m++) {
        days += days_in_month[m-1];
        if (m == 2 && IsLeapYear(t->year)) {
            days += 1U;    // Feb 29
        }
    }

    // Tambah hari dalam bulan berjalan (day=1 berarti offset 0)
    days += (uint32_t)(t->day - 1U);

    uint32_t seconds =
        days * 86400UL +
        (uint32_t)t->hours   * 3600UL +
        (uint32_t)t->minutes * 60UL  +
        (uint32_t)t->seconds;

    return seconds;
}

/* ===================== ADS1115 DRIVER (I2C2) ===================== */
/*
 * Baca satu channel single-ended ADS1115
 * channel: 0..3 -> AIN0..AIN3
 * Mode: single-shot, PGA ±4.096V, 128 SPS
 */

static int16_t ADS1115_ReadRaw(uint8_t ch)
{
    uint8_t  cfg[3];
    uint8_t  rx[2];
    uint16_t config;

    config  = 0x8000U;
    config |= (uint16_t)((0x04U + ch) << 12);
    config |= 0x0200U;   // ±4.096 V
    config |= 0x0100U;   // single-shot
    config |= 0x0080U;   // 128 SPS
    config |= 0x0003U;   // disable comparator

    cfg[0] = 0x01;
    cfg[1] = (uint8_t)(config >> 8);
    cfg[2] = (uint8_t)(config & 0xFF);

    if (HAL_I2C_Master_Transmit(&hi2c2, ADS1115_ADDR, cfg, 3, 100) != HAL_OK)
        return 0;

    HAL_Delay(8);

    cfg[0] = 0x00;
    if (HAL_I2C_Master_Transmit(&hi2c2, ADS1115_ADDR, cfg, 1, 100) != HAL_OK)
        return 0;
    if (HAL_I2C_Master_Receive(&hi2c2, ADS1115_ADDR, rx, 2, 100) != HAL_OK)
        return 0;

    return (int16_t)((rx[0] << 8) | rx[1]);
}

uint8_t VoltageToPWM_0_255(float voltage)
{
    // Batasi dulu ke 0..3.3 V
    if (voltage < 0.0f)   voltage = 0.0f;
    if (voltage > 3.3f)   voltage = 3.3f;

    float scale = voltage / 3.3f;         // 0.0 .. 1.0
    uint8_t pwm = (uint8_t)(scale * 255.0f + 0.5f);  // +0.5 untuk pembulatan

    return pwm;   // 0..255
}

static float ADS1115_ConvertRawToVolt(int16_t raw)
{
    float v = (float)raw * ADS_FS_VOLT / 32768.0f;
    return (v < 0.0f) ? 0.0f : v;
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

uint16_t CRC16_Modbus(uint8_t *data, uint16_t len)
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

/*Callback UART ESP*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2)   // UART2 = link ke ESP32
    {
        uint8_t b = rxByte;

        if (ruleFrameIdx == 0)
        {
            // Cari SOF
            if (b != SOF_BYTE)
            {
                // buang byte sampai ketemu 0x7E
            }
            else
            {
                ruleFrameBuf[0] = b;
                ruleFrameIdx    = 1;
                ruleExpectedLen = -1;
            }
        }
        else
        {
            if (ruleFrameIdx < RX_BUF_SIZE)
            {
                ruleFrameBuf[ruleFrameIdx++] = b;
            }
            else
            {
                // overflow, reset parser
                ruleFrameIdx    = 0;
                ruleExpectedLen = -1;
            }

            // Setelah punya SOF + LEN_H + LEN_L (3 byte)
            if (ruleFrameIdx == 3)
            {
                uint16_t jsonLen = ((uint16_t)ruleFrameBuf[1] << 8) | ruleFrameBuf[2];
                if (jsonLen > RULE_MAX_PAYLOAD)
                {
                    // payload tidak masuk akal, reset
                    ruleFrameIdx    = 0;
                    ruleExpectedLen = -1;
                }
                else
                {
                    // total = 1(SOF) + 2(LEN) + JSON + 2(CRC)
                    ruleExpectedLen = 1 + 2 + jsonLen + 2;
                }
            }

            // Kalau frame lengkap, proses
            if (ruleExpectedLen > 0 && ruleFrameIdx == ruleExpectedLen)
            {
                uint16_t jsonLen = ((uint16_t)ruleFrameBuf[1] << 8) | ruleFrameBuf[2];

                // cek CRC
                uint16_t crcRecv = ((uint16_t)ruleFrameBuf[ruleFrameIdx-2] << 8) |
                                    ruleFrameBuf[ruleFrameIdx-1];
                uint16_t crcCalc = CRC16_Modbus(ruleFrameBuf, ruleFrameIdx-2);

                if (crcRecv == crcCalc)
                {
                    // panggil handler dengan payload JSON saja
                    OnRuleFrameReceived(&ruleFrameBuf[3], jsonLen);
                }
                else
                {
                    // CRC salah -> abaikan; bisa tambahkan debug ke USART1
                     sprintf(buf, "[RULE] CRC error: recv=0x%04X calc=0x%04X\r\n", crcRecv, crcCalc);
                     HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 100);
                }

                // reset parser
                ruleFrameIdx    = 0;
                ruleExpectedLen = -1;
            }
        }

        // lanjut terima 1 byte lagi
        HAL_UART_Receive_IT(&huart2, &rxByte, 1);
    }

    if (huart == &huart1)
    {
    	uint8_t b = rs485RxByte;

        // contoh sederhana: kumpulkan sampai '\n'
        if (rs485Idx < sizeof(rs485Buf) - 1)
        {
        	rs485Buf[rs485Idx++] = b;
        }

        if (b == '\n')
        {
            rs485Buf[rs485Idx] = '\0';
            rs485Idx = 0;

            // Di sini proses perintah dari RS485
            // Contoh: kirim balik apa adanya
            HAL_UART_Transmit(&huart1, (uint8_t*)"ECHO: ", 6, 100);
            HAL_UART_Transmit(&huart1, rs485Buf, strlen((char*)rs485Buf), 100);
        }

        // mulai terima byte berikutnya
        HAL_UART_Receive_IT(&huart1, &rs485RxByte, 1);
    }
}

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
    if (ruleStr == NULL) return;
    if (ruleStr[0] != 'Q') return;      // harus mulai dengan Q

    int qNum = atoi(&ruleStr[1]);       // Q1 -> 1
    if (qNum < 1 || qNum > NUM_OUTPUTS) return;
    int qIdx = qNum - 1;                // index 0..3

    // cari ':' pertama (setelah Qx)
    const char *p = strchr(ruleStr, ':');
    if (!p) return;
    p++;                                // awal string LOGIC

    // cari ':' kedua (setelah LOGIC)
    const char *p2 = strchr(p, ':');
    if (!p2) return;

    // ambil kata LOGIC
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

    // sisa setelah ':' kedua adalah daftar input: "I1,I2,I3"
    const char *r = p2 + 1;
    while (*r != '\0')
    {
        if (*r == 'I')
        {
            int inNum = atoi(r + 1);  // I1 -> 1, I10 -> 10
            if (inNum >= 1 && inNum <= NUM_INPUTS)
            {
                if (g_rules[qIdx].inputCount < NUM_INPUTS)
                {
                    g_rules[qIdx].inputs[g_rules[qIdx].inputCount++] = (uint8_t)inNum;
                }
            }
            // lompat sampai koma atau akhir string
            while (*r && *r != ',' && *r != '\0') r++;
        }
        else
        {
            r++;
        }
    }
}

void ParseRuleJSON(char *json)
{
    Rules_Reset();
    if (json == NULL) return;

    // cari key "rules"
    char *p = strstr(json, "\"rules\"");
    if (!p) return;

    // cari '[' setelah "rules"
    p = strchr(p, '[');
    if (!p) return;
    p++;    // masuk ke dalam array

    while (*p && *p != ']')
    {
        // cari tanda kutip pembuka
        while (*p && *p != '"' && *p != ']') p++;
        if (*p == ']')
            break;
        p++;   // setelah '"'

        char *start = p;
        // cari tanda kutip penutup
        while (*p && *p != '"') p++;
        if (*p != '"')
            break;

        size_t len = (size_t)(p - start);
        if (len > 0 && len < RULE_STR_MAX_LEN)
        {
            char oneRule[RULE_STR_MAX_LEN];
            memcpy(oneRule, start, len);
            oneRule[len] = '\0';

            // contoh oneRule: "Q1:OR:I1,I2"
            ParseOneRuleString(oneRule);
        }

        if (*p == '"') p++;  // lewati tanda kutip penutup
    }

    // tandai ke main loop bahwa rules baru
    g_ruleUpdated = 1;
}

static uint8_t EvalOutputState(int qIdx, uint16_t inputMask)
{
    RuleDef_t *r = &g_rules[qIdx];

    // Jika OFF atau tidak ada input → selalu 0
    if (r->logic == RULE_LOGIC_OFF || r->inputCount == 0)
        return 0;

    if (r->logic == RULE_LOGIC_NOT)
    {
        // NOT hanya pakai input pertama
        uint8_t in = r->inputs[0];   // 1..11
        uint8_t state = (inputMask & (1U << (in - 1))) ? 1 : 0;
        return !state;               // output ON kalau input OFF
    }

    if (r->logic == RULE_LOGIC_OR)
    {
        // ON jika ADA satu input yang ON
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
        // ON jika SEMUA input ON
        for (uint8_t i = 0; i < r->inputCount; i++)
        {
            uint8_t in = r->inputs[i];
            if (!(inputMask & (1U << (in - 1))))
                return 0;  // ada yang OFF
        }
        return 1;
    }

    return 0;
}

void ApplyRules(uint16_t inputMask)
{
    for (int q = 0; q < NUM_OUTPUTS; q++)
    {
        uint8_t on = EvalOutputState(q, inputMask);

        HAL_GPIO_WritePin(
            g_outputs[q].Port,
            g_outputs[q].Pin,
            on ? GPIO_PIN_SET : GPIO_PIN_RESET
        );
    }
}


void PrintRuleStatus(void)
{
    char msg[128];

    // Cetak RAW JSON rules dari ESP
    HAL_UART_Transmit(&huart1,
        (uint8_t*)"[RULE] RAW JSON:\r\n",
        sizeof("[RULE] RAW JSON:\r\n")-1, 100);
    HAL_UART_Transmit(&huart1,
        (uint8_t*)ruleJsonRaw,
        strlen(ruleJsonRaw), 100);
    HAL_UART_Transmit(&huart1,
        (uint8_t*)"\r\n",
        2, 100);

    HAL_UART_Transmit(&huart1,
        (uint8_t*)"[RULE] Parsed rules:\r\n",
        sizeof("[RULE] Parsed rules:\r\n")-1, 100);

    for (int q = 0; q < NUM_OUTPUTS; q++)
    {
        RuleDef_t *r = &g_rules[q];

        const char *logicName = "OFF";
        if      (r->logic == RULE_LOGIC_OR)  logicName = "OR";
        else if (r->logic == RULE_LOGIC_AND) logicName = "AND";
        else if (r->logic == RULE_LOGIC_NOT) logicName = "NOT";

        int len = snprintf(msg, sizeof(msg), "  Q%d: %s (", q + 1, logicName);

        for (uint8_t i = 0; i < r->inputCount; i++)
        {
            len += snprintf(msg + len, sizeof(msg) - len,
                            "I%u%s",
                            r->inputs[i],
                            (i == r->inputCount - 1) ? "" : ",");
        }
        snprintf(msg + len, sizeof(msg) - len, ")\r\n");

        HAL_UART_Transmit(&huart1,
            (uint8_t*)msg, strlen(msg), 100);
    }
}


/*Timetemp set ESP*/
void ParseRTCJSON(const char *json)
{
    if (json == NULL) return;

    // cari key "rtc_set"
    const char *p = strstr(json, "\"rtc_set\"");
    if (!p) return;

    p = strchr(p, ':');
    if (!p) return;
    p++;  // setelah ':'

    // skip spasi dan tanda kutip
    while (*p == ' ' || *p == '\"') p++;

    char dt[32] = {0};
    int i = 0;
    while (*p && *p != '\"' && *p != '}' && i < (int)(sizeof(dt) - 1))
    {
        dt[i++] = *p++;
    }
    dt[i] = '\0';

    // Format yang kita harapkan: "YYYY-MM-DD HH:MM:SS"
    int Y, M, D, h, m, s;
    if (sscanf(dt, "%d-%d-%d %d:%d:%d", &Y, &M, &D, &h, &m, &s) != 6)
    {
        const char *msg = "[RTC] Bad datetime format\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
        return;
    }

    DS3231_TimeTypeDef t;
    t.year   = (uint16_t)Y;
    t.month  = (uint8_t)M;
    t.day    = (uint8_t)D;
    t.hours  = (uint8_t)h;
    t.minutes= (uint8_t)m;
    t.seconds= (uint8_t)s;

    // dayOfWeek bisa diabaikan atau diisi 1..7 sembarang
    t.dayOfWeek = 1;

    if (DS3231_SetTime(&t))
    {
        const char *ok = "[RTC] Time updated from ESP32\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)ok, strlen(ok), 100);
    }
    else
    {
        const char *er = "[RTC] DS3231_SetTime failed\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)er, strlen(er), 100);
    }
}

void OnRuleFrameReceived(uint8_t *payload, uint16_t len)
{
    if (len >= RX_BUF_SIZE)
        len = RX_BUF_SIZE - 1;

    memcpy(ruleJsonRaw, payload, len);
    ruleJsonRaw[len] = '\0';

    // Debug: tampilkan JSON mentah
    HAL_UART_Transmit(&huart1,
        (uint8_t*)"[STM] Frame from ESP:\r\n",
        sizeof("[STM] Frame from ESP:\r\n")-1, 100);
    HAL_UART_Transmit(&huart1,
        (uint8_t*)ruleJsonRaw,
        strlen(ruleJsonRaw), 100);
    HAL_UART_Transmit(&huart1,
        (uint8_t*)"\r\n",
        2, 100);

    if (strstr(ruleJsonRaw, "\"rules\""))
    {
        // frame untuk konfigurasi rules
        ParseRuleJSON(ruleJsonRaw);
        g_ruleUpdated = 1;
    }
    else if (strstr(ruleJsonRaw, "\"rtc_set\""))
    {
        // frame untuk set RTC
        ParseRTCJSON(ruleJsonRaw);
    }
    else
    {
        const char *msg = "[STM] Unknown JSON payload\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    }
}

// ---- CBOR helpers (subset yang kita butuh) ----
static size_t cbor_put_uint(uint8_t *buf, size_t max, uint32_t value)
{
    if (value <= 23) {
        if (max < 1) return 0;
        buf[0] = (uint8_t)(0x00 | value);
        return 1;
    } else if (value <= 0xFF) {
        if (max < 2) return 0;
        buf[0] = 0x18;
        buf[1] = (uint8_t)value;
        return 2;
    } else if (value <= 0xFFFF) {
        if (max < 3) return 0;
        buf[0] = 0x19;
        buf[1] = (uint8_t)(value >> 8);
        buf[2] = (uint8_t)value;
        return 3;
    } else {
        if (max < 5) return 0;
        buf[0] = 0x1A;
        buf[1] = (uint8_t)(value >> 24);
        buf[2] = (uint8_t)(value >> 16);
        buf[3] = (uint8_t)(value >> 8);
        buf[4] = (uint8_t)value;
        return 5;
    }
}

static size_t cbor_put_text(uint8_t *buf, size_t max, const char *str)
{
    size_t len = strlen(str);
    if (len > 0xFFFF) return 0;
    size_t idx = 0;

    if (len <= 23) {
        if (max < 1 + len) return 0;
        buf[idx++] = (uint8_t)(0x60 | len);
    } else if (len <= 0xFF) {
        if (max < 2 + len) return 0;
        buf[idx++] = 0x78;
        buf[idx++] = (uint8_t)len;
    } else {
        if (max < 3 + len) return 0;
        buf[idx++] = 0x79;
        buf[idx++] = (uint8_t)(len >> 8);
        buf[idx++] = (uint8_t)len;
    }
    memcpy(&buf[idx], str, len);
    return idx + len;
}

static size_t cbor_put_array_hdr(uint8_t *buf, size_t max, uint32_t count)
{
    if (count <= 23) {
        if (max < 1) return 0;
        buf[0] = (uint8_t)(0x80 | count);
        return 1;
    } else if (count <= 0xFF) {
        if (max < 2) return 0;
        buf[0] = 0x98;
        buf[1] = (uint8_t)count;
        return 2;
    }
    return 0;
}

static size_t cbor_put_map_hdr(uint8_t *buf, size_t max, uint32_t count)
{
    if (count <= 23) {
        if (max < 1) return 0;
        buf[0] = (uint8_t)(0xA0 | count);
        return 1;
    } else if (count <= 0xFF) {
        if (max < 2) return 0;
        buf[0] = 0xB8;
        buf[1] = (uint8_t)count;
        return 2;
    }
    return 0;
}

static size_t cbor_put_float32(uint8_t *buf, size_t max, float value)
{
    if (max < 5) return 0;
    buf[0] = 0xFA; // simple value, float32
    uint32_t u;
    memcpy(&u, &value, sizeof(u));
    buf[1] = (uint8_t)(u >> 24);
    buf[2] = (uint8_t)(u >> 16);
    buf[3] = (uint8_t)(u >> 8);
    buf[4] = (uint8_t)(u);
    return 5;
}

// Encode Measurement_t -> CBOR map dengan 6 key: timestamp, env, dig_in, an_in, q, seq
size_t CBOR_EncodeMeasurement(const Measurement_t *m, uint8_t *out, size_t max_len)
{
    size_t idx = 0, n;

    // map(6)
    n = cbor_put_map_hdr(&out[idx], max_len - idx, 6);
    if (!n) return 0;
    idx += n;

    // "timestamp": "YYYY-..Z"
    char ts[32];
    snprintf(ts, sizeof(ts), "%04u-%02u-%02uT%02u:%02u:%02uZ",
             m->rtc.year, m->rtc.month, m->rtc.day,
             m->rtc.hours, m->rtc.minutes, m->rtc.seconds);

    n = cbor_put_text(&out[idx], max_len - idx, "timestamp");
    if (!n) return 0;
    idx += n;
    n = cbor_put_text(&out[idx], max_len - idx, ts);
    if (!n) return 0;
    idx += n;

    // "env": { "temp":..., "hum":... }
    n = cbor_put_text(&out[idx], max_len - idx, "env");
    if (!n) return 0;
    idx += n;
    n = cbor_put_map_hdr(&out[idx], max_len - idx, 2);
    if (!n) return 0;
    idx += n;

    n = cbor_put_text(&out[idx], max_len - idx, "temp");
    if (!n) return 0;
    idx += n;
    n = cbor_put_float32(&out[idx], max_len - idx, m->temp);
    if (!n) return 0;
    idx += n;

    n = cbor_put_text(&out[idx], max_len - idx, "hum");
    if (!n) return 0;
    idx += n;
    n = cbor_put_float32(&out[idx], max_len - idx, m->hum);
    if (!n) return 0;
    idx += n;

    // "dig_in": [..]
    n = cbor_put_text(&out[idx], max_len - idx, "dig_in");
    if (!n) return 0;
    idx += n;
    n = cbor_put_array_hdr(&out[idx], max_len - idx, NUM_INPUTS);
    if (!n) return 0;
    idx += n;
    for (int i = 0; i < NUM_INPUTS; i++) {
        n = cbor_put_uint(&out[idx], max_len - idx, m->dig_in[i]);
        if (!n) return 0;
        idx += n;
    }

    // "an_in": [..]
    n = cbor_put_text(&out[idx], max_len - idx, "an_in");
    if (!n) return 0;
    idx += n;
    n = cbor_put_array_hdr(&out[idx], max_len - idx, NUM_ANALOG);
    if (!n) return 0;
    idx += n;
    for (int i = 0; i < NUM_ANALOG; i++) {
        n = cbor_put_uint(&out[idx], max_len - idx, m->an_in[i]);
        if (!n) return 0;
        idx += n;
    }

    // "q": [..]
    n = cbor_put_text(&out[idx], max_len - idx, "q");
    if (!n) return 0;
    idx += n;
    n = cbor_put_array_hdr(&out[idx], max_len - idx, NUM_OUTPUTS);
    if (!n) return 0;
    idx += n;
    for (int i = 0; i < NUM_OUTPUTS; i++) {
        n = cbor_put_uint(&out[idx], max_len - idx, m->q[i]);
        if (!n) return 0;
        idx += n;
    }

    // "seq": m->seq
    n = cbor_put_text(&out[idx], max_len - idx, "seq");
    if (!n) return 0;
    idx += n;
    n = cbor_put_uint(&out[idx], max_len - idx, m->seq);
    if (!n) return 0;
    idx += n;

    return idx;
}

static bool cbor_skip_text(const uint8_t *buf, size_t buf_len, size_t *consumed)
{
    if (buf_len == 0) return false;
    uint8_t ib = buf[0];
    if ((ib & 0xE0) != 0x60) return false; // major type 3 (text)

    uint8_t ai = ib & 0x1F;
    size_t hdr, len;
    if (ai <= 23) {
        hdr = 1;
        len = ai;
    } else if (ai == 24) {
        if (buf_len < 2) return false;
        hdr = 2;
        len = buf[1];
    } else if (ai == 25) {
        if (buf_len < 3) return false;
        hdr = 3;
        len = ((size_t)buf[1] << 8) | buf[2];
    } else {
        return false; // kita tidak butuh yang lebih besar
    }
    if (hdr + len > buf_len) return false;
    *consumed = hdr + len;
    return true;
}

static bool cbor_read_uint32(const uint8_t *buf, size_t buf_len,
                             size_t *consumed, uint32_t *out)
{
    if (buf_len == 0) return false;
    uint8_t ib = buf[0];
    if ((ib & 0xE0) != 0x00) return false; // major type 0 (unsigned int)
    uint8_t ai = ib & 0x1F;

    if (ai <= 23) {
        *out = ai;
        *consumed = 1;
        return true;
    } else if (ai == 24) {
        if (buf_len < 2) return false;
        *out = buf[1];
        *consumed = 2;
        return true;
    } else if (ai == 25) {
        if (buf_len < 3) return false;
        *out = ((uint32_t)buf[1] << 8) | buf[2];
        *consumed = 3;
        return true;
    } else if (ai == 26) {
        if (buf_len < 5) return false;
        *out = ((uint32_t)buf[1] << 24) |
               ((uint32_t)buf[2] << 16) |
               ((uint32_t)buf[3] << 8)  |
               buf[4];
        *consumed = 5;
        return true;
    }
    return false;
}

static bool cbor_skip_float32(const uint8_t *buf, size_t buf_len, size_t *consumed)
{
    if (buf_len < 5) return false;
    if (buf[0] != 0xFA) return false; // simple float32
    *consumed = 5;
    return true;
}

static bool cbor_skip_uint_array(const uint8_t *buf, size_t buf_len,
                                 uint32_t expected_count, size_t *consumed)
{
    if (buf_len == 0) return false;
    uint8_t ib = buf[0];
    if ((ib & 0xE0) != 0x80) return false; // major type 4 (array)
    uint8_t ai = ib & 0x1F;
    if (ai != expected_count) return false; // kita pakai count fix <= 23

    size_t idx = 1;
    for (uint32_t i = 0; i < expected_count; i++) {
        size_t c; uint32_t dummy;
        if (!cbor_read_uint32(&buf[idx], buf_len - idx, &c, &dummy)) return false;
        idx += c;
    }
    *consumed = idx;
    return true;
}

void App_BackgroundSensors(uint32_t now)
{
    static uint8_t  initialized   = 0;
    static uint32_t lastDhtMs     = 0;
    static uint16_t stableMask    = 0;
    static uint16_t lastRawMask   = 0;
    static uint32_t lastChangeMs  = 0;

    if (!initialized)
    {
        uint16_t mask0 = 0;
        for (int i = 0; i < NUM_INPUTS; i++) {
            GPIO_PinState s = HAL_GPIO_ReadPin(g_inputs[i].Port, g_inputs[i].Pin);
            if (s == GPIO_PIN_RESET) mask0 |= (1 << i);
        }
        stableMask        = mask0;
        lastRawMask       = mask0;
        g_inputMaskStable = mask0;
        lastChangeMs      = now;
        lastDhtMs         = now - 2000U;
        initialized       = 1;
    }

    // Update suhu & kelembaban setiap 2s
    if ((now - lastDhtMs) >= 2000U)
    {
        float t_tmp, h_tmp;
        if (DHT22_Read(&t_tmp, &h_tmp)) {
            temp = t_tmp;
            hum  = h_tmp;
        }
        lastDhtMs = now;
    }

    // ADS background (tetap pakai fungsi Anda)
    ADS_Task();

    // Baca RAW input
    uint16_t rawMask = 0;
    for (int i = 0; i < NUM_INPUTS; i++) {
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
        g_inputMaskStable = stableMask;
        g_io_changed_flag = 1;        // trigger acquisition
    }

    // Terapkan rules (logic output) seperti sebelumnya
    static uint16_t lastAppliedMask = 0xFFFF;
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
}

static void Acquire_FillMeasurement(Measurement_t *m,
                                    const DS3231_TimeTypeDef *rtc)
{
    m->seq = g_nextSeq;
    m->rtc = *rtc;      // copy struct RTC dari caller

    m->temp = temp;
    m->hum  = hum;

    uint16_t mask = g_inputMaskStable;
    for (int i = 0; i < NUM_INPUTS; i++) {
        m->dig_in[i] = (mask & (1U << i)) ? 1U : 0U;
    }

    for (int i = 0; i < NUM_ANALOG; i++) {
        m->an_in[i] = g_anCache[i];
    }

    for (int i = 0; i < NUM_OUTPUTS; i++) {
        m->q[i] = (HAL_GPIO_ReadPin(g_outputs[i].Port, g_outputs[i].Pin) == GPIO_PIN_SET) ? 1U : 0U;
    }
}

void App_HandleAcquisition(uint32_t now)
{
    // ===== STATE VARIABLES =====
    static uint32_t lastLogRtcEpoch   = 0;   // Epoch RTC terakhir saat log
    static uint32_t lastRtcCheckTick  = 0;   // Tick terakhir saat RTC dibaca
    static uint32_t lastRtcEpochValue = 0;   // Nilai epoch RTC terakhir (untuk deteksi stuck)
    static uint8_t  rtcStuckCount     = 0;   // Counter jika RTC tidak berubah
    static uint8_t  firstRun          = 1;   // Flag log pertama

    bool needLog = false;
    bool byIo    = false;

    if (!g_sd_mounted)
        return;

    // ===== 1) TRIGGER: I/O CHANGE =====
    if (g_io_changed_flag) {
        g_io_changed_flag = 0;
        needLog = true;
        byIo    = true;
    }

    // ===== 2) BACA RTC =====
    DS3231_TimeTypeDef rtc;
    uint8_t rtcOk = DS3231_ReadTime(&rtc);

    if (!rtcOk) {
        // RTC gagal dibaca - gunakan fallback HAL_GetTick
        UART_Print("[ACQ] RTC read failed!\r\n");

        // Fallback: log setiap 60 detik berdasarkan HAL_GetTick
        if (!needLog && (now - lastRtcCheckTick) >= DELIVERY_TIME_INTERVAL) {
            needLog = true;
            byIo    = false;
            lastRtcCheckTick = now;
        }

        // Isi RTC dengan dummy untuk timestamp
        memset(&rtc, 0, sizeof(rtc));
        rtc.year  = 2000;
        rtc.month = 1;
        rtc.day   = 1;
    }
    else
    {
        // RTC OK - hitung epoch
        uint32_t nowEpoch = RTC_ToEpochSeconds(&rtc);

        // ===== 3) DETEKSI RTC STUCK =====
        // Cek apakah RTC berubah dalam 2 detik terakhir
        if ((now - lastRtcCheckTick) >= 2000) {
            if (nowEpoch == lastRtcEpochValue) {
                rtcStuckCount++;
                if (rtcStuckCount >= 3) {
                    UART_Print("[ACQ] WARN: RTC stuck!\r\n");
                }
            } else {
                rtcStuckCount = 0;  // Reset counter
            }
            lastRtcEpochValue = nowEpoch;
            lastRtcCheckTick  = now;
        }

        // ===== 4) TRIGGER: FIRST RUN =====
        if (firstRun) {
            needLog         = true;
            byIo            = false;
            firstRun        = 0;
            lastLogRtcEpoch = nowEpoch;
        }

        // ===== 5) TRIGGER: INTERVAL 60 DETIK (RTC-BASED) =====
        if (!needLog) {
            // Jika RTC stuck, gunakan HAL_GetTick sebagai fallback
            if (rtcStuckCount >= 3) {
                // Fallback ke HAL_GetTick
                static uint32_t fallbackTick = 0;
                if (fallbackTick == 0) fallbackTick = now;

                if ((now - fallbackTick) >= DELIVERY_TIME_INTERVAL) {
                    needLog = true;
                    byIo    = false;
                    fallbackTick = now;
                }
            }
            else {
                // Normal: gunakan RTC epoch
                if ((nowEpoch - lastLogRtcEpoch) >= 60U) {
                    needLog = true;
                    byIo    = false;
                }
            }
        }

        // Update last epoch jika akan log
        if (needLog) {
            lastLogRtcEpoch = nowEpoch;
        }
    }

    // ===== 6) TIDAK PERLU LOG? RETURN =====
    if (!needLog)
        return;

    // ===== 7) BUAT MEASUREMENT =====
    Measurement_t m;
    Acquire_FillMeasurement(&m, &rtc);

    // ===== 8) ENCODE CBOR =====
    uint8_t cbor[256];
    size_t cbor_len = CBOR_EncodeMeasurement(&m, cbor, sizeof(cbor));
    if (cbor_len == 0) {
        UART_Print("[ACQ] CBOR encode failed\r\n");
        return;
    }

    // ===== 9) SIMPAN KE LOG FILE =====
    if (Log_AppendRecord(cbor, cbor_len)) {
        g_nextSeq++;
        Debug_PrintMeasurementJSON(&m, byIo);
    } else {
        UART_Print("[ACQ] Log append failed!\r\n");
    }
}

void Log_InitRuntime(void)
{
    g_sd_mounted = 0;
    g_nextSeq    = 1;
}

bool SD_MountTask(void)
{
    if (g_sd_mounted) return true;

    FRESULT fr = f_mount(&USERFatFS, (TCHAR const*)USERPath, 1);
    if (fr == FR_OK) {
        g_sd_mounted = 1;
        UART_Print("SD_MountTask: mounted OK\r\n");
        return true;
    } else {
        char msg[32];
        sprintf(msg, "SD_MountTask: mount ERR=%d\r\n", fr);
        UART_Print(msg);
    }
    return false;
}

bool Log_RecoverTask(void)
{
    static uint8_t done = 0;
    if (done) return true;
    if (!g_sd_mounted) return false;

    FIL file;
    FILINFO fi;
    FRESULT fr;

    // 1) Tangani log.new vs log.cbor
    int log_exists  = (f_stat(LOG_FILE_NAME, &fi) == FR_OK);
    int temp_exists = (f_stat("log.new", &fi) == FR_OK);

    if (!log_exists && temp_exists) {
        f_rename("log.new", LOG_FILE_NAME);
        log_exists = 1;
    } else if (log_exists && temp_exists) {
        f_unlink("log.new");
    }

    if (!log_exists) {
        fr = f_open(&file, LOG_FILE_NAME, FA_WRITE | FA_CREATE_ALWAYS);
        if (fr == FR_OK) f_close(&file);
    }

    // pastikan error.cbor ada
    if (f_stat(ERROR_FILE_NAME, &fi) != FR_OK) {
        fr = f_open(&file, ERROR_FILE_NAME, FA_WRITE | FA_CREATE_ALWAYS);
        if (fr == FR_OK) f_close(&file);
    }

    // 2) Scan log.cbor untuk cari max seq
    fr = f_open(&file, LOG_FILE_NAME, FA_READ);
    if (fr != FR_OK) {
        g_nextSeq = 1;
        done = 1;
        return true;
    }

    uint8_t  buf[256];
    uint16_t len;
    uint32_t seq;
    uint32_t max_seq = 0;

    while (Log_ReadNextRecordFromFile(&file, &seq, buf, &len)) {
        if (seq > max_seq) max_seq = seq;
    }

    f_close(&file);

    g_nextSeq = (max_seq == 0) ? 1 : (max_seq + 1);

    done = 1;
    UART_Print("Log_RecoverTask: done, nextSeq=");
    char msg[32];
    sprintf(msg, "%lu\r\n", (unsigned long)g_nextSeq);
    UART_Print(msg);
    return true;
}

bool Log_AppendRecord(const uint8_t *cbor, size_t len)
{
    FIL f;
    FRESULT fr = f_open(&f, LOG_FILE_NAME, FA_WRITE | FA_OPEN_APPEND);
    if (fr != FR_OK) return false;

    UINT bw;
    fr = f_write(&f, cbor, len, &bw);
    f_sync(&f);
    f_close(&f);

    return (fr == FR_OK && bw == len);
}

bool Log_AppendErrorRecord(const uint8_t *cbor, size_t len)
{
    FIL f;
    FRESULT fr = f_open(&f, ERROR_FILE_NAME, FA_WRITE | FA_OPEN_APPEND);
    if (fr != FR_OK) return false;

    UINT bw;
    fr = f_write(&f, cbor, len, &bw);
    f_sync(&f);
    f_close(&f);

    return (fr == FR_OK && bw == len);
}

bool Log_ReadHeadRecord(uint32_t *out_seq,
                        uint8_t *out_cbor,
                        uint16_t *out_len)
{
    FIL f;
    FRESULT fr = f_open(&f, LOG_FILE_NAME, FA_READ);
    if (fr != FR_OK) return false;

    f_lseek(&f, 0);
    bool ok = Log_ReadNextRecordFromFile(&f, out_seq, out_cbor, out_len);
    f_close(&f);
    return ok;
}

bool Log_TruncateUpToSeq(uint32_t upto_seq)
{
    FIL f_in, f_out;
    FRESULT fr;

    fr = f_open(&f_in, LOG_FILE_NAME, FA_READ);
    if (fr != FR_OK) return false;

    fr = f_open(&f_out, "log.new", FA_WRITE | FA_CREATE_ALWAYS);
    if (fr != FR_OK) {
        f_close(&f_in);
        return false;
    }

    uint8_t  buf[256];
    uint16_t len;
    uint32_t seq;

    while (Log_ReadNextRecordFromFile(&f_in, &seq, buf, &len)) {
        if (seq > upto_seq) {
            UINT bw;
            fr = f_write(&f_out, buf, len, &bw);
            if (fr != FR_OK || bw != len) {
                f_close(&f_in);
                f_close(&f_out);
                f_unlink("log.new");
                return false;
            }
        }
    }

    f_sync(&f_out);
    f_close(&f_in);
    f_close(&f_out);

    f_unlink(LOG_FILE_NAME);
    fr = f_rename("log.new", LOG_FILE_NAME);
    if (fr != FR_OK) {
        // recovery: di boot berikutnya Log_RecoverTask akan deteksi log.new
        return false;
    }

    return true;
}

bool Log_ReadNextRecordFromFile(FIL *pf, uint32_t *out_seq,
                                uint8_t *out_cbor, uint16_t *out_len)
{
    UINT   br;
    uint8_t tmp[256];
    DWORD  pos = f_tell(pf);

    FRESULT fr = f_read(pf, tmp, sizeof(tmp), &br);
    if (fr != FR_OK || br == 0) {
        return false; // EOF atau error
    }

    size_t idx = 0;
    if (tmp[0] != 0xA6) {   // map(6)
        return false;
    }
    idx = 1;

    uint32_t seq_val = 0;

    for (int pair = 0; pair < 6; pair++) {
        // --- parse key (text) ---
        if (idx >= br) return false;
        uint8_t key_hdr = tmp[idx];
        if ((key_hdr & 0xE0) != 0x60) return false; // bukan text
        uint8_t key_len = key_hdr & 0x1F;
        if (key_len > (br - idx - 1)) return false;
        idx++;
        const uint8_t *key_ptr = &tmp[idx];
        idx += key_len;

        const uint8_t *val_ptr = &tmp[idx];
        size_t val_len = br - idx;
        size_t c;

        // --- cek nama key & skip value sesuai tipe ---
        if (key_len == 9 && memcmp(key_ptr, "timestamp", 9) == 0) {
            if (!cbor_skip_text(val_ptr, val_len, &c)) return false;
            idx += c;

        } else if (key_len == 3 && memcmp(key_ptr, "env", 3) == 0) {
            // env: map(2) { "temp": float32, "hum": float32 }
            if (val_len == 0 || val_ptr[0] != 0xA2) return false;
            size_t j = 1;

            if (!cbor_skip_text(&val_ptr[j], val_len - j, &c)) return false; // "temp"
            j += c;
            if (!cbor_skip_float32(&val_ptr[j], val_len - j, &c)) return false;
            j += c;

            if (!cbor_skip_text(&val_ptr[j], val_len - j, &c)) return false; // "hum"
            j += c;
            if (!cbor_skip_float32(&val_ptr[j], val_len - j, &c)) return false;
            j += c;

            idx += j;

        } else if (key_len == 6 && memcmp(key_ptr, "dig_in", 6) == 0) {
            if (!cbor_skip_uint_array(val_ptr, val_len, NUM_INPUTS, &c)) return false;
            idx += c;

        } else if (key_len == 5 && memcmp(key_ptr, "an_in", 5) == 0) {
            if (!cbor_skip_uint_array(val_ptr, val_len, NUM_ANALOG, &c)) return false;
            idx += c;

        } else if (key_len == 1 && memcmp(key_ptr, "q", 1) == 0) {
            if (!cbor_skip_uint_array(val_ptr, val_len, NUM_OUTPUTS, &c)) return false;
            idx += c;

        } else if (key_len == 3 && memcmp(key_ptr, "seq", 3) == 0) {
            if (!cbor_read_uint32(val_ptr, val_len, &c, &seq_val)) return false;
            idx += c;

        } else {
            return false; // key tak dikenal
        }
    }

    if (idx > br) return false;

    memcpy(out_cbor, tmp, idx);
    *out_len = (uint16_t)idx;
    *out_seq = seq_val;

    // majukan file pointer ke awal record berikutnya
    f_lseek(pf, pos + idx);
    return true;
}

void Tx_InitRuntime(TxContext_t *ctx)
{
    memset(ctx, 0, sizeof(*ctx));
}

bool Tx_StartFrameSend(TxContext_t *ctx)
{
    if (!g_uart2_tx_done) return false;

    uint16_t idx = 0;
    g_uart2_tx_buf[idx++] = SOF_BYTE;
    g_uart2_tx_buf[idx++] = 0x01;       // TYPE = data
    g_uart2_tx_buf[idx++] = 0x10;       // CMD

    uint16_t len = ctx->cbor_len;
    g_uart2_tx_buf[idx++] = (len >> 8) & 0xFF;
    g_uart2_tx_buf[idx++] = (len & 0xFF);

    memcpy(&g_uart2_tx_buf[idx], ctx->cbor_buf, len);
    idx += len;

    uint16_t crc = CRC16_Modbus(g_uart2_tx_buf, idx);
    g_uart2_tx_buf[idx++] = (crc >> 8) & 0xFF;
    g_uart2_tx_buf[idx++] = crc & 0xFF;

    g_uart2_tx_len  = idx;
    g_uart2_tx_done = 0;

    if (HAL_UART_Transmit_IT(&huart2, g_uart2_tx_buf, g_uart2_tx_len) != HAL_OK) {
        g_uart2_tx_done = 1;
        return false;
    }
    return true;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) {
        g_uart2_tx_done = 1;
    }
}

void Tx_StateMachine_Run(TxState_t *state, TxContext_t *ctx, uint32_t now)
{
    switch (*state)
    {
    case TX_STATE_IDLE:
        if (!g_sd_mounted) break;

        if (Log_ReadHeadRecord(&ctx->seq, ctx->cbor_buf, &ctx->cbor_len)) {
            ctx->retry_count = 0;
            *state = TX_STATE_SEND_FRAME;
        }
        break;

    case TX_STATE_SEND_FRAME:
        if (Tx_StartFrameSend(ctx)) {
            ctx->ack_deadline_ms = now + ACK_TIMEOUT_MS;
            *state = TX_STATE_WAIT_TX_COMPLETE;
        }
        break;

    case TX_STATE_WAIT_TX_COMPLETE:
        if (g_uart2_tx_done) {
            *state = TX_STATE_WAIT_ACK;
        }
        break;

    case TX_STATE_WAIT_ACK:
        if (g_ack_pending) {
            uint32_t ack_seq = g_ack_seq;
            g_ack_pending = 0;

            if (ack_seq == ctx->seq) {
                *state = TX_STATE_ACK_OK;
            } else if (ack_seq < ctx->seq) {
                // ack lama, abaikan
            } else {
                // aneh, treat sebagai NACK
                *state = TX_STATE_RETRY_OR_FAIL;
            }
        } else if ((int32_t)(now - ctx->ack_deadline_ms) >= 0) {
            *state = TX_STATE_RETRY_OR_FAIL;
        }
        break;

    case TX_STATE_ACK_OK:
        if (Log_TruncateUpToSeq(ctx->seq)) {
            *state = TX_STATE_IDLE;
        }
        break;

    case TX_STATE_RETRY_OR_FAIL:
        ctx->retry_count++;
        if (ctx->retry_count <= TX_MAX_RETRY) {
            *state = TX_STATE_SEND_FRAME;
        } else {
            *state = TX_STATE_MOVE_TO_ERROR;
        }
        break;

    case TX_STATE_MOVE_TO_ERROR:
        if (Log_AppendErrorRecord(ctx->cbor_buf, ctx->cbor_len)) {
            if (Log_TruncateUpToSeq(ctx->seq)) {
                *state = TX_STATE_IDLE;
            }
        }
        break;

    default:
        *state = TX_STATE_IDLE;
        break;
    }
}

static void Debug_PrintMeasurementJSON(const Measurement_t *m, bool trigByIo)
{
    char line[320];
    char ts[32];
    char dinStr[64];
    char *p;
    int  rem;

    // Timestamp ISO8601 dengan Z
    snprintf(ts, sizeof(ts), "%04u-%02u-%02uT%02u:%02u:%02uZ",
             m->rtc.year, m->rtc.month, m->rtc.day,
             m->rtc.hours, m->rtc.minutes, m->rtc.seconds);

    // Build JSON array untuk dig_in
    p   = dinStr;
    rem = sizeof(dinStr);
    *p++ = '['; rem--;
    for (int i = 0; i < NUM_INPUTS; i++) {
        int n = snprintf(p, rem, "%u%s",
                         m->dig_in[i],
                         (i < NUM_INPUTS - 1) ? "," : "");
        if (n < 0 || n >= rem) break;
        p   += n;
        rem -= n;
    }
    *p++ = ']';
    *p   = '\0';

    // Prefix: [ACQ][IO] atau [ACQ][TMR]
    const char *src = trigByIo ? "[ACQ][IO] " : "[ACQ][TMR] ";

    int len = snprintf(line, sizeof(line),
        "%s"
        "{\"timestamp\":\"%s\","
        "\"env\":{\"temp\":%.1f,\"hum\":%.1f},"
        "\"dig_in\":%s,"
        "\"an_in\":[%u,%u,%u,%u],"
        "\"q\":[%u,%u,%u,%u],"
        "\"seq\":%lu}\r\n",
        src,
        ts,
        m->temp, m->hum,
        dinStr,
        m->an_in[0], m->an_in[1], m->an_in[2], m->an_in[3],
        m->q[0], m->q[1], m->q[2], m->q[3],
        (unsigned long)m->seq
    );

    if (len > 0 && len < (int)sizeof(line)) {
        UART_Print(line);    // kirim ke UART1
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
