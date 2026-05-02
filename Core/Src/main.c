/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// BQ76952 I2C Adresi (Varsayılan 0x10, STM32 için sola kaydırılır -> 0x20)
#define BQ_I2C_ADDR  0x20

// BQ76952 Direct Command (Doğrudan Okuma) Adresleri
#define BQ_VCELL1    0x14  // Hücre 1 Başlangıç
#define BQ_CURRENT   0x3A  // CC2 Akım Değeri
#define BQ_TEMP_TS1  0x70  // NTC 1
#define BQ_TEMP_TS2  0x72  // NTC 2
#define BQ_TEMP_TS3  0x74  // NTC 3

// Güç / SOC Sabitleri
#define MAX_POWER   1108.8f
#define MIN_POWER    712.8f
#define FLASH_USER_START_ADDR   0x0801F800

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

uint32_t counter;
const uint32_t ARALIK  = 99;
const uint32_t KARALIK = 30;

uint8_t percent = 0, percentholder = 0, bmshata = 0;
uint8_t AKSdata[2] = {0xFF, 0x00};

float temp1, temp2, temp3, akim = 0.0f, cellV[17] = {0}, batvolt = 0.0f;
float maxtemp = 0.0f, power = 0.0f, batlevel = 0.0f, minCV = 5.0f, maxCV = 0.0f;
float akimT = 0.0f, tempT = 0.0f, secwh = 0.0f;
float powerholder = 0.0f;

bool charging = false, ms500 = false, chold = false;
bool UVerror = false, OVerror = false, startup = true;
bool problem = false, danger = false, currerror = false, temperror = false;

int bal = 0, c = 0, k = 0, dc = 0, m = 0, f = 0, q = 0, msx = 0, dead = 0, probz = 0, mss = 0;
uint16_t CANakim = 0, CANtemp = 0, CANvolt = 0, CANminCV = 0, CANmaxCV = 0;

extern FDCAN_HandleTypeDef hfdcan1;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

void AnalogWrite_Fan(uint8_t duty);
void Save_Power_To_Flash(float power_val);
float Read_Power_From_Flash(void);

void readV(void);
void readC(void);
void readtemp(void);
void calcminmax(void);
void calctotalV(void);
void calcpower(void);
void tempcontrol(void);
void checkprob(void);
void chargecheck(void);
void CANparse(void);
void CANvoltsend(void);
void CANdatasend(void);
void CANdatatake(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

void AnalogWrite_Fan(uint8_t duty) {
    if (duty > 100) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    else            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
}

void Save_Power_To_Flash(float power_val) {
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError;
    EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Banks       = FLASH_BANK_1;
    EraseInitStruct.Page        = 63;
    EraseInitStruct.NbPages     = 1;

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) == HAL_OK) {
        uint64_t data_to_write = 0;
        memcpy(&data_to_write, &power_val, sizeof(float));
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLASH_USER_START_ADDR, data_to_write);
    }
    HAL_FLASH_Lock();
}

float Read_Power_From_Flash(void) {
    float read_val = 0.0f;
    uint32_t flash_word = *(__IO uint32_t*)FLASH_USER_START_ADDR;
    if (flash_word != 0xFFFFFFFF) memcpy(&read_val, &flash_word, sizeof(float));
    return read_val;
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_USB_PCD_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // Kontaktör Başlangıç Durumları
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET); // PB2 Şarj
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); // PB1 Deşarj

      printf("-------------------- PEHLIVAN TEAM --------------------\r\n");
      printf("-------- BQ76952 Batarya Yonetim Sistemi Aktif --------\r\n");

      powerholder = Read_Power_From_Flash();
      if ((powerholder < MAX_POWER) && (powerholder > MIN_POWER)) {
          batlevel = powerholder;
      } else {
          batlevel = 1008.1f;
      }

      // FDCAN BAŞLATMA
      FDCAN_FilterTypeDef sFilterConfig;
      sFilterConfig.IdType = FDCAN_STANDARD_ID;
      sFilterConfig.FilterIndex = 0;
      sFilterConfig.FilterType = FDCAN_FILTER_MASK;
      sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
      sFilterConfig.FilterID1 = 0x000;
      sFilterConfig.FilterID2 = 0x000;
      HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
      HAL_FDCAN_Start(&hfdcan1);


      counter = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
      while (1)
            {
                uint32_t currentMillis = HAL_GetTick();
                if (currentMillis - counter >= ARALIK) {
                    c++;
                    if (c > 4) { c = 0; ms500 = true; }
                }

                if (c - k > 0) {
                    readtemp(); readC(); checkprob(); tempcontrol();
                    CANvoltsend(); CANdatasend();
                    k = c; counter = HAL_GetTick();
                }

                if (ms500) {
                    chargecheck();
                    readV();
                    counter = counter - KARALIK;
                    calcminmax(); calctotalV(); readtemp(); readC();
                    checkprob(); tempcontrol(); calcpower();

                    akimT += akim; tempT += maxtemp;
                    ms500 = false; k = 0; dc++; counter = HAL_GetTick();
                }

                if (dc >= 2) {
                    CANparse(); CANdatasend();
                    dc = 0; akimT = 0.0f; tempT = 0.0f;
                    if (AKSdata[0] == 0xF0) { bal++; chold = charging; }
                }

                CANdatatake();
                checkprob();
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 16;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 1;
  hfdcan1.Init.NominalTimeSeg2 = 1;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

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
  hi2c1.Init.Timing = 0x10805D88;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
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
  hi2c2.Init.Timing = 0x10805D88;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA5 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void readV(void) {
    uint8_t rxBuf[32]; // 16 Hücre x 2 Byte = 32 Byte Veri
    // BQ76952, hücre voltajlarını 0x14'ten 0x33'e kadar hazır tutar (mV cinsinden)
    if (HAL_I2C_Mem_Read(&hi2c1, BQ_I2C_ADDR, BQ_VCELL1, I2C_MEMADD_SIZE_8BIT, rxBuf, 32, 100) == HAL_OK) {
        for(int i = 0; i < 16; i++) {
            uint16_t rawV = rxBuf[i*2] | (rxBuf[(i*2)+1] << 8); // Little-Endian formatını birleştir
            cellV[i+1] = rawV / 1000.0f; // mV değerini Volt'a çevir
        }
    }
}

void readC(void) {
    uint8_t rxBuf[2];
    // BQ76952'nin CC2 (veya yapılandırmaya göre Akım) okuma adresi
    if (HAL_I2C_Mem_Read(&hi2c1, BQ_I2C_ADDR, BQ_CURRENT, I2C_MEMADD_SIZE_8BIT, rxBuf, 2, 100) == HAL_OK) {
        int16_t rawC = rxBuf[0] | (rxBuf[1] << 8);
        // CC Gain ayarlarına göre bu değeri Amper'e bölmeniz gerekebilir
        // Şimdilik standart mV/mA formunda okuyup mutlak değere alıyoruz
        akim = rawC / 1000.0f;
        if(akim < 0.0f) akim = -akim;
    }

    // Akım Hata Kontrolü
    bmshata &= ~(1 << 0);
    if (akim > 20.0f) { bmshata |= (1 << 2); } else { bmshata &= ~(1 << 2); }
    if (akim > 24.0f) { if (++m > 20) { currerror = true; bmshata |= (1 << 4); } else { currerror = false; bmshata &= ~(1 << 4); } } else { m = 0; }
}

void readtemp(void) {
    uint8_t rxBuf[2];
    float t1 = 0, t2 = 0, t3 = 0;

    // BQ76952 Sıcaklıkları 0.1 Kelvin cinsinden verir. (°C = Kelvin - 273.15)
    if (HAL_I2C_Mem_Read(&hi2c1, BQ_I2C_ADDR, BQ_TEMP_TS1, I2C_MEMADD_SIZE_8BIT, rxBuf, 2, 100) == HAL_OK) {
        t1 = ((rxBuf[0] | (rxBuf[1] << 8)) / 10.0f) - 273.15f;
    }
    if (HAL_I2C_Mem_Read(&hi2c1, BQ_I2C_ADDR, BQ_TEMP_TS2, I2C_MEMADD_SIZE_8BIT, rxBuf, 2, 100) == HAL_OK) {
        t2 = ((rxBuf[0] | (rxBuf[1] << 8)) / 10.0f) - 273.15f;
    }
    if (HAL_I2C_Mem_Read(&hi2c1, BQ_I2C_ADDR, BQ_TEMP_TS3, I2C_MEMADD_SIZE_8BIT, rxBuf, 2, 100) == HAL_OK) {
        t3 = ((rxBuf[0] | (rxBuf[1] << 8)) / 10.0f) - 273.15f;
    }

    maxtemp = t1;
    if (t2 > maxtemp) maxtemp = t2;
    if (t3 > maxtemp) maxtemp = t3;
}
// ════════════════════════════════════════════════════════════════

void calcminmax(void) {
    maxCV = 0.0f; minCV = 5.0f;
    for (int i = 1; i < 17; i++) {
        if (cellV[i] > maxCV) { maxCV = cellV[i]; }
        if (cellV[i] < minCV) { minCV = cellV[i]; }
    }
    if (minCV < 2.73f) { if (++f > 5) { UVerror = true; bmshata |= (1 << 7); } else { UVerror = false; bmshata &= ~(1 << 7); } } else { f = 0; UVerror = false; bmshata &= ~(1 << 7); }
    if (maxCV > 4.19f) { if (++q > 5) { OVerror = true; bmshata |= (1 << 5); } else { OVerror = false; bmshata &= ~(1 << 5); } } else { q = 0; OVerror = false; bmshata &= ~(1 << 5); }
}

void calctotalV(void) { batvolt = 0.0f; for (int i = 1; i < 17; i++) batvolt += cellV[i]; }

void tempcontrol(void) {
    if (!charging) {
        if (maxtemp > 40.0f && maxtemp < 50.0f) { AnalogWrite_Fan(255); bmshata &= ~(1 << 3); bmshata &= ~(1 << 6); dead = 0; }
        else if (maxtemp >= 50.0f && maxtemp <= 65.0f) { mss = 0; CANdatasend(); AnalogWrite_Fan(255); bmshata |= (1 << 3); bmshata &= ~(1 << 6); dead = 0; probz = 0; }
        else if (maxtemp < 38.0f) { AnalogWrite_Fan(0); bmshata &= ~(1 << 3); bmshata &= ~(1 << 6); dead = 0; probz = 0; }
        else if (maxtemp > 65.0f) { AnalogWrite_Fan(255); temperror = true; bmshata |= (1 << 3); bmshata |= (1 << 6); dead = 0; }
        if (maxtemp > 75.0f) { temperror = true; bmshata |= (1 << 3); bmshata |= (1 << 6); dead++; }
        if (dead > 4) danger = true;
        if (probz > 2) problem = true;
    } else {
        AnalogWrite_Fan(255);
        if (maxtemp > 45.0f) { problem = true; }
    }
}

void calcpower(void) {
    power = akim * batvolt; secwh = power / 3600.0f; batlevel = 16.5f * batvolt;
    percentholder = percent; percent = (uint8_t)(((batlevel - MIN_POWER) / (MAX_POWER - MIN_POWER)) * 100.0f);
    if (percentholder != percent) { Save_Power_To_Flash(batlevel); bmshata &= ~(1 << 1); }
}

void checkprob(void) {
    problem = (UVerror || temperror);
    if (problem || danger) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
}

void chargecheck(void) {
    charging = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_SET);
    if ((charging != chold || startup) && !problem) {
        startup = false;
        if (charging) {
            if (!OVerror) {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
                HAL_Delay(500);
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
                chold = true;
            }
        } else {
            HAL_Delay(500);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
            chold = false;
        }
    }
    if (OVerror) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
}

void CANparse(void) { CANakim = (uint16_t)(akimT * 5.0f); CANtemp = (uint16_t)(tempT * 5.0f); CANvolt = (uint16_t)(batvolt * 100.0f); CANminCV = (uint16_t)(minCV * 100.0f); CANmaxCV = (uint16_t)(maxCV * 100.0f); }

void CANvoltsend(void) {
    uint8_t buf[8]; uint32_t id = 0; int base = 0;
    switch (msx) { case 0: id = 28; base = 1; break; case 1: id = 27; base = 5; break; case 2: id = 26; base = 9; break; case 3: id = 25; base = 13; break; }

    buf[0] = (uint8_t)(((int)(cellV[base] * 100.0f)) >> 8); buf[1] = (uint8_t)((int)(cellV[base] * 100.0f) & 0xFF);
    buf[2] = (uint8_t)(((int)(cellV[base+1] * 100.0f)) >> 8); buf[3] = (uint8_t)((int)(cellV[base+1] * 100.0f) & 0xFF);
    buf[4] = (uint8_t)(((int)(cellV[base+2] * 100.0f)) >> 8); buf[5] = (uint8_t)((int)(cellV[base+2] * 100.0f) & 0xFF);
    buf[6] = (uint8_t)(((int)(cellV[base+3] * 100.0f)) >> 8); buf[7] = (uint8_t)((int)(cellV[base+3] * 100.0f) & 0xFF);

    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.Identifier = id;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, buf);
    msx = (msx + 1) % 4;
}

void CANdatasend(void) {
    if (charging) return;
    uint8_t buf[8];

    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    if (mss == 0) {
        buf[0] = (uint8_t)(CANmaxCV >> 8); buf[1] = (uint8_t)(CANmaxCV & 0xFF); buf[2] = (uint8_t)(CANtemp >> 8); buf[3] = (uint8_t)(CANtemp & 0xFF); buf[4] = percent; buf[5] = bmshata; buf[6] = 0x52; buf[7] = 0xAB;
        TxHeader.Identifier = 21;
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, buf);
        mss = 1;
    }
    else if (mss == 1) {
        buf[0] = (uint8_t)(CANakim >> 8); buf[1] = (uint8_t)(CANakim & 0xFF); buf[2] = (uint8_t)(CANvolt >> 8); buf[3] = (uint8_t)(CANvolt & 0xFF); buf[4] = (uint8_t)(CANminCV >> 8); buf[5] = (uint8_t)(CANminCV & 0xFF); buf[6] = 0x65; buf[7] = 0x22;
        TxHeader.Identifier = 20;
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, buf);
        mss = 2;
    }
    else { mss = 0; }
}

void CANdatatake(void) {
    if (bal > 5) AKSdata[0] = 0xFF;

    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0) {
        HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData);
        if ((RxHeader.Identifier == 50) && (RxHeader.DataLength == FDCAN_DLC_BYTES_2)) {
            AKSdata[0] = RxData[0];
            AKSdata[1] = RxData[1];
            if (AKSdata[0] == 0xF0) {
                bal = 0;
            }
        }
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
