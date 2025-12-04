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
#include "cmsis_os.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdarg.h>
#include <stdio.h>

#include "lwip/udp.h"
#include "lwip/ip_addr.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define RTC_adr 			0xD0
#define	RTC_adr_seconds		0x00
#define	RTC_adr_minutes		0x01
#define RTC_adr_TCH2		0x08
#define	RTC_adr_CFG2		0x09

#define FRAM_opcode_WREN	0x06
#define FRAM_opcode_write	0x02
#define FRAM_opcode_read	0x03
#define test_adr			0xFF

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi4;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;
osThreadId logMessageTaskHandle;
osThreadId clientTaskHandle;
osThreadId serverTaskHandle;
osThreadId heartBeatTaskHandle;
osThreadId FramTaskHandle;
osThreadId AccelerometerTaskHandle;
osThreadId PublishToBroadcastTaskHandle;
osMessageQId messageQueueHandle;
osMutexId uartMutexHandle;
osSemaphoreId AdcEndOfConversionHandle;
/* USER CODE BEGIN PV */

/* FreeRTOS private variables */

/*	Private global variables */
uint16_t ConversionTable[15];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI4_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void const * argument);
void LogMessageTask(void const * argument);
void StartClientTask(void const * argument);
void StartServerTask(void const * argument);
void StartHeartBeatTask(void const * argument);
void vFramTask(void const * argument);
void vAccelerometerTask(void const * argument);
void vPublishToBroadcastTask(void const * argument);

/* USER CODE BEGIN PFP */

uint8_t DecimalToBCD(uint8_t decimal);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_RTC_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_SPI4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of uartMutex */
  osMutexDef(uartMutex);
  uartMutexHandle = osMutexCreate(osMutex(uartMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of AdcEndOfConversion */
  osSemaphoreDef(AdcEndOfConversion);
  AdcEndOfConversionHandle = osSemaphoreCreate(osSemaphore(AdcEndOfConversion), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of messageQueue */
  osMessageQDef(messageQueue, 16, uint32_t);
  messageQueueHandle = osMessageCreate(osMessageQ(messageQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityAboveNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of logMessageTask */
  osThreadDef(logMessageTask, LogMessageTask, osPriorityAboveNormal, 0, 512);
  logMessageTaskHandle = osThreadCreate(osThread(logMessageTask), NULL);

  /* definition and creation of clientTask */
  osThreadDef(clientTask, StartClientTask, osPriorityBelowNormal, 0, 512);
  clientTaskHandle = osThreadCreate(osThread(clientTask), NULL);

  /* definition and creation of serverTask */
  osThreadDef(serverTask, StartServerTask, osPriorityBelowNormal, 0, 512);
  serverTaskHandle = osThreadCreate(osThread(serverTask), NULL);

  /* definition and creation of heartBeatTask */
  osThreadDef(heartBeatTask, StartHeartBeatTask, osPriorityIdle, 0, 512);
  heartBeatTaskHandle = osThreadCreate(osThread(heartBeatTask), NULL);

  /* definition and creation of FramTask */
  osThreadDef(FramTask, vFramTask, osPriorityLow, 0, 512);
  FramTaskHandle = osThreadCreate(osThread(FramTask), NULL);

  /* definition and creation of AccelerometerTask */
  osThreadDef(AccelerometerTask, vAccelerometerTask, osPriorityNormal, 0, 512);
  AccelerometerTaskHandle = osThreadCreate(osThread(AccelerometerTask), NULL);

  /* definition and creation of PublishToBroadcastTask */
  osThreadDef(PublishToBroadcastTask, vPublishToBroadcastTask, osPriorityNormal, 0, 4096);
  PublishToBroadcastTaskHandle = osThreadCreate(osThread(PublishToBroadcastTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */

  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c2.Init.Timing = 0x20404768;
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 7;
  hspi4.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 108-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SPI4_CS_Pin */
  GPIO_InitStruct.Pin = SPI4_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI4_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void log_message(const char *format, ...) {
	static uint8_t msg_id = 0;
	static Message_t msg;
	msg.id = msg_id++;

	va_list args;
	va_start(args, format);
	vsnprintf(msg.text, sizeof(msg.text), format, args);
	va_end(args);

	/* add your message processing and formatting here */
	osMessagePut(messageQueueHandle, (uint32_t)&msg, 0);
}

void Init_RTC(void){
	// Init the super capacitor
	uint8_t tdata_init[] = {RTC_adr_TCH2, 0x20, 0x45};
	HAL_I2C_Master_Transmit(&hi2c2, RTC_adr, tdata_init, 3, 1000);
}

void Set_RTC(uint8_t seconds, uint8_t minutes, uint8_t hours, uint8_t day, uint8_t date, uint8_t month, uint8_t years)
{
	// Convert decimal values to BCD
	uint8_t BCD_seconds = DecimalToBCD(seconds);
	uint8_t BCD_minutes = DecimalToBCD(minutes);
	uint8_t BCD_hours = DecimalToBCD(hours);
	uint8_t BCD_day = DecimalToBCD(day);
	uint8_t BCD_date = DecimalToBCD(date);
	uint8_t BCD_month = DecimalToBCD(month);
	uint8_t BCD_years = DecimalToBCD(years);

	// Sent all the BCD values to the RTC
	uint8_t tdata[] = {RTC_adr_seconds, BCD_seconds, BCD_minutes, BCD_hours, BCD_day, BCD_date, BCD_month, BCD_years};
	HAL_I2C_Master_Transmit(&hi2c2, RTC_adr, tdata, 8, 1000);
}


/* Using this function has to be like :
 * uint8_t seconds, minutes, hours, ... ;
 * Read_RTC(&seconds, &minutes, &hours, ...);
 */
void Read_RTC(uint8_t *seconds, uint8_t *minutes, uint8_t *hours, uint8_t *day, uint8_t *date, uint8_t *month, uint8_t *years)
{
	// Point to the seconds register then receive the data
	uint8_t tdata[] = {RTC_adr_seconds};
	uint8_t rdata[7] = {0};
	HAL_I2C_Master_Transmit(&hi2c2, RTC_adr, tdata, 1, 1000);
	HAL_I2C_Master_Receive(&hi2c2, RTC_adr, rdata, 2, 1000);

	// Store the converted BCD values into the corresponding output pointers
	*seconds = DecimalToBCD(rdata[0]);
	*minutes = DecimalToBCD(rdata[1]);
	*hours = DecimalToBCD(rdata[2]);
	*day = DecimalToBCD(rdata[3]);
	*date = DecimalToBCD(rdata[4]);
	*month = DecimalToBCD(rdata[5]);
	*years = DecimalToBCD(rdata[6]);
}

uint8_t DecimalToBCD(uint8_t decimal)
{
	uint8_t d_decimal = decimal / 10;
	uint8_t u_decimal = (decimal - (d_decimal * 10));
	uint8_t BCD = (d_decimal << 4) || (u_decimal & 0xF);
	return BCD;
}

uint8_t BCDToDecimal(uint8_t BCD)
{
	uint8_t d_BCD = BCD >> 4;
	uint8_t u_BCD = BCD & 0x0F;
	uint8_t Decimal = (d_BCD * 10) + u_BCD;
	return Decimal;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
    if (hadc == &hadc1){
        //BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        osSemaphoreRelease(AdcEndOfConversionHandle);
    }
}

/* USER CODE BEGIN ApplicationHooks */

/* USER CODE BEGIN ApplicationHooks */

/* USER CODE END ApplicationHooks */

/* USER CODE END ApplicationHooks */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_LogMessageTask */
/**
* @brief Function implementing the logMessageTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LogMessageTask */
void LogMessageTask(void const * argument)
{
  /* USER CODE BEGIN LogMessageTask */
  /* Infinite loop */
  Message_t msg;
  for(;;){
	  osEvent evt = osMessageGet(messageQueueHandle, 200);
	  if (evt.status == osEventMessage){
		  msg = *(Message_t *)evt.value.p;
		  osMutexWait(uartMutexHandle, osWaitForever);
		  HAL_UART_Transmit(&huart3, (uint8_t *)&msg.text, strlen(msg.text), 10);
		  osMutexRelease(uartMutexHandle); //not necessary
	  }
    osDelay(200);
  }
  /* USER CODE END LogMessageTask */
}

/* USER CODE BEGIN Header_StartClientTask */
/**
* @brief Function implementing the clientTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartClientTask */
void StartClientTask(void const * argument)
{
  /* USER CODE BEGIN StartClientTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartClientTask */
}

/* USER CODE BEGIN Header_StartServerTask */
/**
* @brief Function implementing the serverTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartServerTask */
void StartServerTask(void const * argument)
{
  /* USER CODE BEGIN StartServerTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartServerTask */
}

/* USER CODE BEGIN Header_StartHeartBeatTask */
/**
* @brief Function implementing the heartBeatTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartHeartBeatTask */
void StartHeartBeatTask(void const * argument)
{
  /* USER CODE BEGIN StartHeartBeatTask */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
	  osDelay(200);
  }
  /* USER CODE END StartHeartBeatTask */
}

/* USER CODE BEGIN Header_vFramTask */
/**
* @brief Function implementing the FramTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vFramTask */
void vFramTask(void const * argument)
{
  /* USER CODE BEGIN vFramTask */

	// Dummy message pour initialiser correctement le SPI
	uint8_t dummy = 0xFF;
	HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_SET);
	HAL_SPI_Transmit(&hspi4, &dummy, 1, 1000);
	osDelay(1);

	// Activer WREN
	uint8_t tdata_init = FRAM_opcode_WREN;
	HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi4, &tdata_init, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_SET);


	// Envoyer l'opcode d'écriture, la bonne adresse et ce qu'on veut écrire
	uint8_t tdata[] = {FRAM_opcode_write, (test_adr >> 16),  (test_adr >> 8), test_adr, 14};
	HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi4, tdata, sizeof(tdata), 1000);
	HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_SET);

  /* Infinite loop */
  for(;;)
  {
	  // Envoyer l'opcode de lecture avec l'adresse puis se mettre en lecture
	  uint8_t rdata = 0;
	  uint8_t tdata2[] = {FRAM_opcode_read, (test_adr >> 16),  (test_adr >> 8), test_adr};
	  HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi4, tdata2, sizeof(tdata2), 1000);
	  HAL_SPI_Receive(&hspi4, &rdata, sizeof(rdata), 1000);
	  HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_SET);

	  //log_message("FRAM values : %d", rdata);
	  osDelay(500);
  }
  /* USER CODE END vFramTask */
}

/* USER CODE BEGIN Header_vAccelerometerTask */
/**
* @brief Function implementing the AccelerometerTa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vAccelerometerTask */
void vAccelerometerTask(void const * argument)
{
  /* USER CODE BEGIN vAccelerometerTask */

	HAL_TIM_Base_Start(&htim2);
	HAL_ADC_Start_DMA(&hadc1, ConversionTable, 15);

  /* Infinite loop */
  for(;;)
  {
	  osSemaphoreWait(AdcEndOfConversionHandle, osWaitForever);

	  uint32_t Sum_X = 0;
	  uint32_t Sum_Y = 0;
	  uint32_t Sum_Z = 0;

	  for (int i=0;i<15;i+=3){
		  Sum_X += ConversionTable[i];
		  Sum_Y += ConversionTable[i+1];
		  Sum_Z += ConversionTable[i+2];
	  }
	  uint32_t Average_X = Sum_X / 5;
		uint32_t Average_Y = Sum_Y / 5;
		uint32_t Average_Z = Sum_Z / 5;

		// log_message("ADC values : 		X -> %d		;	Y -> %d 	;	 Z-> %d", Average_X, Average_Y, Average_Z);
  }
  /* USER CODE END vAccelerometerTask */
}

/* USER CODE BEGIN Header_vPublishToBroadcastTask */
/**
* @brief Function implementing the PublishToBroadcastTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vPublishToBroadcastTask */
void vPublishToBroadcastTask(void const * argument)
{
  /* USER CODE BEGIN vPublishToBroadcastTask */
	/* Infinite loop */
		struct udp_pcb *udp;
		struct pbuf *p;

		const char *device_id = "nucleo-14";
		const char *my_ip = "192.168.128.185";   //
		uint16_t len=0;
		uint16_t err=0;
		ip_addr_t dest_ip;

		IP4_ADDR(&dest_ip, 192,168,1,255);       //

		// Attendre autorisation de la MasterTask
		// osSemaphoreWait(SemaphoreMasterHandle, osWaitForever);

		log_message("Broadcast task started.\r\n");

		udp = udp_new();
		//udp_setflags(udp, UDP_FLAGS_BROADCAST);
		ip_set_option(udp, SOF_BROADCAST);
		printf("Flags netif: 0x%X\n", netif_default->flags);
		if (!udp) {
		   log_message("UDP alloc failed!\r\n");
		   vTaskDelete(NULL);
		}
		//err=udp_connect(udp, &dest_ip, 50000);
		udp_bind(udp, IP_ADDR_ANY, 0);     // port source aléatoire

		for(;;)
		{
		    char json_msg[256];
	        len=snprintf(json_msg, sizeof(json_msg),
	       		"{"
	      		   "\"type\":\"presence\","
	       		   "\"id\":\"%s\","
	       		   "\"ip\":\"%s\","
	       		   "\"timestamp\":\"2025-10-02T08:20:00Z\""
	       		 "}",
				 device_id,
		         my_ip//,
		         //get_timestamp() // A faire avec la RTC //"\"timestamp\":\"%s\""
		        );


		     p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
		     if (!p) continue;
		     pbuf_take(p, json_msg, len);
		     udp_sendto(udp, p, &dest_ip, 1234);

		     pbuf_free(p);


		     osDelay(10000);
		 }
  /* USER CODE END vPublishToBroadcastTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
