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
#include <string.h>
#include <ctype.h>
#include <stdint.h>

// Inlcudes for LWIP
#include "lwip/udp.h"
#include "lwip/ip_addr.h"
#include "lwip/api.h"
#include <time.h>

// Own .h file
#include "processing.h"
#include "communication.h"
#include "os_resources.h"	// Use for mutex to not let every file who use main.h to have access to the rtos lib



// Additional includes
#include <math.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// For TCP & UDP
#define PORT	1234
#define Max_IP	30


// For NTP
// #define NTP_SERVER_IP 		 "129.6.15.28"
#define NTP_SERVER_IP        "216.239.35.0"
#define NTP_PORT             123
#define NTP_PACKET_LEN       48
#define NTP_TIMESTAMP_DELTA  2208988800UL

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
osThreadId TCPClientTaskHandle;
osThreadId TCPServerTaskHandle;
osThreadId heartBeatTaskHandle;
osThreadId AccelerometerTaskHandle;
osThreadId PublishtoBroadcastTaskHandle;
osThreadId UDPServerTaskHandle;
osThreadId MainTaskHandle;
osMessageQId messageQueueHandle;
osMutexId uartMutexHandle;
osMutexId SPI4MutexHandle;
osMutexId I2C2MutexHandle;
osSemaphoreId AdcEndOfConversionHandle;
/* USER CODE BEGIN PV */

/* FreeRTOS private variables */

/*	Private global variables */

// ADC variables

// For UDP
const char *device_id = "nucleo-14";
const char *my_ip = "192.168.1.185";

// For NTP
struct udp_pcb *ntp_pcb;
uint8_t ntp_packet[NTP_PACKET_LEN];
volatile uint8_t is_synced = 0;

// All banks
ip_addr_t IP_Bank[Max_IP] = {0};		// Max_IP set in defines

// To read RTC for many sender
uint8_t seconds, minutes, hours, day, date, month, years;

// To store de RMS result
LocalValue_t LastProcessedValue;		// Dernière valeur issue de l'ADC
LocalValue_t MaxLocalValues[10]={0};	// Tableau des plus grandes valeurs

// To store the last value set as a shake
LocalValue_t LastShakeValue;

// For processing
uint8_t NewValToProcess = 0;
uint8_t FirstTime = 1;


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
void TCP_ClientTask(void const * argument);
void TCP_ServerTask(void const * argument);
void StartHeartBeatTask(void const * argument);
void vAccelerometerTask(void const * argument);
void PublishToBroadcastTask(void const * argument);
void UDP_ServerTask(void const * argument);
void vMainTask(void const * argument);

/* USER CODE BEGIN PFP */

// For NTP
void ntp_send_request(void);
void ntp_receive(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);

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

  /* definition and creation of SPI4Mutex */
  osMutexDef(SPI4Mutex);
  SPI4MutexHandle = osMutexCreate(osMutex(SPI4Mutex));

  /* definition and creation of I2C2Mutex */
  osMutexDef(I2C2Mutex);
  I2C2MutexHandle = osMutexCreate(osMutex(I2C2Mutex));

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
  osMessageQDef(messageQueue, 512, uint32_t);
  messageQueueHandle = osMessageCreate(osMessageQ(messageQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 1024);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of logMessageTask */
  osThreadDef(logMessageTask, LogMessageTask, osPriorityNormal, 0, 2048);
  logMessageTaskHandle = osThreadCreate(osThread(logMessageTask), NULL);

  /* definition and creation of TCPClientTask */
  osThreadDef(TCPClientTask, TCP_ClientTask, osPriorityNormal, 0, 2048);
  TCPClientTaskHandle = osThreadCreate(osThread(TCPClientTask), NULL);

  /* definition and creation of TCPServerTask */
  osThreadDef(TCPServerTask, TCP_ServerTask, osPriorityNormal, 0, 1024);
  TCPServerTaskHandle = osThreadCreate(osThread(TCPServerTask), NULL);

  /* definition and creation of heartBeatTask */
  osThreadDef(heartBeatTask, StartHeartBeatTask, osPriorityBelowNormal, 0, 1024);
  heartBeatTaskHandle = osThreadCreate(osThread(heartBeatTask), NULL);

  /* definition and creation of AccelerometerTask */
  osThreadDef(AccelerometerTask, vAccelerometerTask, osPriorityNormal, 0, 2048);
  AccelerometerTaskHandle = osThreadCreate(osThread(AccelerometerTask), NULL);

  /* definition and creation of PublishtoBroadcastTask */
  osThreadDef(PublishtoBroadcastTask, PublishToBroadcastTask, osPriorityAboveNormal, 0, 4096);
  PublishtoBroadcastTaskHandle = osThreadCreate(osThread(PublishtoBroadcastTask), NULL);

  /* definition and creation of UDPServerTask */
  osThreadDef(UDPServerTask, UDP_ServerTask, osPriorityNormal, 0, 2048);
  UDPServerTaskHandle = osThreadCreate(osThread(UDPServerTask), NULL);

  /* definition and creation of MainTask */
  osThreadDef(MainTask, vMainTask, osPriorityNormal, 0, 1024);
  MainTaskHandle = osThreadCreate(osThread(MainTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */

  osThreadSuspend(logMessageTaskHandle);
  osThreadSuspend(TCPClientTaskHandle);
  osThreadSuspend(TCPServerTaskHandle);
  osThreadSuspend(heartBeatTaskHandle);
  osThreadSuspend(AccelerometerTaskHandle);
  osThreadSuspend(PublishtoBroadcastTaskHandle);
  osThreadSuspend(UDPServerTaskHandle);
  osThreadSuspend(MainTaskHandle);


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
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
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


void ntp_send_request(void)
{
    ip_addr_t ntp_addr;

    ntp_pcb = udp_new();
    if (ntp_pcb == NULL) {
        log_message("NTP: udp_new failed\r\n");
        return;
    }

    udp_bind(ntp_pcb, IP_ADDR_ANY, 0);

    udp_recv(ntp_pcb, ntp_receive, NULL);

    memset(ntp_packet, 0, NTP_PACKET_LEN);
    ntp_packet[0] = 0x1B;

    ipaddr_aton(NTP_SERVER_IP, &ntp_addr);

    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, NTP_PACKET_LEN, PBUF_RAM);
    if (p == NULL) {
        log_message("NTP: pbuf_alloc failed\r\n");
        udp_remove(ntp_pcb);
        return;
    }

    memcpy(p->payload, ntp_packet, NTP_PACKET_LEN);

    udp_sendto(ntp_pcb, p, &ntp_addr, NTP_PORT);
    pbuf_free(p);

    log_message("NTP request sent to %s\r\n", ipaddr_ntoa(&ntp_addr));
}



void ntp_receive(void *arg, struct udp_pcb *pcb,
                 struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    log_message("NTP response received from %s:%d\r\n",
                ipaddr_ntoa(addr), port);

    if (p->len >= 48) {
        uint32_t timestamp;
        memcpy(&timestamp, (uint8_t *)p->payload + 40, 4);
        timestamp = ntohl(timestamp) - NTP_TIMESTAMP_DELTA;

        time_t rawtime = timestamp;
        struct tm *timeinfo = gmtime(&rawtime);

        Set_RTC(
            timeinfo->tm_sec,
            timeinfo->tm_min,
            timeinfo->tm_hour+1,
            timeinfo->tm_wday,
            timeinfo->tm_mday,
            timeinfo->tm_mon + 1,
            timeinfo->tm_year - 100
        );

        is_synced = 1;
        log_message("RTC synced with NTP\r\n");
    }

    pbuf_free(p);
    udp_remove(pcb);   // OK pour one-shot
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

  Init_FRAM();
  // If we want to clear the FRAM
  FRAM_ClearRange((uint32_t)256000, (uint32_t)(30*50));
  FRAM_ClearRange((uint32_t)130000, (uint32_t)(25*10));

  /* Infinite loop */
  for(;;)
  {
	  uint8_t Vbtn = HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin);
	  if (Vbtn){
		  osThreadResume(MainTaskHandle);
		  osThreadResume(logMessageTaskHandle);
		  osThreadResume(TCPClientTaskHandle);
		  osThreadResume(TCPServerTaskHandle);
		  osThreadResume(heartBeatTaskHandle);
		  osThreadResume(AccelerometerTaskHandle);
		  osThreadResume(PublishtoBroadcastTaskHandle);
		  osThreadResume(UDPServerTaskHandle);
		  Vbtn = 0;
	  }
	  osDelay(3);
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

/* USER CODE BEGIN Header_TCP_ClientTask */
/**
* @brief Function implementing the TCPClientTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TCP_ClientTask */
void TCP_ClientTask(void const * argument)
{
  /* USER CODE BEGIN TCP_ClientTask */
	struct netconn *conn;
	err_t err;

	for(;;)
	{
		// Send to all the IP from the IP bank
		for (uint8_t i=0;i<Max_IP;i++){
			if (ip_addr_isany(&IP_Bank[i])){break;} // If the IP position 'i' is 000.000.000.000 (not set yet), ip_addr_isany return 1

			conn = netconn_new(NETCONN_TCP);
			if (conn != NULL) {
				err = netconn_connect(conn, &IP_Bank[i], PORT);

				if (err != ERR_OK) {log_message("[CLIENT] Send error: %d\r\n", err);}


				if (err == ERR_OK) {

					Read_RTC(&seconds, &minutes, &hours, &day, &date, &month, &years);

					const char *status;
					if (LastShakeValue.status == 0){status = "normal";}
					else{status = "secousse";}

					char json[512];
					snprintf(json, sizeof(json),
					         "{"
					           "\"type\": \"data\","
					           "\"id\": \"%s\","
					           "\"timestamp\": \"20%02d-%02d-%02dT%02d:%02d:%02dZ\","
					           "\"acceleration\": {"
					               "\"x\": %f,"
					               "\"y\": %f,"
					               "\"z\": %f"
					           "},"
					           "\"status\": \"%s\""
					         "}",
					         device_id,
					         years,
					         month,
					         date,
					         hours,
					         minutes,
					         seconds,
							 LastShakeValue.Value_x,
							 LastShakeValue.Value_y,
							 LastShakeValue.Value_z,
							 status
					);

					log_message("[CLIENT] Sending : %s...\r\n", json);
					netconn_write(conn, json, strlen(json), NETCONN_COPY);
					osDelay(3000);
				}
				else {log_message("[CLIENT] Could not reach server.\r\n");}
				netconn_close(conn);
				netconn_delete(conn);
			}
			else {log_message("[CLIENT] No connection available.\r\n");}
			osDelay(20);
		}
		osDelay(5000);

	}
  /* USER CODE END TCP_ClientTask */
}

/* USER CODE BEGIN Header_TCP_ServerTask */
/**
* @brief Function implementing the TCPServerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TCP_ServerTask */
void TCP_ServerTask(void const * argument)
{
  /* USER CODE BEGIN TCP_ServerTask */
	struct netconn *conn, *newconn;
	struct netbuf *buf;
	char *data;
	u16_t len;
	conn = netconn_new(NETCONN_TCP);
	netconn_bind(conn, IP_ADDR_ANY, PORT);
	netconn_listen(conn);
	netconn_set_recvtimeout(conn, 2000);
	/* Infinite loop inside task */
	for(;;) {

		if (netconn_accept(conn, &newconn) == ERR_OK) {

			if (netconn_recv(newconn, &buf) == ERR_OK) {
				netbuf_data(buf, (void**)&data, &len);

				    char rx_buffer[512];
				    if (len >= sizeof(rx_buffer)) len = sizeof(rx_buffer) - 1;
				    memcpy(rx_buffer, data, len);
				    rx_buffer[len] = '\0';

				    OtherDevice_t DeviceThatSent = {0};

				    // Extract all the data from the received buffer
				    extract_nucleoid(rx_buffer, &DeviceThatSent);
				    extract_timestamp(rx_buffer, &DeviceThatSent);
				    extract_acceleration(rx_buffer, &DeviceThatSent);
				    extract_status(rx_buffer, &DeviceThatSent);

				    // Store the structure (with an ID manager)
				    StoreExternDeviceInFRAM(&DeviceThatSent);

					netbuf_delete(buf);
				}
			else {
				log_message("[SERVER] No reception.\r\n");
			}
			netconn_close(newconn);
			netconn_delete(newconn);
		}
		else {/* no client connection at the moment*/ }
		osDelay(2000);
	}
  /* USER CODE END TCP_ServerTask */
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
	  osDelay(500);
  }
  /* USER CODE END StartHeartBeatTask */
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

	uint16_t ConversionTable[300];

	HAL_TIM_Base_Start(&htim2);
	HAL_ADC_Start_DMA(&hadc1, ConversionTable, 300);

  /* Infinite loop */
  for(;;)
  {
	  // Wait till the semaphore is set by the interruption of the end of the conversion
	  osSemaphoreWait(AdcEndOfConversionHandle, osWaitForever);

		  /*	Reorganize the values from the DMA buffer in appropriate buffers		*/
	  uint16_t X[100], Y[100], Z[100];

	  for (int i=0;i<100;i++){
	      X[i] = ConversionTable[3*i];
	      Y[i] = ConversionTable[3*i+1];
	      Z[i] = ConversionTable[3*i+2];
	  }

	  /*	Moving average calculations		*/
	  static float MovingAverage_X[100], MovingAverage_Y[100], MovingAverage_Z[100];

	  // First we calculate the edges of the table (0 & 99)
	  MovingAverage_X[0] = (X[0] + X[1]) / 2;
	  MovingAverage_X[99] = (X[98] + X[99]) / 2;
	  MovingAverage_Y[0] = (Y[0] + Y[1]) / 2;
	  MovingAverage_Y[99] = (Y[98] + Y[99]) / 2;
	  MovingAverage_Z[0] = (Z[0] + Z[1]) / 2;
	  MovingAverage_Z[99] = (Z[98] + Z[99]) / 2;

	  for (int i=1;i<99;i++){
		  MovingAverage_X[i] = (X[i-1] + X[i] + X[i+1]) / 3.0f;
		  MovingAverage_Y[i] = (Y[i-1] + Y[i] + Y[i+1]) / 3.0f;
		  MovingAverage_Z[i] = (Z[i-1] + Z[i] + Z[i+1]) / 3.0f;
	  }

	  /*	Calculations of the RMS values		*/
	  float SumSquareX = 0, SumSquareY = 0, SumSquareZ = 0;

	  for (int i=0;i<100;i++){
		  SumSquareX += MovingAverage_X[i]*MovingAverage_X[i];
		  SumSquareY += MovingAverage_Y[i]*MovingAverage_Y[i];
		  SumSquareZ += MovingAverage_Z[i]*MovingAverage_Z[i];
	  }

	  LastProcessedValue.Value_x = sqrtf(SumSquareX / 100.0f);
	  LastProcessedValue.Value_y = sqrtf(SumSquareY / 100.0f);
	  LastProcessedValue.Value_z = sqrtf(SumSquareZ / 100.0f);

	  Read_RTC(&LastProcessedValue.sec, &LastProcessedValue.min, &LastProcessedValue.hour,
			  &day, &LastProcessedValue.mday, &LastProcessedValue.mon, &LastProcessedValue.year);

	  NewValToProcess = 1;

	  log_message("Accelerometre : %f ; %f ; %f    -    %d:%d:%d\r\n", LastProcessedValue.Value_x, LastProcessedValue.Value_y, LastProcessedValue.Value_z, LastProcessedValue.hour, LastProcessedValue.min, LastProcessedValue.sec);


  }
  /* USER CODE END vAccelerometerTask */
}

/* USER CODE BEGIN Header_PublishToBroadcastTask */
/**
* @brief Function implementing the PublishtoBroadcastTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PublishToBroadcastTask */
void PublishToBroadcastTask(void const * argument)
{
  /* USER CODE BEGIN PublishToBroadcastTask */
	struct udp_pcb *udp;
	struct pbuf *p;
	uint16_t len=0;
	ip_addr_t dest_ip;

	IP4_ADDR(&dest_ip, 192,168,1,255);

	log_message("Broadcast task started.\r\n");

	udp = udp_new();
	ip_set_option(udp, SOF_BROADCAST);
	printf("Flags netif: 0x%X\n", netif_default->flags);

	if (!udp) {
		log_message("UDP alloc failed!\r\n");
		vTaskDelete(NULL);
	}
	udp_bind(udp, IP_ADDR_ANY, 0);     // port source aléatoire

	/* Infinite loop */
	for(;;)
	{
		// Read the data from the RTC and stock to the adress of these global variables
		Read_RTC(&seconds, &minutes, &hours, &day, &date, &month, &years);

		char json_msg[256];

		len = snprintf(json_msg, sizeof(json_msg),
		    "{"
		    "\"type\": \"presence\","
		    "\"id\": \"%s\","
		    "\"ip\": \"%s\","
		    "\"timestamp\": \"20%02d-%02d-%02dT%02d:%02d:%02dZ\""
		    "}",
		    device_id,
		    my_ip,
		    years,
		    month,
		    date,
		    hours,
		    minutes,
		    seconds
		);


		p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
		if (!p) continue;
		pbuf_take(p, json_msg, len);
		udp_sendto(udp, p, &dest_ip, PORT);

		pbuf_free(p);

		osDelay(10000);
	 }
  /* USER CODE END PublishToBroadcastTask */
}

/* USER CODE BEGIN Header_UDP_ServerTask */
/**
* @brief Function implementing the UDPServerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UDP_ServerTask */
void UDP_ServerTask(void const * argument)
{
  /* USER CODE BEGIN UDP_ServerTask */
	struct netconn *conn;
	    struct netbuf *buf;
	    char *data;
	    u16_t len;
	    err_t err;

	    conn = netconn_new(NETCONN_UDP);
	    if (conn == NULL)
	    {
	        log_message("[UDP SERVER] netconn_new failed\r\n");
	        vTaskDelete(NULL);
	    }

	    netconn_bind(conn, IP_ADDR_ANY, PORT);
	    netconn_set_recvtimeout(conn, 15000);

	    // log_message("[UDP SERVER] Broadcast listener started on port 5005\r\n");

	    for (;;)
	    {
	        err = netconn_recv(conn, &buf);

	        if (err == ERR_OK)
	        {
	            netbuf_data(buf, (void**)&data, &len);
	            data[len] = '\0';
	            ip_addr_t src_ip_copy = *netbuf_fromaddr(buf);		// Get the IP adress of the sender

	            // Check if the IP adress is already known
	            uint8_t AlreadyKnown = 0;
	            for (uint8_t i = 0; i < Max_IP; i++) {
	                if (!ip_addr_isany(&IP_Bank[i]) && ip_addr_cmp(&IP_Bank[i], &src_ip_copy)) {
	                	AlreadyKnown = 1;
	                	log_message("[UDP SERVER] Broadcast received, IP already known : %s", ipaddr_ntoa(&src_ip_copy));
	                    break;
	                }
	            }
	            // If not known, store it (need to put it after all the already stored IP adress
	            if (!AlreadyKnown) {
	                for (uint8_t i = 0; i < Max_IP; i++) {
	                    if (ip_addr_isany(&IP_Bank[i])) {
	                        IP_Bank[i] = src_ip_copy;
	                        log_message("[UDP SERVER] Broadcast received, IP memorized : %s", ipaddr_ntoa(&src_ip_copy));
	                        break;
	                    }
	                }
	            }

	            netbuf_delete(buf);
	        }
	        else if (err == ERR_TIMEOUT)
	        {
	        	log_message("[UDP SERVER] Timout error");

	        }
	        else
	        {
	            log_message("[UDP SERVER] Receive error: %d\r\n", err);
	        }

	        osDelay(100);
	    }
  /* USER CODE END UDP_ServerTask */
}

/* USER CODE BEGIN Header_vMainTask */
/**
* @brief Function implementing the MainTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vMainTask */
void vMainTask(void const * argument)
{
  /* USER CODE BEGIN vMainTask */
	Init_RTC();
	ntp_send_request();
	osDelay(10);

	float PreviousVal_x = 0.0f;
	float PreviousVal_y = 0.0f;
	float PreviousVal_z = 0.0f;

	const float ACC_THRESHOLD = 4.0f;	// Seuil de détection, augmenter si trop sensible

  /* Infinite loop */
	for (;;)
	    {
	        if (NewValToProcess)	// Si une nouvelle valeur a été traitée
	        {
	        	/***********		Détection de secousse		****************/
	            float dx = LastProcessedValue.Value_x - PreviousVal_x;
	            float dy = LastProcessedValue.Value_y - PreviousVal_y;
	            float dz = LastProcessedValue.Value_z - PreviousVal_z;
	            if (fabsf(dx) > ACC_THRESHOLD || fabsf(dy) > ACC_THRESHOLD || fabsf(dz) > ACC_THRESHOLD)
	            {
	            	// Put the state of the value as a "secousse"
	            	LastProcessedValue.status = 1;
	            	log_message("Detection locale !\r\n");
	            }
	            else{LastProcessedValue.status = 0;}
	            PreviousVal_x = LastProcessedValue.Value_x;
				PreviousVal_y = LastProcessedValue.Value_y;
				PreviousVal_z = LastProcessedValue.Value_z;

				/***********		If there is a lower value in the FRAM, replace it with the new one		****************/
				if (UpdateFRAMIfHigherValue(&LastProcessedValue, &LastShakeValue)){}

				/***********		Check the other device value state in the FRAM to make an alert of shake or not		****************/

				if (CheckShakeCorrelationInFRAM(&LastShakeValue)){
					// Use the semaphore to activate the LED
					HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);
				}
				else {HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);}

	            NewValToProcess = 0;
	        }

	        // Compare the values of other devices reported as “shake” with my last value reported as shake,
	        // 	and if the timestamp matches, turn on an LED.


	        osDelay(3);
	    }
  /* USER CODE END vMainTask */
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
