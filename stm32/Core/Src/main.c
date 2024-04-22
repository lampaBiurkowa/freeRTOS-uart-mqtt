/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticQueue_t osStaticMessageQDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUFFER_SIZE (256)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* Definitions for sendData */
osThreadId_t sendDataHandle;
const osThreadAttr_t sendData_attributes = {
  .name = "sendData",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for receiveData */
osThreadId_t receiveDataHandle;
const osThreadAttr_t receiveData_attributes = {
  .name = "receiveData",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for persistGreenDat */
osThreadId_t persistGreenDatHandle;
const osThreadAttr_t persistGreenDat_attributes = {
  .name = "persistGreenDat",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for persistYellowDa */
osThreadId_t persistYellowDaHandle;
const osThreadAttr_t persistYellowDa_attributes = {
  .name = "persistYellowDa",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for printData */
osThreadId_t printDataHandle;
const osThreadAttr_t printData_attributes = {
  .name = "printData",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for yellowDataQueue */
osMessageQueueId_t yellowDataQueueHandle;
uint8_t yellowDataQueueBuffer[ 16 * sizeof( uint16_t ) ];
osStaticMessageQDef_t yellowDataQueueControlBlock;
const osMessageQueueAttr_t yellowDataQueue_attributes = {
  .name = "yellowDataQueue",
  .cb_mem = &yellowDataQueueControlBlock,
  .cb_size = sizeof(yellowDataQueueControlBlock),
  .mq_mem = &yellowDataQueueBuffer,
  .mq_size = sizeof(yellowDataQueueBuffer)
};
/* Definitions for greenDataQueue */
osMessageQueueId_t greenDataQueueHandle;
uint8_t greenDataQueueBuffer[ 16 * sizeof( uint16_t ) ];
osStaticMessageQDef_t greenDataQueueControlBlock;
const osMessageQueueAttr_t greenDataQueue_attributes = {
  .name = "greenDataQueue",
  .cb_mem = &greenDataQueueControlBlock,
  .cb_size = sizeof(greenDataQueueControlBlock),
  .mq_mem = &greenDataQueueBuffer,
  .mq_size = sizeof(greenDataQueueBuffer)
};
/* USER CODE BEGIN PV */
volatile uint32_t adc_values[ADC_BUFFER_SIZE];
const char YELLOW_ID = 'y';
const char GREEN_ID = 'g';
static const uint8_t YELLOW_EEPROM_ADDR = 0x50 << 1;
static const uint8_t GREEN_EEPROM_ADDR = 0x51 << 1;
uint16_t currentYellowIndex, currentGreenIndex = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
void SendDataTask(void *argument);
void ReceiveDataTask(void *argument);
void PersistGreenDataTask(void *argument);
void PersistYellowDataTask(void *argument);
void PrintDataTask(void *argument);

/* USER CODE BEGIN PFP */
void persistData(osMessageQueueId_t queue, uint8_t address, uint16_t *currentIndex);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void persistData(osMessageQueueId_t queue, uint8_t address, uint16_t *currentIndex)
{
	uint8_t value;
	while (osMessageQueueGetCount(queue) > 0)
	{
	  osMessageQueueGet(queue, &value, NULL, 0);
	  HAL_I2C_Mem_Write(&hi2c1, address, *currentIndex, I2C_MEMADD_SIZE_16BIT, &value, 1, 500);
	  osDelay(20);
	  (*currentIndex)++;
	}
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of yellowDataQueue */
  yellowDataQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &yellowDataQueue_attributes);

  /* creation of greenDataQueue */
  greenDataQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &greenDataQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of sendData */
  sendDataHandle = osThreadNew(SendDataTask, NULL, &sendData_attributes);

  /* creation of receiveData */
  receiveDataHandle = osThreadNew(ReceiveDataTask, NULL, &receiveData_attributes);

  /* creation of persistGreenDat */
  persistGreenDatHandle = osThreadNew(PersistGreenDataTask, NULL, &persistGreenDat_attributes);

  /* creation of persistYellowDa */
  persistYellowDaHandle = osThreadNew(PersistYellowDataTask, NULL, &persistYellowDa_attributes);

  /* creation of printData */
  printDataHandle = osThreadNew(PrintDataTask, NULL, &printData_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_values, ADC_BUFFER_SIZE);
  /* USER CODE END RTOS_EVENTS */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/* USER CODE BEGIN Header_SendDataTask */
/**
  * @brief  Function implementing the sendData thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_SendDataTask */
void SendDataTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  uint16_t valueToSend;
  for(;;)
  {
	valueToSend = adc_values[0];
	HAL_UART_Transmit(&huart1, (uint8_t*)&valueToSend, 2, 10);
    osDelay(20);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_ReceiveDataTask */
/**
* @brief Function implementing the receiveData thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ReceiveDataTask */
void ReceiveDataTask(void *argument)
{
  /* USER CODE BEGIN ReceiveDataTask */
  /* Infinite loop */
  for(;;)
  {
	uint8_t buffer[2] = {};
	HAL_UART_Receive(&huart1, buffer, 2, 10);
	if (buffer[0] != YELLOW_ID && buffer[0] != GREEN_ID)
		continue;

	osMessageQueueId_t queue = buffer[0] == YELLOW_ID ? yellowDataQueueHandle : greenDataQueueHandle;
	if (osMessageQueueGetSpace(queue) > 0)
		osMessageQueuePut(queue, &buffer[1], 0, 0);

    osDelay(20);
  }
  /* USER CODE END ReceiveDataTask */
}

/* USER CODE BEGIN Header_PersistGreenDataTask */
/**
* @brief Function implementing the persistGreenDat thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PersistGreenDataTask */
void PersistGreenDataTask(void *argument)
{
  /* USER CODE BEGIN PersistGreenDataTask */
  /* Infinite loop */
  for(;;)
  {
	persistData(greenDataQueueHandle, GREEN_EEPROM_ADDR, &currentGreenIndex);
    osDelay(1000);
  }
  /* USER CODE END PersistGreenDataTask */
}
/* USER CODE BEGIN Header_PersistYellowDataTask */
/**
* @brief Function implementing the persistYellowDa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PersistYellowDataTask */
void PersistYellowDataTask(void *argument)
{
  /* USER CODE BEGIN PersistYellowDataTask */
  /* Infinite loop */
  for(;;)
  {
	persistData(yellowDataQueueHandle, YELLOW_EEPROM_ADDR, &currentYellowIndex);
	osDelay(1000);
  }
  /* USER CODE END PersistYellowDataTask */
}

/* USER CODE BEGIN Header_PrintDataTask */
/**
* @brief Function implementing the printData thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PrintDataTask */
void PrintDataTask(void *argument)
{
  /* USER CODE BEGIN PrintDataTask */
  /* Infinite loop */
  char messageBuffer[128] = {};
  uint8_t output;
  osDelay(5000);
  for(;;)
  {
	HAL_I2C_Mem_Read(&hi2c1, YELLOW_EEPROM_ADDR, currentYellowIndex, I2C_MEMADD_SIZE_16BIT, &output, 1, 500);
	sprintf(messageBuffer, "Last persisted yellow: %u \r\n", output);
	HAL_UART_Transmit(&huart2, (uint8_t*)&messageBuffer, 128, 10);

	HAL_I2C_Mem_Read(&hi2c1, GREEN_EEPROM_ADDR, currentGreenIndex, I2C_MEMADD_SIZE_16BIT, &output, 1, 500);
	sprintf(messageBuffer, "Last persisted green: %u \r\n", output);
	HAL_UART_Transmit(&huart2, (uint8_t*)&messageBuffer, 128, 10);
    osDelay(5000);
  }
  /* USER CODE END PrintDataTask */
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
  if (htim->Instance == TIM1) {
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

#ifdef  USE_FULL_ASSERT
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
