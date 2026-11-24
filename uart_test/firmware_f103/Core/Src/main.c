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
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUG_PIN_LEVEL 0
#define ENCODER_TX_INTERVAL_MS 50U
#define MOTOR_TIMEOUT_MS 300U
#define COMMAND_BUFFER_SIZE 24U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
static uint8_t rxByte;
static char rxLine[COMMAND_BUFFER_SIZE];
static char pendingCommand[COMMAND_BUFFER_SIZE];
static uint8_t rxLength;
static uint8_t discardLine;
static volatile uint8_t commandReady;
static volatile uint8_t stopRequested;
static uint8_t previousCR;
static uint16_t previousLeftEncoder;
static uint16_t previousRightEncoder;
static uint32_t leftEncoder;
static uint32_t rightEncoder;
static char encoderMessage[sizeof("ENC,4294967295,4294967295\r\n")];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static int readDuty(const char **text, int *value)
{
  const char *p = *text;
  int sign = 1;
  int number = 0;

  if (*p == '-')
  {
    sign = -1;
    ++p;
  }
  if (*p < '0' || *p > '9')
  {
    return 0;
  }

  while (*p >= '0' && *p <= '9')
  {
    number = number * 10 + (*p - '0');
    if (number > 100)
    {
      return 0;
    }
    ++p;
  }

  *text = p;
  *value = sign * number;
  return 1;
}

static int parseCommand(const char *text, int *left, int *right)
{
  if (text[0] != 'M' || text[1] != ',')
  {
    return 0;
  }

  text += 2;
  if (!readDuty(&text, left) || *text != ',')
  {
    return 0;
  }
  ++text;

  return readDuty(&text, right) && *text == '\0';
}

static void setMotor(uint32_t channel, GPIO_TypeDef *port,
                     uint16_t in1, uint16_t in2, int duty, int *previousDuty)
{
  if (duty == *previousDuty)
  {
    return;
  }

  if ((duty > 0) != (*previousDuty > 0) ||
      (duty < 0) != (*previousDuty < 0))
  {
    __HAL_TIM_SET_COMPARE(&htim3, channel, 0U);
    if (duty != 0 && *previousDuty != 0)
    {
      HAL_Delay(2U);
    }
    HAL_GPIO_WritePin(port, in1 | in2, GPIO_PIN_RESET);
    if (duty != 0)
    {
      HAL_GPIO_WritePin(port, duty > 0 ? in1 : in2, GPIO_PIN_SET);
    }
  }

  uint32_t percent = (uint32_t)(duty < 0 ? -duty : duty);
  uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim3) + 1U;
  __HAL_TIM_SET_COMPARE(&htim3, channel, period * percent / 100U);
  *previousDuty = duty;
}

static void setMotors(int left, int right)
{
  static int previousLeft;
  static int previousRight;
  setMotor(TIM_CHANNEL_1, GPIOB, GPIO_PIN_0, GPIO_PIN_1, left, &previousLeft);
  setMotor(TIM_CHANNEL_2, GPIOB, GPIO_PIN_12, GPIO_PIN_13, right, &previousRight);
}

static void accumulateEncoder(TIM_HandleTypeDef *timer, uint16_t *previous, uint32_t *total)
{
  /* Samples must be less than half of a 16-bit counter cycle apart. */
  uint16_t current = (uint16_t)__HAL_TIM_GET_COUNTER(timer);
  uint16_t wrappedDelta = (uint16_t)(current - *previous);
  int32_t delta = wrappedDelta <= INT16_MAX ? wrappedDelta : (int32_t)wrappedDelta - 65536;

  *previous = current;
  *total += (uint32_t)delta;
}

static void sendEncoder(void)
{
  uint32_t left = leftEncoder;
  uint32_t right = rightEncoder;
  if (huart3.gState != HAL_UART_STATE_READY)
  {
    return;
  }
  int length = snprintf(encoderMessage, sizeof(encoderMessage), "ENC,%" PRIu32 ",%" PRIu32 "\r\n", left, right);

  if (length > 0 && (size_t)length < sizeof(encoderMessage))
  {
    HAL_UART_Transmit_IT(&huart3, (uint8_t *)encoderMessage, (uint16_t)length);
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0U);
  if (HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK ||
      HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK ||
      HAL_UART_Receive_IT(&huart3, &rxByte, 1U) != HAL_OK)
  {
    Error_Handler();
  }

  uint32_t lastCommand = HAL_GetTick();
  uint32_t lastEncoder = lastCommand - ENCODER_TX_INTERVAL_MS;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();
    accumulateEncoder(&htim2, &previousLeftEncoder, &leftEncoder);
    accumulateEncoder(&htim4, &previousRightEncoder, &rightEncoder);

    if (stopRequested)
    {
      uint32_t irqState = __get_PRIMASK();
      __disable_irq();
      commandReady = 0U;
      stopRequested = 0U;
      __set_PRIMASK(irqState);
      setMotors(0, 0);
    }

    if (commandReady)
    {
      char command[COMMAND_BUFFER_SIZE];
      uint32_t irqState = __get_PRIMASK();
      __disable_irq();
      memcpy(command, pendingCommand, sizeof(command));
      commandReady = 0;
      __set_PRIMASK(irqState);

      int left;
      int right;
      if (parseCommand(command, &left, &right))
      {
        setMotors(left, right);
        lastCommand = now;
      }
      else
      {
        setMotors(0, 0);
      }
    }

    if (now - lastCommand >= MOTOR_TIMEOUT_MS)
    {
      setMotors(0, 0);
    }

    if (now - lastEncoder >= ENCODER_TX_INTERVAL_MS)
    {
      lastEncoder = now;
      sendEncoder();
    }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_12 | GPIO_PIN_13, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_12 | GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

static void MX_TIM2_Init(void)
{
  TIM_Encoder_InitTypeDef encoder = {0};
  TIM_MasterConfigTypeDef master = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = UINT16_MAX;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  encoder.EncoderMode = TIM_ENCODERMODE_TI12;
  encoder.IC1Polarity = TIM_ICPOLARITY_RISING;
  encoder.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  encoder.IC1Prescaler = TIM_ICPSC_DIV1;
  encoder.IC1Filter = 4;
  encoder.IC2Polarity = TIM_ICPOLARITY_RISING;
  encoder.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  encoder.IC2Prescaler = TIM_ICPSC_DIV1;
  encoder.IC2Filter = 4;

  if (HAL_TIM_Encoder_Init(&htim2, &encoder) != HAL_OK)
  {
    Error_Handler();
  }

  master.MasterOutputTrigger = TIM_TRGO_RESET;
  master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &master) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 63;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK ||
      HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

static void MX_TIM4_Init(void)
{
  TIM_Encoder_InitTypeDef encoder = {0};
  TIM_MasterConfigTypeDef master = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = UINT16_MAX;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  encoder.EncoderMode = TIM_ENCODERMODE_TI12;
  encoder.IC1Polarity = TIM_ICPOLARITY_RISING;
  encoder.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  encoder.IC1Prescaler = TIM_ICPSC_DIV1;
  encoder.IC1Filter = 4;
  encoder.IC2Polarity = TIM_ICPOLARITY_RISING;
  encoder.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  encoder.IC2Prescaler = TIM_ICPSC_DIV1;
  encoder.IC2Filter = 4;

  if (HAL_TIM_Encoder_Init(&htim4, &encoder) != HAL_OK)
  {
    Error_Handler();
  }

  master.MasterOutputTrigger = TIM_TRGO_RESET;
  master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &master) != HAL_OK)
  {
    Error_Handler();
  }
}

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
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/* USER CODE BEGIN 4 */
#if DEBUG_PIN_LEVEL
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  static uint8_t pinMessage[] = "PIN,0,0\r\n";

  /* HAL advances the TX pointer to the end of the completed buffer. */
  if (huart != &huart3 ||
      huart->TxXferSize >= sizeof(encoderMessage) ||
      huart->pTxBuffPtr != (uint8_t *)encoderMessage + huart->TxXferSize)
  {
    return;
  }

  pinMessage[4] = (GPIOB->IDR & GPIO_PIN_6) ? '1' : '0';
  pinMessage[6] = (GPIOB->IDR & GPIO_PIN_7) ? '1' : '0';
  HAL_UART_Transmit_IT(huart, pinMessage, sizeof(pinMessage) - 1U);
}
#endif

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART3)
  {
    return;
  }

  if (rxByte == '\r' || rxByte == '\n')
  {
    if (!(rxByte == '\n' && previousCR))
    {
      if (discardLine || rxLength == 0U || commandReady || stopRequested)
      {
        stopRequested = 1U;
      }
      else
      {
        rxLine[rxLength] = '\0';
        memcpy(pendingCommand, rxLine, rxLength + 1U);
        commandReady = 1U;
      }
    }
    previousCR = (rxByte == '\r');
    rxLength = 0U;
    discardLine = 0U;
  }
  else
  {
    previousCR = 0U;
    if (!discardLine)
    {
      if (rxByte == '\0' || rxLength >= sizeof(rxLine) - 1U)
      {
        discardLine = 1U;
        stopRequested = 1U;
      }
      else
      {
        rxLine[rxLength++] = (char)rxByte;
      }
    }
  }

  if (HAL_UART_Receive_IT(huart, &rxByte, 1U) != HAL_OK)
  {
    stopRequested = 1U;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    stopRequested = 1U;
    rxLength = 0U;
    discardLine = 1U;
    previousCR = 0U;
    HAL_UART_AbortReceive(huart);
    if (HAL_UART_Receive_IT(huart, &rxByte, 1U) != HAL_OK)
    {
      Error_Handler();
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
  __HAL_RCC_TIM3_CLK_ENABLE();
  TIM3->CCR1 = 0U;
  TIM3->CCR2 = 0U;
  TIM3->CCER = 0U;
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
