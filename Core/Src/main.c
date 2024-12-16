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
#include <string.h> // For strlen

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RS GPIO_PIN_0 // Register Select
#define RW GPIO_PIN_1 // Read/Write
#define EN GPIO_PIN_2 // Enable
#define DATA_PORT GPIOA // Data Port
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t rxBuffer[10]; // Buffer for received UART data
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
void LCD_Init(void);
void LCD_Command(uint8_t cmd);
void LCD_Data(uint8_t data);
void LCD_Print(char *str);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay_ms(uint16_t ms)
{
    HAL_Delay(ms);
}

void LCD_Init(void)
{
    // Initialize LCD in 4-bit mode
    LCD_Command(0x02); // Initialize to 4-bit mode
    LCD_Command(0x28); // 2 lines, 5x7 matrix
    LCD_Command(0x0C); // Display on, cursor off
    LCD_Command(0x06); // Increment cursor
    LCD_Command(0x01); // Clear display
    delay_ms(2);
}

void LCD_Command(uint8_t cmd)
{
    HAL_GPIO_WritePin(DATA_PORT, RS, GPIO_PIN_RESET); // RS = 0 for command
    HAL_GPIO_WritePin(DATA_PORT, RW, GPIO_PIN_RESET); // RW = 0 for write

    GPIOA->ODR = (GPIOA->ODR & 0x0F) | (cmd & 0xF0); // Send higher nibble
    HAL_GPIO_WritePin(DATA_PORT, EN, GPIO_PIN_SET);
    delay_ms(1);
    HAL_GPIO_WritePin(DATA_PORT, EN, GPIO_PIN_RESET);

    GPIOA->ODR = (GPIOA->ODR & 0x0F) | ((cmd << 4) & 0xF0); // Send lower nibble
    HAL_GPIO_WritePin(DATA_PORT, EN, GPIO_PIN_SET);
    delay_ms(1);
    HAL_GPIO_WritePin(DATA_PORT, EN, GPIO_PIN_RESET);

    delay_ms(2);
}

void LCD_Data(uint8_t data)
{
    HAL_GPIO_WritePin(DATA_PORT, RS, GPIO_PIN_SET); // RS = 1 for data
    HAL_GPIO_WritePin(DATA_PORT, RW, GPIO_PIN_RESET); // RW = 0 for write

    GPIOA->ODR = (GPIOA->ODR & 0x0F) | (data & 0xF0); // Send higher nibble
    HAL_GPIO_WritePin(DATA_PORT, EN, GPIO_PIN_SET);
    delay_ms(1);
    HAL_GPIO_WritePin(DATA_PORT, EN, GPIO_PIN_RESET);

    GPIOA->ODR = (GPIOA->ODR & 0x0F) | ((data << 4) & 0xF0); // Send lower nibble
    HAL_GPIO_WritePin(DATA_PORT, EN, GPIO_PIN_SET);
    delay_ms(1);
    HAL_GPIO_WritePin(DATA_PORT, EN, GPIO_PIN_RESET);

    delay_ms(2);
}

void LCD_Print(char *str)
{
    while (*str)
    {
        LCD_Data(*str++);
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
  char txMessage[] = "Hello UART\r\n";
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  LCD_Init();
  LCD_Print("STM32 Ready");

  HAL_UART_Transmit(&huart2, (uint8_t *)txMessage, strlen(txMessage), HAL_MAX_DELAY);
  HAL_UART_Receive_IT(&huart2, rxBuffer, sizeof(rxBuffer));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* Display received data on LCD */
    if (rxBuffer[0] != 0)
    {
        LCD_Command(0x01); // Clear display
        LCD_Print((char *)rxBuffer);
        memset(rxBuffer, 0, sizeof(rxBuffer)); // Clear buffer
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RS|RW|EN, GPIO_PIN_RESET);

  /*Configure GPIO pins for LCD */
  GPIO_InitStruct.Pin = RS|RW|EN|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
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
}
