/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TEST_PASS 0
#define TEST_FAIL 1
#define TEST_GO 1
#define TEST_WAIT 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t buf_uart_rx[128];
uint8_t uart_rx[2];
int pos_uart_rx = 0;
int isUartRXCplt = 0;

int isGoTest = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */  
PUTCHAR_PROTOTYPE  
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART */
    //HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);
    return ch;
}  

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
	if (huart->Instance != USART1) return;
	if (pos_uart_rx == 0) memset(buf_uart_rx, 0, 128);
	if (pos_uart_rx >= 128) {
		pos_uart_rx = 0;
	}
	HAL_UART_Receive_IT(huart, uart_rx, 1);
	if (uart_rx[0] == ' ') {
		buf_uart_rx[pos_uart_rx++] = uart_rx[0];
		isUartRXCplt = 1;
		pos_uart_rx = 0;
	}
	else {
		buf_uart_rx[pos_uart_rx++] = uart_rx[0];
		isUartRXCplt = 1;
	}
	//int i = 1000*1000;
	//while (i--) {__NOP();}
	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
}

/**
  * @brief  EXTI line detection callback.
  * @param  GPIO_Pin Specifies the port pin connected to corresponding EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
#if 0	
	//int i = 1000*1000;
  switch (GPIO_Pin) {
		case GPIO_PIN_0:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0));
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 125);	
			i = 1*1000*1000;	
			while (i--) {__NOP();}	
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 25);
			break;
		case GPIO_PIN_1:
			//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 125);
			i = 3*1000*1000;
			while (i--) {__NOP();}	
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 25);
			break;
		default:
			break;
	}
#endif
	isGoTest = TEST_GO;
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
}

int test8MAGNET(void) {
	GPIO_PinState bitstatus = GPIO_PIN_RESET;	
	int ret = TEST_PASS;
	
#if 0 //pressure test
	return ret;
#endif 	
	
	bitstatus = HAL_GPIO_ReadPin(GPIOA, CUBE1_Pin);
	if (bitstatus == GPIO_PIN_SET) {
		ret |= (0x1ul<<0);
		//return ret;
	}
	bitstatus = HAL_GPIO_ReadPin(GPIOA, CUBE2_Pin);
	if (bitstatus == GPIO_PIN_SET) {
		ret |= (0x1ul<<1);
		//return ret;
	}
	bitstatus = HAL_GPIO_ReadPin(GPIOA, CUBE3_Pin);
	if (bitstatus == GPIO_PIN_SET) {
		ret |= (0x1ul<<2);
		//return ret;
	}
	bitstatus = HAL_GPIO_ReadPin(GPIOA, CUBE4_Pin);
	if (bitstatus == GPIO_PIN_SET) {
		ret |= (0x1ul<<3);
		//return ret;
	}
	bitstatus = HAL_GPIO_ReadPin(GPIOA, CUBE5_Pin);
	if (bitstatus == GPIO_PIN_SET) {
		ret |= (0x1ul<<4);
		//return ret;
	}
	bitstatus = HAL_GPIO_ReadPin(GPIOA, CUBE6_Pin);
#if 0 //my sensor broken...	
	if (bitstatus == GPIO_PIN_SET) {
		ret |= (0x1ul<<5);
		//return ret;
	}
#endif	
	bitstatus = HAL_GPIO_ReadPin(GPIOA, CUBE7_Pin);
#if 1 //my sensor broken...	
	if (bitstatus == GPIO_PIN_SET) {
		ret |= (0x1ul<<6);
		//return ret;
	}
#endif
	bitstatus = HAL_GPIO_ReadPin(GPIOB, CUBE8_Pin);
	if (bitstatus == GPIO_PIN_SET) {
		ret |= (0x1ul<<7);
		//return ret;
	}
	return ret;
}


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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart1, uart_rx, 1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	int i = 1000*1000;
	int ret = TEST_FAIL;
	char msg[32];	
  while (1)
  {
		ret = TEST_FAIL;
		//if (isGoTest == TEST_GO) {
		if (1) {
			ret = test8MAGNET();
		}
		if (ret == TEST_PASS) {
			// servo open and close
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0));
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 125);	
			i = 1*1000*1000;	
			while (i--) {__NOP();}	
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 25);
			HAL_Delay(1*1000);
		}
		else {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
			HAL_Delay(1*500);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
		}
		isGoTest = TEST_WAIT;
		
#if 1 //print uart buffer
			if (ret != TEST_PASS) {	
				sprintf(msg, "%s0x%x\n", "[TEST]:", ret);
				if ((ret & (0x1ul<<0))) {
					strcat(msg, "[1]");
				}
				if ((ret & (0x1ul<<1))) {
					strcat(msg, "[2]");
				}
				if ((ret & (0x1ul<<2))) {
					strcat(msg, "[3]");
				}
				if ((ret & (0x1ul<<3))) {
					strcat(msg, "[4]");
				}
				if ((ret & (0x1ul<<4))) {
					strcat(msg, "[5]");
				}
				if ((ret & (0x1ul<<5))) {
					strcat(msg, "[6]");
				}
				if ((ret & (0x1ul<<6))) {
					strcat(msg, "[7]");
				}
				if ((ret & (0x1ul<<7))) {
					strcat(msg, "[8]");
				}
				strcat(msg, "FAIL\n");
				int len = strlen(msg);
				HAL_UART_Transmit(&huart1, (uint8_t *)msg, len, 100*len);
			}
			else {
				sprintf(msg, "%s\n", "[TEST PASS]");
				int len = strlen(msg);
				HAL_UART_Transmit(&huart1, (uint8_t *)msg, len, 100*len);
			}
#else			
		__NOP();
#endif	
		HAL_Delay(1*40);
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
	}	
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  
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

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.Prescaler = 960;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CUBE1_Pin|CUBE2_Pin|CUBE3_Pin|CUBE4_Pin
                          |CUBE5_Pin|CUBE6_Pin|CUBE7_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CUBE8_GPIO_Port, CUBE8_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CUBE1_Pin CUBE2_Pin CUBE3_Pin CUBE4_Pin
                           CUBE5_Pin CUBE6_Pin CUBE7_Pin */
  GPIO_InitStruct.Pin = CUBE1_Pin|CUBE2_Pin|CUBE3_Pin|CUBE4_Pin
                          |CUBE5_Pin|CUBE6_Pin|CUBE7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CUBE8_Pin */
  GPIO_InitStruct.Pin = CUBE8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CUBE8_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
