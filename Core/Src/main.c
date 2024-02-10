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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "eeprom.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define debug 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile int32_t currentCount=0, prevCount=0, tempCount=0, count_diff=0;
volatile uint32_t encoder_cnt_total=0, encoder_cnt_section=0;
uint16_t encoder_cnt_unit_100mm=0;
uint8_t mode=0;

uint8_t aTxBuffer[2] ={0x0,};

uint16_t VirtAddVarTab[3] = {0x5555, 0x6666, 0x7777};
uint16_t VarDataTab[3] = {0, 0, 0};
//uint16_t VarValue,VarDataTmp = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void count_encoder_value(void);
static void count_encoder_value_diff(void);
static void clear_encoder_counter(void);
static uint8_t save_100mm_encoder_counter(uint16_t value);
static uint16_t read_100mm_encoder_counter(void);
static void sent_data_to_dmi(uint16_t data);
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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  printf("Encoder Started!!\r\n");

//save_100mm_encoder_counter(34567);
//read_100mm_encoder_counter();
/*printf("EEPROM Emulation W/R Test : %d\r\n",read_100mm_encoder_counter());*/

  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
//  HAL_TIM_Base_Start_IT(&htim1);

  /* USER CODE END 2 */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 360;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRG_OUT_GPIO_Port, TRG_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ENC_MODE2_Pin ENC_MODE1_Pin */
  GPIO_InitStruct.Pin = ENC_MODE2_Pin|ENC_MODE1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC_MODE_INT_Pin */
  GPIO_InitStruct.Pin = ENC_MODE_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENC_MODE_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC_MODE0_Pin */
  GPIO_InitStruct.Pin = ENC_MODE0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENC_MODE0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TRG_OUT_Pin */
  GPIO_InitStruct.Pin = TRG_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRG_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC_INDEX_INT_Pin */
  GPIO_InitStruct.Pin = ENC_INDEX_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENC_INDEX_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == ENC_INDEX_INT_Pin)
	{
		printf("Encoder Z\r\n");
		count_encoder_value_diff();
	}
	else if(GPIO_Pin == ENC_MODE_INT_Pin)
	{

		mode  = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
		mode |= HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1)<<1;
		mode |= HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0)<<2;

		printf("check Encoder working mode: %d\r\n", mode);

		switch(mode)
		{
			case START_ENCODER_MEASURE_DISTANCE:		//0x0

				printf("dmi-mode : START_ENCODER_MEASURE_DISTANCE\r\n");

			break;

			case STOP_ENCODER_MEASURE_DISTANCE:			//0x1

				printf("dmi-mode : STOP_ENCODER_MEASURE_DISTANCE\r\n");

			break;

			case START_ENCODER_CALIBRATION:				//0x2

				printf("dmi-mode : START_ENCODER_CALIBRATION\r\n");

			break;

			case STOP_ENCODER_CALIBRATION:			 	//0x3

				printf("dmi-mode : STOP_ENCODER_CALIBRATION\r\n");

			break;

			case START_DISTANCE_CALIBRATION:			//0x4

				printf("dmi-mode : START_DISTANCE_CALIBRATION\r\n");

				/*Encoder Z index interrupt*/
 				HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
 				HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

			break;

			case STOP_DISTANCE_CALIBRATION:			  	//0x5

				printf("dmi-mode : STOP_DISTANCE_CALIBRATION\r\n");

				/*Encoder Z index interrupt*/
				HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
				HAL_TIM_Encoder_Stop(&htim1, TIM_CHANNEL_ALL);

				count_encoder_value_diff();

			break;

			case SAVE_DISTANCE_CALIBRATION:				//0x6

			break;

			case ENCODER_IDLE:							//0x7

				printf("dmi-mode : ENCODER_IDLE\r\n");

			break;

			default:
				printf("dmi-mode : ENCODER_IDLE\r\n");
			break;
		}

	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
   if(htim == &htim1)
   {
    /*Encoder Overflow Handler here*/
	  printf("TIM1 Counter update\r\n");
   }
}

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

void count_encoder_value(void)
{
	currentCount =  __HAL_TIM_GET_COUNTER(&htim1);
	printf("currentCount : %ld\r\n", currentCount);
}

void count_encoder_value_diff(void)
{
	currentCount =  __HAL_TIM_GET_COUNTER(&htim1);
	printf("currentCount : %ld\r\n", currentCount);

	if(currentCount - prevCount<0)
	{
		count_diff = currentCount - prevCount + 65536;
	}
	else
	{
		count_diff = currentCount - prevCount;
	}

	encoder_cnt_total+=count_diff;

 	printf("count_diff : %ld\r\n", count_diff);
 	printf("encoder_cnt_total : %ld\r\n", encoder_cnt_total);

	sent_data_to_dmi(count_diff);

	tempCount = currentCount;
	prevCount = tempCount;
	currentCount = 0;
	count_diff = 0;
}

void clear_encoder_counter(void)
{
	currentCount=0, prevCount=0, tempCount=0, count_diff=0;
	encoder_cnt_total=0, encoder_cnt_section=0;
}

uint8_t save_100mm_encoder_counter(uint16_t VarValue)
{
    HAL_FLASH_Unlock();

	if( EE_Init() != EE_OK)
	{
		Error_Handler();
	}

	if((EE_WriteVariable(VirtAddVarTab[0],  VarValue)) != HAL_OK)
	{
		Error_Handler();
	}

	if((EE_ReadVariable(VirtAddVarTab[0],  &VarDataTab[0])) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_FLASH_Lock();

	if (VarValue != VarDataTab[0])
	{
		Error_Handler();
	}

	return 1;
}

uint16_t read_100mm_encoder_counter(void)
{
	if( EE_Init() != EE_OK)
	{
		Error_Handler();
	}

	if((EE_ReadVariable(VirtAddVarTab[0],  &VarDataTab[0])) != HAL_OK)
	{
		Error_Handler();
	}

	return VarDataTab[0];
}


void sent_data_to_dmi(uint16_t data)
{
	if(data>0)
	{
		aTxBuffer[0] = data>>8;
		aTxBuffer[1] = data;

//		if(HAL_UART_Transmit_IT(&huart1, (uint8_t*)aTxBuffer, sizeof(aTxBuffer))!= HAL_OK)
		if(HAL_UART_Transmit(&huart1, (uint8_t*)aTxBuffer, sizeof(aTxBuffer),0xFFFF)!= HAL_OK)
		{
			Error_Handler();
		}

	}
}

#if 0
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{

	if(UartHandle == &huart1)
	  {
		printf("uart Tx complete\r\n");
	  }
}
#endif

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
