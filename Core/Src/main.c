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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;

/* USER CODE BEGIN PV */
int n;
uint16_t ButtonState = 0;
int secretNumber; //random_number 0-9
uint16_t Number = 0; //button 0-9
int b = 0; //B1(start)
int mode = 0; // mode check
int state = 0; //state Button matrix
int Button = 0; //ok,clear
int Button_state_ok = 0; //state ok
int Button_state_clr = 0; //state clear
int guessNumber = 0; //input number
int guessCount = 3; //count1-3
int LED[3] = {0,0,0}; //LED

struct _ButMtx_Struct
{
	GPIO_TypeDef* Port;
	uint16_t Pin;
};

struct _ButMtx_Struct BMX_L[4] = {
	{GPIOA,GPIO_PIN_9},
	{GPIOC,GPIO_PIN_7},
	{GPIOB,GPIO_PIN_6},
	{GPIOA,GPIO_PIN_7}
};

struct _ButMtx_Struct BMX_R[3] = {
	{GPIOB,GPIO_PIN_5},
	{GPIOB,GPIO_PIN_4},
	{GPIOB,GPIO_PIN_10},
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void ButtonMatrixRead();
void CheckNumber();
void CheckGuess();
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
  MX_LPUART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	static uint32_t BTMX_TimeStamp = 0;
	if(HAL_GetTick() > BTMX_TimeStamp)
	{
	    BTMX_TimeStamp = HAL_GetTick() + 25; //next scan in 25 ms
	  	ButtonMatrixRead();
	}
	b = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13); //start button
	if(b == 1 && mode == 0)
	{
	  	secretNumber = rand()%9; // Generate a random number between 0 and 9
	  	mode = 1;
	}
	if(mode == 1)
	{
	  	CheckNumber();
	  	CheckGuess();
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA4 LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA7 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void ButtonMatrixRead(){
	static uint8_t X=0;
	for(int i=0; i<4; i++)
	{
	if(HAL_GPIO_ReadPin(BMX_L[i].Port, BMX_L[i].Pin) == GPIO_PIN_RESET)
	{
		ButtonState |= 1 << (i + (X * 4));
	}
	else
	{
		ButtonState &= ~(1 << (i + (X * 4)));
	}
	}
	//set currentL to Hi-z (open drain)
	HAL_GPIO_WritePin(BMX_R[X].Port, BMX_R[X].Pin, GPIO_PIN_SET);
	//set nextL to low
	uint8_t nextX = (X + 1) %3;
	HAL_GPIO_WritePin(BMX_R[nextX].Port, BMX_R[nextX].Pin, GPIO_PIN_RESET);
	X = nextX;
}
void CheckNumber(){
	if(ButtonState == 8)
		{
			Number = 0;
			state = 1;
		}
		else if(ButtonState == 4)
		{
			Number = 1;
			state = 1;

		}
		else if(ButtonState == 64)
		{
			Number = 2;
			state = 1;

		}
		else if(ButtonState == 1024)
		{
			Number = 3;
			state = 1;

		}
		else if(ButtonState == 2)
	    {
			Number = 4;
			state = 1;

		}
		else if(ButtonState == 32)
		{
			Number = 5;
			state = 1;

		}
		else if(ButtonState == 512)
		{
			Number = 6;
			state = 1;

		}
		else if(ButtonState == 1)
		{
			Number = 7;
			state = 1;

		}
		else if(ButtonState == 16)
		{
			Number = 8;
			state = 1;

		}
		else if(ButtonState == 256)
		{
			Number = 9;
			state = 1;

		}
	if(state == 1)
	{
		if(ButtonState == 0)
			{
				guessNumber = Number;
				state = 0;
			}
	}
}
void CheckGuess(){
	if(guessCount == 3){
		  n = 0;
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, LED[n]);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, LED[n+1]);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, LED[n+2]);
	}
	else if(guessCount == 2){
		n = 1;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, LED[n]);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, LED[n+1]);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, LED[n-2]);
	}
	else if(guessCount == 1){
		n = 1;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, LED[n]);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, LED[n-1]);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, LED[n-2]);
	}




	if(ButtonState == 128){Button_state_ok = 1;}
	if(Button_state_ok == 1 && ButtonState == 0){

		if(memcmp(secretNumber,guessNumber,sizeof(secretNumber)) == 0){
			secretNumber = rand()%9;
			Button_state_ok = 0;
			guessNumber = 0;
		}else if(memcmp(secretNumber,guessNumber,sizeof(secretNumber)) != 0){
			guessCount--;
			Button_state_ok = 0;
			guessNumber = 0;
		}

//		Button_state_ok = 0;
//		Button = 0; //ok
	}
	if(guessCount == 0){
		secretNumber = rand()%9;


	}


	if(ButtonState == 2048){
			Button = 1; //clear
			Button_state_clr = 0;
		}
	}
//	if(ButtonState == 128 && memcmp(secretNumber,guessNumber,sizeof(secretNumber)) == 0){
//		Button = 0; //ok
//	}
//	else if(ButtonState == 128 && memcmp(secretNumber,guessNumber,sizeof(secretNumber)) != 0){
//		Button = 1; //clear
//		guessCount--;
//	}

//	if(S1 == 1 && memcmp(secretNumber,guessNumber,sizeof(secretNumber)) == 0) //S1_OK
//		{
//			guessCount--;
//			//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,GPIO_PIN_SET); //ไฟติด
//		}

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
