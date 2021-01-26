/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include <stdbool.h>
#include "math.h"

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
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char RxBUF[200]; // bufor do odbioru
char TxBUF[200]; // bufor do nadawania
uint8_t temp[1]; // zmienna na pojedynczy znak
uint8_t rx_e = 0, rx_f = 0; // wskaźniki w buforze do odbioru
uint8_t tx_e = 0, tx_f = 0; // wskaźniki w buforze do nadawania
uint8_t counter = 0; // wskaźnik dla komendy bufora odbiorczego
bool frameStarted = false; // wskaźnik odczytania początku ramki
bool frameCompleted = false; // wskaźnik kompletności ramki
int dataLength; // długośc pola danych
bool sign0xEAIsRead = false; // wskaźnik odczytania znaku specjalnego
uint8_t error;
char frame[100];
bool crcCorrect; // wskaźnik poprawności kodu kontrolnego
char crc[2]; // kod kontrolny odczytany w ramce
char hello_command[] = "Hello, I am STM32!!! \n\r";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

LCD_setRST(LCD_RST_Port, LCD_RST_Pin);
LCD_setCE(LCD_CE1_Port, LCD_CE1_Pin);
LCD_setDC(LCD_DC_Port, LCD_DC_Pin);
LCD_setDIN(SPI1_MOSI_Port, SPI1_MOSI_Pin);
LCD_setCLK(SPI1_SCK_Port, SPI1_SCK_Pin);



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) { // odbiór i zapis danych na przerwaniach
			if (huart->Instance == USART2) {
				if (rx_e == 199)
					rx_e = 0;
				else
					rx_e++;
				HAL_UART_Receive_IT(&huart2, &RxBUF[rx_e], 1);
	}
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) { // wysyłanie danych na przerwaniach
	if (huart->Instance == USART2) {
		uint8_t temp = TxBUF[tx_f]; // znak do wysłania
		if (tx_f != tx_e) {
			if (tx_f == 199)
				tx_f = 0;
			else
				tx_f++;
			HAL_UART_Transmit_IT(&huart2, &temp, 1);
		}
	}
}

void put(char ch[]) { // dodawanie komend do bufora nadawczego
	uint8_t index = tx_e; // zapamiętanie wartości wskaźnika tx_e
	for (int i = 0; i < strlen(ch); i++) { // dodawanie znaków do bufora
		TxBUF[index] = ch[i];

		if (index == 199)
			index = 0;
		else
			index++;
	}
	__disable_irq();
	if((tx_e == tx_f) && // jeżeli bufor wysyłający był pusty przed dodaniem znaków łańcucha
			(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE) == SET)) {
		tx_e = index; // przesunięcie wskaźnika na puste miejsce po dodaniu znaków łańcucha
		uint8_t tmp = TxBUF[tx_f]; // zapamiętanie znaku do wysłania
		if (tx_f == 199)
			tx_f = 0;
		else
			tx_f++;
		HAL_UART_Transmit_IT(&huart2, &tmp, 1);
	} else // jeżel w buforze są dane
		tx_e = index; // znaki łańcucha czekają w kolejce w buforze
	__enable_irq();
}
void readChar() {

	if (RxBUF[rx_f] == 0xEE) { // znak początku ramki
		counter = 0; // ustawienie wartości startowych zmiennych
		frameStarted = true;
		frameCompleted = false;
		dataLength = 0;
		sign0xEAIsRead = false;
		for(int i = 0; i < 14; i++){
			frame[i] = 0x00;
		}
		error = 0;
	}

	if (!frameStarted) {
		if (rx_f == 199)
				rx_f = 0;
		else
				rx_f++;
		return;
	}

	frame[counter] = RxBUF[rx_f]; // przepisanie znaku bufora

	if (counter == 5 + dataLength) { // sprawdzenie, czy odczytano ostatni znak ramki
		frameCompleted = true;
	}

	if (!frameCompleted) {
		if (counter == 1) {
			if (frame[1] > 0x08) { // błędna zawartośc pola LENGTH

				error = 0x09;
				my_Error_Handler();
				return;
			}
			else
				dataLength = frame[1];  // długośc pola DATA przed ew. przekodowaniem
		}
		if (counter == 2) {
			char test = frame[2] ^ frame[1];
			if (test != 0xFF)  { // błędna zawartośc pola NOT_LENGTH
				error = 0x08;
				my_Error_Handler();
				return;
			}
		}
		if (dataLength > 0) {
			if (counter > 3 && counter <= counter + dataLength) { // pole DATA
				if (sign0xEAIsRead) { // sprawdzenie czy poprzedni znak to 0xEA
					sign0xEAIsRead = false;
					if (RxBUF[rx_f] == 0xEB)
						frame[counter] = 0xEE; // dekodowanie znaku 0xEE
					else if (RxBUF[rx_f] == 0xEC)
						frame[counter] = 0xEA; // dekodowanie znaku 0xEA
					else {
						error = 0x07;
						my_Error_Handler();
						return;
					}
				}
				if (frame[counter] == 0xEA) {
					sign0xEAIsRead = true;
					counter--;  // w celu przykrycia bieżącego znaku przez następny
				}
			}
		}
		if (counter == 5 + dataLength)
			frameCompleted = true; // przeczytano wszystkie znaki ramki
	}

	if (frameCompleted) {
		checkCRC();
		if (!crcCorrect) {
			error = 0x06;
			my_Error_Handler();
			return;
		} else
			if (frame[3] != 0x11 && frame[3] != 0x22 && frame[3] != 0x33
					&& frame[3] != 0x44 && frame[3] != 0x01 && frame[3] != 0x02
					&& frame[3] != 0x03 && frame[3] != 0x04 && frame[3] != 0x66 && frame[3] != 0x55) {
				error = 0x05;  // nierozpoznane polecenie
				my_Error_Handler();
				return;
			}


		//Wybór komendy oraz obsługa błędu
		switch(frame[3]){ // komendy
					case  0x55:
						put("\nkomenda 0x55");

						break;

					case 0x22:
						put("\nkomenda 0x22");

						break;
					case 0x11:
						put("\nkomenda 0x11");

						break;
					case 0x66:
						put("\nkomenda 0x66");

						break;
					case 0x33:
						put("\nkomenda 0x33");
						break;
		}
	}
	if (rx_f == 199)
			rx_f = 0;
	else
			rx_f++;
	counter++;
}

void checkCRC() {//Obliczanie CRC
	crc[0] = crc[1] = 0x00;
	int frameLength = 4 + dataLength;
 	for(int i = 0; i < frameLength; i++) { // sprawdzanie kolejnych bajtów ramki
 		char byte = frame[i];
 		int numberOf1 = 0;
 		for(int j = 0; j < 8; j++){ // zliczanie jedynek w bitach kolejnego bajtu
 			if (byte & 0x01)
 				numberOf1++;
 			byte >>= 1;
 		}
 		if (numberOf1 == 1 || numberOf1 == 3 || numberOf1 == 5 || numberOf1 == 7)
 			crc[1] |= 0x01; // ustawienie najmłodszego bitu kodu crc

 		crc[0] <<= 1;		// przesunięcie bitowe słowa 16-bitowego
 		if (crc[1] & 0x80)
 			crc[0] |= 0x01;
 		crc[1] <<= 1;
 	}
 	for(int i = 0; i < 15 - frameLength; i++) { // przesunięcie do najstarszego bitu
 		crc[0] <<= 1;
 		if (crc[1] & 0x80)
 			crc[0] |= 0x01;
 		crc[1] <<= 1;
 	}
 	// Porównanie otrzymanego CRC z obliczonym
 	if (frame[counter - 1] == crc[0] && frame[counter] == crc[1])
 		crcCorrect = true;
 	else
 		crcCorrect = false;
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, &RxBUF[rx_e], 1);
  put("\nWpisz ramke: ");
  LCD_init();
  LCD_print("Hello World", 0, 0);
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LCD_CE1_Pin|LCD_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_CE1_Pin LCD_DC_Pin */
  GPIO_InitStruct.Pin = LCD_CE1_Pin|LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_RST_Pin */
  GPIO_InitStruct.Pin = LCD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_RST_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void my_Error_Handler()
{
	counter = 0;
	frameStarted = false;
	frameCompleted = false;
	dataLength = 0;
	sign0xEAIsRead = false;
	for(int i = 0; i < 14; i++){
		frame[i] = 0x00;
	}
	if (rx_f == 199)
			rx_f = 0;
	else
			rx_f++;

	switch (error) {
	case 0x05: put("\nNierozpoznane polecenie. ");     //EE00FF000000
			   break;
	case 0x06: put("\nBledny kod CRC. ");			   //EE00FF110001
			   break;
	case 0x07: put("\nBledny znak po znaku 0xEA. ");   //EE01FE33EA00
			   break;
	case 0x08: put("\nBledna struktura ramki. ");      //EE0022
			   break;
	case 0x09: put("\nBledna zawartosc pola LENGTH. ");//EEFF00
			   break;							// dobra:EE00FF330000, EE08F73304040404040404046FF0, EE01FE660660
	case 0x0A: put("\nPrzekroczony zakres amplitudy ");
			   break;
	default:   put("\nNierozpoznany kod bledu. ");
			   break;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
