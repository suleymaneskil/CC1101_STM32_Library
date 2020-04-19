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
#include "cc1101.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define state 	0
  //to set transmitter   uint8_t state = 1;
  // to set receiver/modem uint8_t state = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint8_t GDO0_FLAG;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void UserLEDHide()
{
	 HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
}


void UserLEDShow()
{
	 HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  BYTE status;
  Power_up_reset();
  TI_init(&hspi1, CS_GPIO_Port, CS_Pin);


#if state
  //for transiver
  BYTE veri = 7;
  char packet[] = "RF Test";

  while (1)
  {

	  status = TI_read_status(CCxxx0_VERSION); // it is for checking only //sadece kontrol için
	  //it must be 0x14 //0x14 değeri vermelidir

	  status = TI_read_status(CCxxx0_TXBYTES); // it is too // bu da kontrol için

	  TI_strobe(CCxxx0_SFTX); // flush the buffer //bufferi temizler

	  UserLEDShow(); // turn on the led before send the data // veri göndermeden önce ledi yakar

	  TI_send_packet(packet, sizeof(packet)); //the function is sending the data

	  while(HAL_GPIO_ReadPin(GDO0_GPIO_Port, GDO0_Pin));
	  while(!HAL_GPIO_ReadPin(GDO0_GPIO_Port, GDO0_Pin));
	  //if the pass to this function, the data was sent. // veri gönderme işlemi tamamlanması için bekletir

	  status = TI_read_status(CCxxx0_TXBYTES); // it is checking to send the data //veri gönderildiğini kontrol etmek için

	  UserLEDHide(); // turn off the led // veri gönderildiğinde led söner
	  HAL_Delay(100);
  }


#else
  //for receiver/modem

  MX_USART2_UART_Init();
  init_serial(&huart2);

  char buffer[64];
  char mesaj[7];
  char kontrol[7] = {'R', 'F', ' ', 'T', 'e', 's', 't' };
  uint8_t value = 0;
  BYTE length;

  TI_write_reg(CCxxx0_IOCFG0, 0x06);
  //interrupt setting on GDO0 for data receiving with FIFO
  // GDO0 pininde FIFO ile kullanmak için interrupt ayarı

  /* EXTI interrupt init*/
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  while(1)
  {
	  BYTE status;
	  BYTE LQI;

	  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);

	  GDO0_FLAG = 0;

	  TI_strobe(CCxxx0_SFRX);// Flush the buffer //Bufferi temizle
	  TI_strobe(CCxxx0_SRX); // Set RX Mode //RX moda ayarlar

	  while(GDO0_FLAG == 0);
	  //when the data came, it passes this fucntion
	  //veri geldiğinde bu adımı geçer

	  status = TI_read_status(CCxxx0_RXBYTES);
	  //it is for checking
	  // gelen datanın olup olmadığını denetler

	  if (!(status & 0x7f)) continue;
	  // if it get the data, the code can contuniue
	  // data gelirse, kod devam eder

	  LQI = TI_read_status(CCxxx0_LQI);
	  // LQI is quaility of the receiving data
	  // LQI gelen veri kalitesi

	  if (LQI & 0x80 /*CRC_OK*/)
	  {
		  status = TI_receive_packet(buffer, &length);//read function //okuma fonksiyonu

		  for(uint8_t i = 0; i <7; i++)//to check for receiving data // gelen datayı kontrol etmek için
		  {
			  mesaj[i] = buffer[i];
			  if(strcmp(mesaj[i], kontrol[i])!=0)
			  {
				  value = 1;
				  break;
			  }
		  }

		  //to send over uart on serial port with 9600baudrate
		  // Gelen datayı uart üzerinden 9600 baudrate ile seri porta gönderir
		  HAL_UART_Transmit(&huart2, (uint8_t*)mesaj, sizeof(mesaj), HAL_MAX_DELAY);
		  HAL_UART_Transmit(&huart2, "\n\r", 2, HAL_MAX_DELAY);

		  if (value == 0)
			  // if receiving data is equal to checking data, the led is blink
			  // eğer gelen data kontrol datasına eşitse, led yanıp söner
		  {
			  UserLEDShow();

			  HAL_Delay(50);

			  UserLEDHide();
		  }
	  }
	  else
	  {
		  status = TI_read_status(CCxxx0_PKTSTATUS); // if it isnt, check pktstatus // değilse, paket durumunu kontrol eder

		  GDO0_FLAG = 0;
	  }
  }
#endif

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);


#if state
  //for transiver
	/*Configure GPIO pin : GDO0_Pin */
	GPIO_InitStruct.Pin = GDO0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GDO0_GPIO_Port, &GPIO_InitStruct);

#else
	//for receiver/modem
  /*Configure GPIO pin : GDO0_Pin */
  GPIO_InitStruct.Pin = GDO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GDO0_GPIO_Port, &GPIO_InitStruct);
#endif

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
#if state == 0
//for receiver/modem
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//if it get the receiving data, the interrupt works
	// eğer gelen data gelirse, kesme çalışır.
	if(GPIO_Pin == GDO0_Pin){
			GDO0_FLAG = 1;
		}
}
#endif
/* USER CODE END 4 */

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
