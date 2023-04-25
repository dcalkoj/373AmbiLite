/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "stdio.h"
//#include "math.h"
#include "stdlib.h"
//#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define NUMW 37
#define NUMH 21
#define NUMLED 37*2+21*2
#define NUMLED2 3*(37*2+21*2)

#define max(X,Y) ( (X) > (Y) ? (X) : (Y) )

typedef struct{
	uint8_t staticstart : 3;
	uint8_t globbright : 5;
	uint8_t b : 8;
	uint8_t g : 8;
	uint8_t r : 8;
} __attribute__((__packed__)) ledFrame;

typedef struct{
	uint32_t startFrame;
	ledFrame ledarray[NUMLED];
	uint32_t endFrame;
} Frame;

enum Color{Red, Green, Blue};
enum Mode{piCam, colorProfile, test, refreshBorder, still, runner, runner2, pulse, runner3};
enum Wanted{Top, Right, Bottom, Left};
enum piOpCode{null, update, send};
uint8_t index = 10;
int index2 = 240;

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

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */

static enum Mode mode = colorProfile;
static int rgb_state = 0;

static int receive_signal_state = 0;
static int command = 0;
static int checksum = 0;
static int one_count = 0;

static int bit_pos;
static int lead_time;
static int bit_time;

static int receive_signal_state2 = 0;
static int command2 = 0;
static int checksum2 = 0;
static int one_count2 = 0;

static int bit_pos2;
static int lead_time2;
static int bit_time2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t flag=0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	flag=1;
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
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	//LED Strip Setup
	volatile uint8_t data[400];

	Frame frame;
	frame.startFrame = 0;
	frame.endFrame = 0xFFFFFFFF;
	for (int i =0; i<NUMLED;++i){
		frame.ledarray[i].b = 0;
		frame.ledarray[i].g = 0;
		frame.ledarray[i].r = 0;

		frame.ledarray[i].staticstart = 0b111;
		frame.ledarray[i].globbright = 0b11111;
	}
	HAL_SPI_Transmit(&hspi1, &frame, sizeof(Frame), 10000);

	enum piOpCode op = send;

	//Mode select
	//enum Mode mode = piCam;
	//enum Wanted nextNeeded = Top;
	enum Color currentColor = Green;
	int increment = 15;

	//Color Profiling Mode Config
	int8_t modg = 4;
	int8_t modr = 5;
	int8_t modb = 3;
	uint8_t green = 190;
	uint8_t red = 6;
	uint8_t blue = 40;
	uint16_t delay = 50;


	while (1)
	{

		if (mode == piCam){
			//Wait for start frame 1001
			uint8_t buf[1];
			uint8_t bufR[1];
			bufR[0] = 0;
			buf[0] = op;
			uint8_t stop = 0;
			do{ //handshake
				HAL_UART_Abort(&huart3);
				HAL_DMA_Abort(&hdma_usart3_rx);
				HAL_UART_Transmit(&huart3, buf, 1, 100);
				HAL_UART_Receive(&huart3, bufR, 1, 1000);

				if(mode!=piCam){
					stop=1;
					break;
				}

			}while(bufR[0] != 101);

			if(stop){
				stop=0;
				continue;
			}

			HAL_UART_Abort(&huart3);
			HAL_DMA_Abort(&hdma_usart3_rx);
			HAL_UART_Receive_DMA(&huart3, data, NUMLED2);


			uint8_t bool=0;

			while(flag==0){
				bool;
			}

			for(int i = 0; i < NUMLED; i++)
			{
				float brightness = max(max(data[3*i], data[3*i+1]), data[3*i+2])/255.0;
				float thresh= 0.0;

				uint8_t ledbrite = 0 + ((31 - 0) / (255 - 0)) * (brightness - 0);

				if(i < 37)
				{

					frame.ledarray[37 -i - 1].b =  brightness > thresh ? data[3*i+0]: 0 ;
					frame.ledarray[37 -i - 1].g =  brightness > thresh ? data[3*i+1]: 0 ;
					frame.ledarray[37 -i - 1].r =  brightness > thresh ? data[3*i+2]: 0 ;

				}
				else if(i < 58)
				{
					frame.ledarray[NUMLED -i + 36].b =  brightness > thresh ? data[3*i+0]: 0 ;
					frame.ledarray[NUMLED -i + 36].g =  brightness > thresh ? data[3*i+1]: 0 ;
					frame.ledarray[NUMLED -i + 36].r = brightness > thresh ? data[3*i+2]: 0 ;

				}
				else if(i < 95)
				{
					frame.ledarray[NUMLED - 22 - i + 58].b =  brightness > thresh ? data[3*i+0]: 0 ;
					frame.ledarray[NUMLED - 22 - i + 58].g =  brightness > thresh ?data[3*i+1]: 0 ;
					frame.ledarray[NUMLED - 22 - i + 58].r =  brightness > thresh ? data[3*i+2]: 0 ;

				}
				else
				{
					frame.ledarray[57 -i + 95].b =  brightness > thresh ? data[3*i+0]: 0 ;
					frame.ledarray[57 -i + 95].g = brightness > thresh ? data[3*i+1]: 0 ;
					frame.ledarray[57 -i + 95].r =  brightness > thresh ?data[3*i+2]: 0 ;
				}
			}

			HAL_SPI_Transmit(&hspi1, &frame, sizeof(Frame), 10000);

			//while(1){}
			HAL_Delay(5);
			flag=0;
		}

		else if (mode == refreshBorder){ //Ask pi to update the border, wait for a 101 ACK message back.0
			op = update;
			uint8_t buf[1];
			buf[0] = op;
			uint8_t bufR[1];
			bufR[0] = 0;

			uint8_t stop = 0;

			do{ //handshake
				HAL_UART_Abort(&huart3);
				HAL_DMA_Abort(&hdma_usart3_rx);
				HAL_UART_Transmit(&huart3, buf, 1, 100);
				HAL_UART_Receive(&huart3, bufR, 1, 1000);
				if(mode!=piCam){
					stop=1;
					break;
				}

			}while(bufR[0] != 101);

			if(stop) {
				stop=0;
				continue;
			}


			mode = piCam;
			op = send;
		}

		//Mode 2 -- CP
		else if(mode == colorProfile){
			for (int i =0; i<NUMLED;++i){
				frame.ledarray[i].b = blue;
				frame.ledarray[i].g = green;
				frame.ledarray[i].r = red;
			}
			HAL_SPI_Transmit(&hspi1, &frame, sizeof(Frame), 1000);

			if(green<=5 || green>=250)
				modg*=-1;
			if(red <=5 || red >= 250)
				modr*=-1;
			if(blue <=5 || blue >= 250)
				modb*=-1;

			green+=modg;
			blue+=modb;
			red+=modr;

			HAL_Delay(delay);
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 120-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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
  huart3.Init.BaudRate = 921600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_EnableFifoMode(&huart3) != HAL_OK)
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
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB14 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STLINK_TX_Pin STLINK_RX_Pin */
  GPIO_InitStruct.Pin = STLINK_TX_Pin|STLINK_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_LPUART1;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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

  /*Configure GPIO pins : PD0 PD1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_0) {
		// read data packet
		//		receive_signal();

		switch(receive_signal_state) {
		case 0:
			// measure leading frame
			__HAL_TIM_SET_COUNTER(&htim2,0);
			++receive_signal_state;

			break;
		case 1:
			// validate leading frame
			lead_time = __HAL_TIM_GET_COUNTER(&htim2);
			__HAL_TIM_SET_COUNTER(&htim2,0);

//			if (lead_time > 23500 && lead_time < 25500) {
//				++receive_signal_state;
//				bit_pos = 3;
//			} else {
//				--receive_signal_state;
//			}


			if (lead_time > 20 && lead_time < 30000) {
				++receive_signal_state;
				bit_pos = 3;
			} else {
				--receive_signal_state;
			}


			break;
		case 2:
			// read 4-bit command
			bit_time = __HAL_TIM_GET_COUNTER(&htim2);
			__HAL_TIM_SET_COUNTER(&htim2,0);

			if (bit_time > 3000) {
				command |= (1 << bit_pos);
				++one_count;
			}

			--bit_pos;

			if (bit_pos == -1) {
				++receive_signal_state;
				bit_pos = 2;
			}

			break;
		case 3:
			// read 3-bit checksum
			bit_time = __HAL_TIM_GET_COUNTER(&htim2);
			__HAL_TIM_SET_COUNTER(&htim2,0);

			if (bit_time > 3000) {
				checksum |= (1 << bit_pos);
			}

			--bit_pos;


			// set and reset
			if (bit_pos == -1) {
				if (checksum == one_count) {
					mode = command;
				}

				command = 0;
				checksum = 0;
				one_count = 0;
				receive_signal_state = 0;
			}

			break;
		}

	}



	if (GPIO_Pin == GPIO_PIN_1) {
		// read data packet
		//		receive_signal();

		switch(receive_signal_state2) {
		case 0:
			// measure leading frame
			__HAL_TIM_SET_COUNTER(&htim2,0);
			++receive_signal_state2;

			break;
		case 1:
			// validate leading frame
//			lead_time2 = __HAL_TIM_GET_COUNTER(&htim2);
//			__HAL_TIM_SET_COUNTER(&htim2,0);
//
//			if (lead_time2 > 23500 && lead_time2 < 25500) {
//				++receive_signal_state2;
//				bit_pos2 = 3;
//			} else {
//				--receive_signal_state2;
//			}


			lead_time2 = __HAL_TIM_GET_COUNTER(&htim2);
			__HAL_TIM_SET_COUNTER(&htim2,0);

			if (lead_time2 > 30 && lead_time2 < 30000) {
				++receive_signal_state2;
				bit_pos2 = 3;
			} else {
				--receive_signal_state2;
			}

			break;
		case 2:
			// read 4-bit command
			bit_time2 = __HAL_TIM_GET_COUNTER(&htim2);
			__HAL_TIM_SET_COUNTER(&htim2,0);

			if (bit_time2 > 3000) {
				command2 |= (1 << bit_pos2);
				++one_count2;
			}

			--bit_pos2;

			if (bit_pos2 == -1) {
				++receive_signal_state2;
				bit_pos2 = 2;
			}

			break;
		case 3:
			// read 3-bit checksum
			bit_time2 = __HAL_TIM_GET_COUNTER(&htim2);
			__HAL_TIM_SET_COUNTER(&htim2,0);

			if (bit_time2 > 3000) {
				checksum2 |= (1 << bit_pos2);
			}

			--bit_pos2;


			// set and reset
			if (bit_pos2 == -1) {
				if (checksum2 == one_count2) {
					mode = command2;
				}

				command2 = 0;
				checksum2 = 0;
				one_count2 = 0;
				receive_signal_state2 = 0;
			}

			break;
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
