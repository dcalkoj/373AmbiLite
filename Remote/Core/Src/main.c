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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim15;

/* USER CODE BEGIN PV */
//static int send_signal = 0;
static int button_num = 0;

static int output_state = 0;


enum Remote_States {
	idle,
	send_signal,
	wait_for_input,
	receive_NEC_signal,
	assign_button,
	send_NEC_signal
};

enum Remote_States global_state = idle;

static uint32_t received_data = 0;
static uint32_t queue_data = 0;

// Universal Remote


enum Buttons {
	b0800,
	b0801,
	b1100,
	b4
};


enum Button_function {
	default__,
	alternate_function
};




// Pin 5 Case 0
static enum Button_function b0500_func = default__;
static uint32_t b0500_data;



// Pin 5 Case 1
static enum Button_function b0501_func = default__;
static uint32_t b0501_data;


// Pin 5 Case 2
static enum Button_function b0502_func = default__;
static uint32_t b0502_data;






// Pin 8 Case 0
static enum Button_function b0800_func = default__;
static uint32_t b0800_data;



// Pin 8 Case 1
static enum Button_function b0801_func = default__;
static uint32_t b0801_data;


// Pin 8 Case 2
static enum Button_function b0802_func = default__;
static uint32_t b0802_data;






// Pin 11 Case 2
static enum Button_function b1102_func = default__;
static uint32_t b1102_data;




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM15_Init(void);
/* USER CODE BEGIN PFP */
void delay_us(uint16_t us);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void send_bit(int bit) {
	if (bit) {

		TIM2->CCR1 = 52;
		delay_us(1050);
		TIM2->CCR1 = 0;
		delay_us(3050);
	} else {

		TIM2->CCR1 = 52;
		delay_us(1050);
		TIM2->CCR1 = 0;
		delay_us(1050);
	}
}


void delay_us(uint16_t us) {
	__HAL_TIM_SET_COUNTER(&htim1,0);
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);
}

void delay_ms(uint16_t ms) {
//	__HAL_TIM_SET_COUNTER(&htim15,0);
//	while (__HAL_TIM_GET_COUNTER(&htim15) < ms);

	int ms_elapsed = 0;
	while(ms_elapsed < ms) {
		delay_us(1000);
		++ms_elapsed;
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM7_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Base_Start(&htim15);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
//  TIM2->CCR1 = 0;

//  HAL_NVIC_DisableIRQ(EXTI2_IRQn);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

//	  switchStart:

		switch(global_state) {
		case idle:

			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

			break;

		case send_signal:


			for (int t = 0; t < 5; ++t) {
				// Leading frame

				TIM2->CCR1 = 52;
				delay_us(16050);
				TIM2->CCR1 = 0;
				delay_us(8050);


				int checksum = 0;
				int command = button_num;

				// 4-bit command
				for (int i = 0; i < 4; ++i) {
					int bit = 1 & (command >> (3-i));

					if (bit) {
						++checksum;
					}

					send_bit(bit);
				}


				// 3-bit checksum
				for (int i = 0; i < 3; ++i) {
					int bit = 1 & (checksum >> (2-i));

					send_bit(bit);
				}


				// End frame

				TIM2->CCR1 = 52;
				delay_us(2050);
				TIM2->CCR1 = 0;


				delay_us(10000);

			}


			global_state = idle;

			break;

		case wait_for_input:


			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

			break;

		case receive_NEC_signal:

			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
			// wait for signal
			while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) && global_state == receive_NEC_signal);

			if (global_state == receive_NEC_signal) {

				// Leading frame
				while(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)); // 9ms
				while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)); // 4.5ms


				// Read 32 bits

				for (int i = 0; i < 32; ++i) {

					while(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)); // 562.5 us burst


					__HAL_TIM_SET_COUNTER(&htim1,0);
					while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2));

					if (__HAL_TIM_GET_COUNTER(&htim1) > 1000) {

						// 1b
						received_data |= (1 << (31-i));
					}

				}


				while(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)); // 562.5 us final burst

				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
				global_state = assign_button;

			} else {

				global_state = idle;

			}


			break;

		case assign_button:

			// bool

			break;
		case send_NEC_signal:


			for (int t = 0; t < 3; ++t) {

				// Leading frame
				TIM2->CCR1 = 52;
				delay_us(8810);
				TIM2->CCR1 = 0;
				delay_us(4454);



				// Send 32-bit data

				for (int i = 0; i < 32; ++i) {

					if (queue_data & (1 << (31-i))) {

						TIM2->CCR1 = 52;
						delay_us(527);
						TIM2->CCR1 = 0;
						delay_us(1670);

					} else {

						TIM2->CCR1 = 52;
						delay_us(527);
						TIM2->CCR1 = 0;
						delay_us(562);

					}

				}

				// Final burst

				TIM2->CCR1 = 52;
				delay_us(527);
				TIM2->CCR1 = 0;


				HAL_Delay(36);

			}


			global_state = idle;


			break;
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 4-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 60000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 104;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 4-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1000-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 4000-1;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {



		if (GPIO_Pin == GPIO_PIN_5) {


			switch(output_state) {
			case 0:



				switch(global_state) {
				case idle:


					switch(b0500_func) {
					case default__:

//						button_num = 0;
//						global_state = send_signal;

						queue_data = 0xCDFE4D18;
						global_state = send_NEC_signal;

						break;
					case alternate_function:

						queue_data = b0500_data;
						global_state = send_NEC_signal;

						break;
					}


					break;
					case send_signal:

						// bool

						break;
					case wait_for_input:

						b0500_func = default__;
						global_state = idle;

						break;
					case receive_NEC_signal:

						// bool

						break;
					case assign_button:

						b0500_func = alternate_function;
						b0500_data = received_data;


						received_data = 0;
						global_state = idle;

						break;
					case send_NEC_signal:
						// bool
						break;
				}



				break;


				case 1:



					switch(global_state) {
					case idle:


						switch(b0501_func) {
						case default__:

//							button_num = 1;
//							global_state = send_signal;

							queue_data = 0xD4081C98;
							global_state = send_NEC_signal;

							break;
						case alternate_function:

							queue_data = b0501_data;
							global_state = send_NEC_signal;

							break;
						}


						break;
						case send_signal:

							// bool

							break;
						case wait_for_input:

							b0501_func = default__;
							global_state = idle;

							break;
						case receive_NEC_signal:

							// bool

							break;
						case assign_button:

							b0501_func = alternate_function;
							b0501_data = received_data;


							received_data = 0;
							global_state = idle;

							break;
						case send_NEC_signal:
							// bool
							break;
					}






					break;

				case 2:

					switch(global_state) {
					case idle:


						switch(b0502_func) {
						case default__:

//							button_num = 2;
//							global_state = send_signal;

							queue_data = 0xF51E64D2;
							global_state = send_NEC_signal;

							break;
						case alternate_function:

							queue_data = b0502_data;
							global_state = send_NEC_signal;

							break;
						}


						break;
						case send_signal:

							// bool

							break;
						case wait_for_input:

							b0502_func = default__;
							global_state = idle;

							break;
						case receive_NEC_signal:

							// bool

							break;
						case assign_button:

							b0502_func = alternate_function;
							b0502_data = received_data;


							received_data = 0;
							global_state = idle;

							break;
						case send_NEC_signal:
							// bool
							break;
					}

					break;
			}


		}


		if (GPIO_Pin == GPIO_PIN_8) {


			switch(output_state) {
			case 0:



				switch(global_state) {
				case idle:


					switch(b0800_func) {
					case default__:

						button_num = 3;
						global_state = send_signal;

						break;
					case alternate_function:

						queue_data = b0800_data;
						global_state = send_NEC_signal;

						break;
					}


					break;
					case send_signal:

						// bool

						break;
					case wait_for_input:

						b0800_func = default__;
						global_state = idle;

						break;
					case receive_NEC_signal:

						// bool

						break;
					case assign_button:

						b0800_func = alternate_function;
						b0800_data = received_data;


						received_data = 0;
						global_state = idle;

						break;
					case send_NEC_signal:
						// bool
						break;
				}



				break;


				case 1:



					switch(global_state) {
					case idle:


						switch(b0801_func) {
						case default__:

							button_num = 4;
							global_state = send_signal;

							break;
						case alternate_function:

							queue_data = b0801_data;
							global_state = send_NEC_signal;

							break;
						}


						break;
						case send_signal:

							// bool

							break;
						case wait_for_input:

							b0801_func = default__;
							global_state = idle;

							break;
						case receive_NEC_signal:

							// bool

							break;
						case assign_button:

							b0801_func = alternate_function;
							b0801_data = received_data;


							received_data = 0;
							global_state = idle;

							break;
						case send_NEC_signal:
							// bool
							break;
					}






					break;

				case 2:

					switch(global_state) {
					case idle:


						switch(b0802_func) {
						case default__:

							button_num = 5;
							global_state = send_signal;

							break;
						case alternate_function:

							queue_data = b0802_data;
							global_state = send_NEC_signal;

							break;
						}


						break;
						case send_signal:

							// bool

							break;
						case wait_for_input:

							b0802_func = default__;
							global_state = idle;

							break;
						case receive_NEC_signal:

							// bool

							break;
						case assign_button:

							b0802_func = alternate_function;
							b0802_data = received_data;


							received_data = 0;
							global_state = idle;

							break;
						case send_NEC_signal:
							// bool
							break;
					}

					break;
			}
		}




		if (GPIO_Pin == GPIO_PIN_11) {
			switch(output_state) {

			// Secondary Program Button
			case 0:


				switch(global_state) {
				case idle:





					break;


					case send_signal:

						// bool

						break;
					case wait_for_input:


						global_state = receive_NEC_signal;
						delay_ms(500);

						break;
					case receive_NEC_signal:

						// bool

						break;
					case assign_button:


						break;
					case send_NEC_signal:
						// bool
						break;
				}

				break;


			// Primary Program Button
			case 1:



				switch(global_state) {
				case idle:


					global_state = wait_for_input;

					delay_ms(500);

					break;
				case send_signal:
					// bool
					break;
				case wait_for_input:

					break;
				case receive_NEC_signal:
					// bool


					global_state = idle;
					delay_ms(500);
					break;
				case assign_button:
					received_data = 0;

					global_state = idle;
					delay_ms(500);
					break;

				case send_NEC_signal:
					// bool
					break;
				}


				break;

			case 2:

				switch(global_state) {
				case idle:


					switch(b1102_func) {
					case default__:

						button_num = 6;
						global_state = send_signal;

						break;
					case alternate_function:

						queue_data = b1102_data;
						global_state = send_NEC_signal;

						break;
					}


					break;
					case send_signal:

						// bool

						break;
					case wait_for_input:

						b1102_func = default__;
						global_state = idle;

						break;
					case receive_NEC_signal:

						// bool

						break;
					case assign_button:

						b1102_func = alternate_function;
						b1102_data = received_data;


						received_data = 0;
						global_state = idle;

						break;
					case send_NEC_signal:
						// bool
						break;
				}

				break;
			}
		}

//	}


}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	++output_state;

	if (output_state == 3) {
		output_state = 0;
	}


	switch(output_state) {
	case 0:
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
		break;
	case 1:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
		break;
	case 2:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
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
