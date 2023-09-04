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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

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
	uint32_t i,d;
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
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  //------------------------------------------- Begin 600 Hz 167us
	  //TIM1->CCR1=3800;
	  //------------------------------------------- End 600 Hz 167us

	  //------------------------------------------- Begin 620 Hz 161us
	  //TIM1->CCR1=3850;
	  //------------------------------------------- End 620 Hz 161us

	  //------------------------------------------- Begin 630 Hz 161us
	  //TIM1->CCR1=3900;
	  //------------------------------------------- End 630 Hz 161us

	  //------------------------------------------- Begin 1000 Hz 100us
	  //TIM1->CCR1=2400;
	  //------------------------------------------- End 1000 Hz 100us

	  //------------------------------------------- Begin 2000 Hz 50us
	  //TIM1->CCR1=1200;
	  //------------------------------------------- End 2000 Hz 50us

	  //------------------------------------------- Begin 3000 Hz 33us
	  //TIM1->CCR1=790;
	  //------------------------------------------- End 3000 Hz 33us

	  //------------------------------------------- Begin 3980 Hz 25us
	  //TIM1->CCR1=600;
	  //------------------------------------------- End 3980 Hz 25us

	  //------------------------------------------- Begin 5000 Hz 20us
	  //TIM1->CCR1=485;
	  //------------------------------------------- End 5000 Hz 20us

	  //------------------------------------------- Begin 6000 Hz 17us
	  //TIM1->CCR1=408;
	  //------------------------------------------- End 6000 Hz 17us

	  //------------------------------------------- Begin 7960 Hz 13us
	  //TIM1->CCR1=358;
	  //------------------------------------------- End 7960 Hz 13us

	  //------------------------------------------- Begin 9000 Hz 11us
	  TIM1->CCR1=265;
	  //------------------------------------------- End 9000 Hz 11us


	  //TIM1->CCR1=357;  /*10us 1250Hz*/
	  //TIM1->CCR1=18; //10us при 30 Гц htim1.Init.Period = 65535;

	  //TIM1->CCR1=70; // 1us при 1кГц htim1.Init.Period = 65535;
	  //TIM1->CCR1=360; // 5us при 1кГц htim1.Init.Period = 65535;
	  //TIM1->CCR1=1; //1us при 10 Гц htim1.Init.Period = 65535;
	  //TIM1->CCR1=2; //1us при 30 Гц htim1.Init.Period = 65535;
	  //TIM1->CCR1=7; //1us при 10Ф0 Гц htim1.Init.Period = 65535;
	  //TIM1->CCR1=25; //1us при 300 Гц htim1.Init.Period = 65535;
	  //TIM1->CCR1=38; //1us при 630 Гц htim1.Init.Period = 65535;

	  //TIM1->CCR1=5; //5us при 10 Гц htim1.Init.Period = 65535;
	  //TIM1->CCR1=10; //5us при 30 Гц htim1.Init.Period = 65535;
	  //TIM1->CCR1=14; //5us при 100 Гц htim1.Init.Period = 65535;
	  //TIM1->CCR1=125; //5us при 300 Гц htim1.Init.Period = 65535;
	  //TIM1->CCR1=180; //5us при 630 Гц htim1.Init.Period = 65535;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */


  htim1.Instance = TIM1;
  //------------------------------------------- Begin 600 Hz
  //htim1.Init.Prescaler = 2; // 600 Hz
  //htim1.Init.Period = 39998;
  //------------------------------------------- End 600 Hz

  //------------------------------------------- Begin 620 Hz
  //htim1.Init.Prescaler = 2; // 620 Hz
  //htim1.Init.Period = 38705;
  //------------------------------------------- End 620 Hz

  //------------------------------------------- Begin 630 Hz
  //htim1.Init.Prescaler = 2; // 630 Hz
  //htim1.Init.Period = 38090;
  //------------------------------------------- End 630 Hz

  //------------------------------------------- Begin 1000 Hz
  //htim1.Init.Prescaler = 2; // 1000 Hz
  //htim1.Init.Period = 23998;
  //------------------------------------------- End 1000 Hz

  //------------------------------------------- Begin 2000 Hz
  //htim1.Init.Prescaler = 2; // 2000 Hz
  //htim1.Init.Period = 11998;
  //------------------------------------------- End 2000 Hz

  //------------------------------------------- Begin 3000 Hz
  //htim1.Init.Prescaler = 2; // 3000 Hz
  //htim1.Init.Period = 7998;
  //------------------------------------------- End 3000 Hz

  //------------------------------------------- Begin 3980 Hz
  //htim1.Init.Prescaler = 2; // 3980 Hz
  //htim1.Init.Period = 6028;
  //------------------------------------------- End 3980 Hz

  //------------------------------------------- Begin 3980 Hz
  //htim1.Init.Prescaler = 2; // 3980 Hz
  //htim1.Init.Period = 6028;
  //------------------------------------------- End 3980 Hz

  //------------------------------------------- Begin 5000 Hz
  //htim1.Init.Prescaler = 2; // 5000 Hz
  //htim1.Init.Period = 4798;
  //------------------------------------------- End 5000 Hz

  //------------------------------------------- Begin 6000 Hz
  //htim1.Init.Prescaler = 2; // 6000 Hz
  //htim1.Init.Period = 3998;
  //------------------------------------------- End 6000 Hz

  //------------------------------------------- Begin 6000 Hz
  //htim1.Init.Prescaler = 2; // 6000 Hz
  //htim1.Init.Period = 3998;
  //------------------------------------------- End 6000 Hz

  //------------------------------------------- Begin 7960 Hz
  //htim1.Init.Prescaler = 2; // 7960 Hz
  //htim1.Init.Period = 3014;
  //------------------------------------------- End 7960 Hz

  //------------------------------------------- Begin 9000 Hz
  //htim1.Init.Prescaler = 2; // 9000 Hz
  //htim1.Init.Period = 2665; //
  //------------------------------------------- End 9000 Hz

  //------------------------------------------- Begin 10000 Hz
  htim1.Init.Prescaler = 2; // 10000 Hz
  htim1.Init.Period = 2365; //
  //------------------------------------------- End 10000 Hz

    /*if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11)==GPIO_PIN_SET)
    {
  	  htim1.Init.Prescaler = 100; // 10 Hz
  	  htim1.Init.Period = 65535;
    }

    if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10)==GPIO_PIN_SET)
    {
    	htim1.Init.Prescaler = 35; // 30 Hz
    	htim1.Init.Period = 65535;
    }

    if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)==GPIO_PIN_SET)
    {
    	htim1.Init.Prescaler = 10; // 100 Hz
    	htim1.Init.Period = 65100;
    }

    if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)==GPIO_PIN_SET)
    {
    	htim1.Init.Prescaler = 7; // 300 Hz
    	htim1.Init.Period = 30000;
    }

    else
    {
    	htim1.Init.Prescaler = 1; // 1250 Hz
    	htim1.Init.Period = 28780;

    }
*/


  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  //htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
