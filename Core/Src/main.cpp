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
#include	<stdio.h>
#include	<string.h>
#include "main.h"
#include "StepperMotorDriver.h"
#include "StepperTimerManager.h"
#include	"Timer.h"
#include	"CommandHandler.h"
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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2) {
		TimerManager::getInstance().updateAll();
	} else if (htim->Instance == TIM3) {
		StepperTimerManager::Instance().OnTimerTick();
	}
}

// Abschnitte = X => PreLoad, Y => Load, Z => Unload
StepperMotorDriver stepperPreLoad(GPIOA, GPIO_PIN_6,   // STEP
                                  	GPIOA, GPIO_PIN_1,   // DIR
									GPIOA, GPIO_PIN_0,
									800,
									40) ; // EN (aktiv LOW)

StepperMotorDriver stepperLoad(GPIOB, GPIO_PIN_0,
									GPIOB, GPIO_PIN_1,
									GPIOB, GPIO_PIN_2,
									800,
									48);
StepperMotorDriver stepperUnload(GPIOB, GPIO_PIN_4,
									GPIOB, GPIO_PIN_5,
									GPIOB, GPIO_PIN_6,
									800,
									40);

void InitSteppers()
{
    stepperPreLoad.Init();
    stepperLoad.Init();
    stepperUnload.Init();

    // Beim Start werden sie registriert, aber sie bleiben deaktiviert,
    // bis du explizit Start() aufrufst.
    stepperPreLoad.SetMaxSpeed(1600.0f);      // 2000 steps/s
    stepperPreLoad.SetAcceleration(5000.0f);  // 1000 steps/s²
    stepperPreLoad.SetJerk(10000.0f);          // 5000 steps/s³

    StepperTimerManager::Instance().RegisterStepper(&stepperPreLoad);
    StepperTimerManager::Instance().RegisterStepper(&stepperLoad);
    StepperTimerManager::Instance().RegisterStepper(&stepperUnload);

    stepperPreLoad.SetEndswitch(StepperPosition::PRELOAD_POS, ES_LOADED_GPIO_Port, ES_LOADED_Pin);
}
void timer1Callback() {
//	static	bool	val	=	false ;
//	uint8_t rx = val ? '*' : '#' ;
//	HAL_UART_Transmit(&huart2, &rx, 1, HAL_MAX_DELAY);
//	HAL_GPIO_WritePin( GPIOA, GPIO_PIN_1, val ? GPIO_PIN_RESET : GPIO_PIN_SET);
//	val	=	val ? false : true ;
}
void g0Handler(float p, float l, float u, float f) {
	stepperPreLoad.SetMaxSpeed((float) f);      // 2000 steps/s
	if ( f == 0.0f) {
		stepperPreLoad.Stop();
		stepperLoad.Stop();
		stepperUnload.Stop();
	} else {
		stepperPreLoad.StartAbs( p);
		stepperLoad.StartAbs( l);
		stepperUnload.StartAbs( u);
	}
}
void g1Handler(float p, float l, float u) {
	stepperPreLoad.StartRel( p);
	stepperLoad.StartRel( l);
	stepperUnload.StartRel( u);
}
void g2Handler() {
	stepperPreLoad.Stop();
	stepperLoad.Stop();
	stepperUnload.Stop();
}
void m18Handler(void) {
	stepperPreLoad.EnableOutput( false) ;
	stepperLoad.EnableOutput( false) ;
	stepperUnload.EnableOutput( false) ;
}
void m19Handler(void) {
	stepperPreLoad.EnableOutput( true) ;
	stepperLoad.EnableOutput( true) ;
	stepperUnload.EnableOutput( true) ;
}

void m114Handler(void) {
	uint8_t	buffer[64] ;
	snprintf(( char *) buffer, 64, "P:%d L:%d U:%d\n",
			( int) stepperPreLoad.GetCurrentPositionMm(),
			( int) stepperLoad.GetCurrentPositionMm(),
			( int) stepperUnload.GetCurrentPositionMm()) ;
	HAL_UART_Transmit(&huart2, buffer, strlen(( const char *) buffer), HAL_MAX_DELAY);
}

void m119Handler(void) {
	uint8_t	buffer[64] ;
	snprintf(( char *) buffer, 64, "pre-load %s\n", ( stepperPreLoad.GetPositionState( StepperPosition::PRELOAD_POS) == PositionState::POSITION_LOADED) ? "loaded" : "unloaded") ;
	HAL_UART_Transmit(&huart2, buffer, strlen(( const char *) buffer), HAL_MAX_DELAY);
	snprintf(( char *) buffer, 64, "load %s\n", ( stepperLoad.GetPositionState( StepperPosition::LOAD_POS) == PositionState::POSITION_LOADED) ? "loaded" : "unloaded") ;
	HAL_UART_Transmit(&huart2, buffer, strlen(( const char *) buffer), HAL_MAX_DELAY);
	snprintf(( char *) buffer, 64, "unload %s\n", ( stepperUnload.GetPositionState( StepperPosition::UNLOAD_POS) == PositionState::POSITION_LOADED) ? "loaded" : "unloaded") ;
	HAL_UART_Transmit(&huart2, buffer, strlen(( const char *) buffer), HAL_MAX_DELAY);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
    CommandInterpreter interpreter;
	Timer timer1(2500, timer1Callback);  // 1 Sekunde
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
  /* USER CODE BEGIN 2 */
	InitSteppers() ;
	HAL_TIM_Base_Start_IT(&htim2);		//	start TIM2 with interrupt => 10ms (sps)timer
	HAL_TIM_Base_Start_IT(&htim3);		//	start TIM2 with interrupt => 1ms stepper motors
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t rx;
  // Registriere Callbacks
  interpreter.setG0Callback(g0Handler);
  interpreter.setG1Callback(g1Handler);
  interpreter.setG2Callback(g2Handler);
  interpreter.setM18Callback(m18Handler);
  interpreter.setM19Callback(m19Handler);
  interpreter.setM114Callback(m114Handler);
  interpreter.setM119Callback(m119Handler);
  while (1)
  {
	if (HAL_UART_Receive(&huart2, &rx, 1, HAL_MAX_DELAY) == HAL_OK) {
		interpreter.addChar( rx);
//		HAL_UART_Transmit(&huart2, &rx, 1, HAL_MAX_DELAY);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
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
  htim2.Init.Prescaler = 6400-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 6400-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
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
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  huart2.Init.BaudRate = 38400;
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|LD2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ES_LOADED_Pin */
  GPIO_InitStruct.Pin = ES_LOADED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ES_LOADED_GPIO_Port, &GPIO_InitStruct);

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
