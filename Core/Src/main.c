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
#include <stdio.h>
#include "pwmPidControl.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ONE_LAP 360 //360 degrees in one lap
#define PPR 1024 //encoder resolution in pulses per revolution
#define MOTOR_MAX_PULSES 70 //encoder pulses when motor @ 24V

#define MILISECONDS_TO_SECONDS 0.001 //conversion
#define RPM_TO_DEGREES_PER_SECOND 6 //conversion
#define HIGH_LEVEL_INPUT_TO_PWM_PERCENTAGE 1/127


//controller defines
#define MAX_PWM 100
#define MIN_PWM -100
#define KP_A 0.03
#define KI_A 20
#define KD_A 0

#define KP_B 10
#define KI_B 0
#define KD_B 0

#define STEP_BUFFER_SIZE 10
#define TIMER_INIT_VALUE 30000 //this way, there is no need to deal with overflow
#define CCR_LIMIT 999
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
int A_Step = 0; //stores encoder pulses
int B_Step = 0;
float A_Speed = 0; //speed in pulses / s
float B_Speed = 0;
float pastTime = 0; //time since application started, used for graphics
uint32_t currentTick, lastTick = 0; //stores clock ticks


int A_Buffer[STEP_BUFFER_SIZE];
int B_Buffer[STEP_BUFFER_SIZE];
float A_Sum = 0, B_Sum =0;
int i, j;


//CONTROL VARIABLES
Pid A_MotorController;
Pid B_MotorController;
float A_Duty = 0; //for floating point operations
float B_Duty = 0;
float A_SetPoint = 0; //setPoint in pulses/s
float B_SetPoint = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

//for data transmission using FTDI, requires USART 1 enabled
int _write(int fd, char* ptr, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t *) ptr, len, HAL_MAX_DELAY);
    return len;
}

// driver logic config so motor goes clockwise
void A_MotorClockWise();

// driver logic config so motor goes counter-clockwise
void A_MotorCounterClockWise();

// driver logic config so motor stops
void A_MotorStop();

//same for motor B
void B_MotorClockWise();
void B_MotorCounterClockWise();
void B_MotorStop();

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1); //starts PWM timer
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2); //starts PWM timer
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); //starts encoder timer for motor 1
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); //starts encoder timer for motor 2

  //same kp, ki, kd for both motors
  pidInit(&A_MotorController, MIN_PWM, MAX_PWM, KP_A, KI_A, KD_A); //starts PID controller
  pidInit(&B_MotorController, MIN_PWM, MAX_PWM, KP_B, KI_B, KD_B); //starts PID controller


  HAL_GPIO_WritePin(GPIOF, ENABLE_A_Pin, GPIO_PIN_SET); //enable A ON (required)
  HAL_GPIO_WritePin(GPIOF, ENABLE_B_Pin, GPIO_PIN_SET); //enable B ON (required)

  //motors initially stopped
  A_MotorStop();
  B_MotorStop();
  //TIM5->CC1: PWM FOR MOTOR A
  //TIM5->CCR2 PWM FOR MOTOR B
  TIM5->CCR1 = 0; TIM5->CCR2 = 0; //PWM timer inittialy 0


  for(i= 0; i< STEP_BUFFER_SIZE; i++){
	  A_Buffer[i] = 0;
	  B_Buffer[i] = 0;
  }
  i=0;

  TIM3->CNT = TIMER_INIT_VALUE;
  TIM4->CNT = TIMER_INIT_VALUE;
//  A_MotorClockWise();
//  B_MotorClockWise();
  HAL_TIM_Base_Start_IT(&htim2); //starts interrupt timer
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 720;
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
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 14;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, ENABLE_A_Pin|ENABLE_B_Pin|A_IN_1_Pin|A_IN_2_Pin
                          |B_IN_1_Pin|B_IN_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ENABLE_A_Pin ENABLE_B_Pin A_IN_1_Pin A_IN_2_Pin
                           B_IN_1_Pin B_IN_2_Pin */
  GPIO_InitStruct.Pin = ENABLE_A_Pin|ENABLE_B_Pin|A_IN_1_Pin|A_IN_2_Pin
                          |B_IN_1_Pin|B_IN_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//control interrupt
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
	if(htim == &htim2){

		currentTick = HAL_GetTick(); //para contar o tempo decorrido (gerar gráfico)

		A_Step = TIM3->CNT - TIMER_INIT_VALUE;
		B_Step = (TIM4->CNT - TIMER_INIT_VALUE) * -1; //o motor está de cabeça pra baixo no andador k

		TIM3->CNT = TIMER_INIT_VALUE;
		TIM4->CNT = TIMER_INIT_VALUE;

		pastTime += (float)(currentTick - lastTick) / 1000;

		/*Testes de velocidade*/

		if(pastTime > 5) A_SetPoint = 1000;
		if(pastTime > 5) B_SetPoint = 1000;

		//dealing with different kinds of movement for motor A
		if(A_SetPoint > 0) A_MotorClockWise();
		//antiClockWise movement
		else if(A_SetPoint < 0) A_MotorCounterClockWise();
		//no movement
		else {
			A_MotorStop();
			TIM5->CCR1 = 0; //0% dutyCycle
			A_Duty = 0; //avoids biasing next motor speed
		}
		//dealing with different kinds of movement for motor B
		if(B_SetPoint > 0) B_MotorClockWise();
		//antiClockWise movement
		else if(B_SetPoint < 0) B_MotorCounterClockWise();
		//no movement
		else{
			B_MotorStop();
			TIM5->CCR2 = 0; //0% dutyCycle
			B_Duty = 0; //avoids biasing next motor speed
		}

		if(i >= 10) i=0; //creates queue structure in the buffer

		//CONTROLLER
		A_Buffer[i] = A_Step;
		B_Buffer[i] = B_Step;
		i++;

		for(j=0; j< STEP_BUFFER_SIZE; j++){
			A_Sum += A_Buffer[j];
			B_Sum += B_Buffer[j];
		}

		A_Sum /= STEP_BUFFER_SIZE;
		B_Sum /= STEP_BUFFER_SIZE;

		// current speed in pulses / s
		A_Speed = A_Sum * 100;
		B_Speed = B_Sum * 100;

		A_Duty += computePwmValue(A_SetPoint, A_Speed, &A_MotorController); //gets pwm variation in floating point
		B_Duty += computePwmValue(B_SetPoint, B_Speed, &B_MotorController); //gets pwm variation in floating point

		//considers CCR limits for motor A
		if(A_Duty > CCR_LIMIT) A_Duty = CCR_LIMIT;
		else if(A_Duty < 0) A_Duty = 0;

		//considers CCR limits for motor B
		if(B_Duty > CCR_LIMIT) B_Duty = CCR_LIMIT;
		else if(B_Duty < 0) B_Duty = 0;

		//CCR in 999 -> DUTY CYCLE 100%
		//CCR in 0 -> DUTY CYCLE 0%
		//configurable in .ioc
		TIM5->CCR1 = A_Duty; //updates pwm for motor A
		TIM5->CCR2 = B_Duty; //updates pwm for motor B

//		debug printing
//		printf("currentStep: %d setPoint: %f\r\n", currentStep, pulsesSetPoint); //encoder step and setPoint
//		printf("PWM: %ld\r\n", TIM1->CCR4); //pwm register
		printf("%f %f %f\n\r", A_Speed, B_Speed, pastTime); //speed and time formatted for python script (graphic)
//		printf("%ld\r\n", currentTick - lastTick); //time between interrupts
//		printf("%f \n\r", auxSum);

		//updates for next iteration
		A_Sum = 0;
		B_Sum = 0;
		lastTick = currentTick;
	}
}

/**
 * IN A: ON
 * IN B: OFF
 * */
void A_MotorClockWise(){
	HAL_GPIO_WritePin(GPIOF, A_IN_1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOF, A_IN_2_Pin, GPIO_PIN_RESET);
}

/**
 * IN A: OFF
 * IN B: ON
 * */
void A_MotorCounterClockWise(){
	HAL_GPIO_WritePin(GPIOF, A_IN_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF, A_IN_2_Pin, GPIO_PIN_SET);
}

/**
 * IN A: OFF
 * IN B: OFF
 * */
//brakes to GND
void A_MotorStop(){
	HAL_GPIO_WritePin(GPIOF, A_IN_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF, A_IN_2_Pin, GPIO_PIN_RESET);
}


void B_MotorCounterClockWise(){
	HAL_GPIO_WritePin(GPIOF, B_IN_1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOF, B_IN_2_Pin, GPIO_PIN_RESET);
}

/**
 * IN 1: OFF
 * IN 2: ON
 * */
void B_MotorClockWise(){
	HAL_GPIO_WritePin(GPIOF, B_IN_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF, B_IN_2_Pin, GPIO_PIN_SET);
}

/**
 * IN A: OFF
 * IN B: OFF
 * */
//brakes to GND
void B_MotorStop(){
	HAL_GPIO_WritePin(GPIOF, B_IN_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF, B_IN_2_Pin, GPIO_PIN_RESET);
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
