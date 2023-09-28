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


//definições dos controles

//maiores e menores mudanças no PWM
#define MAX_PWM 200
#define MIN_PWM -200


//Parametros para o motor A (podem ser ajustados)
#define KP_A 0.04
#define KI_A 0.00001
#define KD_A 0

//parametros para o motor B (podem ser ajustados)
#define KP_B 0.04
#define KI_B 0.00001
#define KD_B 0

//valores utilizados no código
#define STEP_BUFFER_SIZE 10
#define TIMER_INIT_VALUE 30000 //para eviar lidar com overflow
#define CCR_LIMIT 999
#define DELTA_T 0.01 //intervalo entre interrupções
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
//passo de cada encoder: Como está em modo x4 (.ioc), um ciclo de pulsos equivale a aumentar o contador em 4
int A_Step = 0;
int B_Step = 0;
//velocidade em pulsos/s
float A_Speed = 0;
float B_Speed = 0;
//tempo desde o início da aplicação, usado para gerar gráficos
float pastTime = 0;
//contagem dos ticks do clock
uint32_t currentTick, lastTick = 0;

//buffers para atuar como filtro média móvel
int A_Buffer[STEP_BUFFER_SIZE];
int B_Buffer[STEP_BUFFER_SIZE];
//variáveis usadas na interrupção
float A_Sum = 0, B_Sum =0;
int i, j;


//variáveis de controle
Pid A_MotorController;
Pid B_MotorController;
//para receber valores float e não acontecer perdas de valores decimais
float A_Duty = 0;
float B_Duty = 0;
//velocidades alvos definidas pelo alto nível
//TODO: realizar integração com alto nível obtendo estas velocidades
float A_SetPoint = 0;
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

//Para transmitir dados usando uma FTDI e putty. Precisa do UART1 ligado
int _write(int fd, char* ptr, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t *) ptr, len, HAL_MAX_DELAY);
    return len;
}

// configuração lógica do driver para que o motor A gire no sentido horário
void A_MotorClockWise();

// configuração lógica do driver para que o motor A gire no sentido anti-horário
void A_MotorCounterClockWise();

// configuração lógica do driver para que o motor A não gire
void A_MotorStop();

//análogo para o motor B
//a lógica dentro da função é invertida pois o motor está de cabeça para baixo no andador
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

  //instancia dois canais de PWM do timer 1
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1); //motor A
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2); //motor B

  //instancia os encoders para os motores
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); //motor A
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); //motor B

  //instancia os controladores PID de cada motor
  pidInit(&A_MotorController, MIN_PWM, MAX_PWM, KP_A, KI_A, KD_A); //motor A
  pidInit(&B_MotorController, MIN_PWM, MAX_PWM, KP_B, KI_B, KD_B); //motor B

  //mantém os pinos de enable ligados (condição necessária para o driver)
  HAL_GPIO_WritePin(GPIOF, ENABLE_A_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOF, ENABLE_B_Pin, GPIO_PIN_SET);

  //motores inicialmente parados (questão de segurança)
  A_MotorStop();
  B_MotorStop();
  //TIM5->CC1: PWM PARA MOTOR A
  //TIM5->CCR2 PWM PARA MOTOR B
  TIM5->CCR1 = 0; TIM5->CCR2 = 0; //dutyCycle inicialmente em 0%


  //inicializa os buffers com 0
  for(i= 0; i< STEP_BUFFER_SIZE; i++){
	  A_Buffer[i] = 0;
	  B_Buffer[i] = 0;
  }
  i=0;

  //mantém os registradores dos encoders em um valor diferente de zero
  //para não lidar com overflow
  TIM3->CNT = TIMER_INIT_VALUE;
  TIM4->CNT = TIMER_INIT_VALUE;

  //inicia a interrupção, é chamada a cada 10 ms. Configurável no .ioc
  //chama a função HAL_TIM_PeriodElapsedCallback
  HAL_TIM_Base_Start_IT(&htim2);
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

		//TODO: integração com o alto nível! Atualizar o setPoint em tempo real

//		currentTick = HAL_GetTick(); //para contar o tempo decorrido (gerar gráfico  com o script python)

		//obtém quanto cada encoder andou
		A_Step = TIM3->CNT - TIMER_INIT_VALUE;
		B_Step = (TIM4->CNT - TIMER_INIT_VALUE) * -1; //o motor está de cabeça para baixo no andador

		TIM3->CNT = TIMER_INIT_VALUE;
		TIM4->CNT = TIMER_INIT_VALUE;

//		pastTime += (float)(currentTick - lastTick) / 1000; //para gerar gráficos  com o script python

//		/*Testes de velocidade*/

//		if(pastTime > 45){
//			A_SetPoint = 2000;
//			B_SetPoint = 2000;
//		}
//		else if(pastTime > 35){
//			A_SetPoint = -2000;
//			B_SetPoint = -2000;
//		}
//		else if(pastTime > 25){
//			A_SetPoint = 0;
//			B_SetPoint = 0;
//		}
//		else if(pastTime > 15){
//			A_SetPoint = -2000;
//			B_SetPoint = -2000;
//		}
//		else if(pastTime > 5){
//			A_SetPoint = 2000;
//			B_SetPoint = 2000;
//		}


		/*diferentes tipos de movimento para motor A */
		//horário
		if(A_SetPoint > 0) A_MotorClockWise();
		//anti-horário
		else if(A_SetPoint < 0) A_MotorCounterClockWise();
		//parado
		else {
			A_MotorStop();
			TIM5->CCR1 = 0; //0% dutyCycle
			A_Duty = 0; //evita enviesar a próxima velocidade
		}
		//análogo para o motor B
		if(B_SetPoint > 0) B_MotorClockWise();
		else if(B_SetPoint < 0) B_MotorCounterClockWise();
		else{
			B_MotorStop();
			TIM5->CCR2 = 0;
			B_Duty = 0;
		}

		if(i >= 10) i=0; //cria estrutura de fila no buffer

		//atualiza o buffer de velocidade (atua como filtro média móvel)
		A_Buffer[i] = A_Step;
		B_Buffer[i] = B_Step;
		i++;

		//obtém a velocidade média de cada buffer
		for(j=0; j< STEP_BUFFER_SIZE; j++){
			A_Sum += A_Buffer[j];
			B_Sum += B_Buffer[j];
		}

		A_Sum /= STEP_BUFFER_SIZE;
		B_Sum /= STEP_BUFFER_SIZE;


		// velocidade atual em pulsos/s
		A_Speed = A_Sum / DELTA_T;
		B_Speed = B_Sum / DELTA_T;

		//obtém variação no PWM em ponto flutuante
		A_Duty += computePwmValue(A_SetPoint, A_Speed, &A_MotorController);
		B_Duty += computePwmValue(B_SetPoint, B_Speed, &B_MotorController);

		//considera os limites do CCR para os motores
		if(A_Duty > CCR_LIMIT) A_Duty = CCR_LIMIT;
		else if(A_Duty < 0) A_Duty = 0;

		if(B_Duty > CCR_LIMIT) B_Duty = CCR_LIMIT;
		else if(B_Duty < 0) B_Duty = 0;

		//CCR em CCR_LIMIT -> DUTY CYCLE 100%
		//CCR em 0 -> DUTY CYCLE 0%
		//configurável no .ioc

		//atualiza pwm enviado ao driver
		TIM5->CCR1 = A_Duty;
		TIM5->CCR2 = B_Duty;

		/* debug printing */
//		printf("stepA: %d stepB: %d\r\n", A_Step, B_Step); //encoder step
//		printf("PWM: %ld\r\n", TIM1->CCR4); //pwm register
//		printf("%f %f %f\n\r", A_Speed, B_Speed, pastTime); //graphic for both motors
//		printf("%ld\r\n", currentTick - lastTick); //time between interrupts

		//atualizações para a próxima iteração
		A_Sum = 0;
		B_Sum = 0;
		//lastTick = currentTick; //para gerar gráficos com o script python
	}
}

/**
 * AIN1: ON
 * AIN2: OFF
 * */
void A_MotorClockWise(){
	HAL_GPIO_WritePin(GPIOF, A_IN_1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOF, A_IN_2_Pin, GPIO_PIN_RESET);
}

/**
 * AIN1: OFF
 * AIN2: ON
 * */
void A_MotorCounterClockWise(){
	HAL_GPIO_WritePin(GPIOF, A_IN_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF, A_IN_2_Pin, GPIO_PIN_SET);
}

/**
 * AIN1: OFF
 * AIN2: OFF
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


void B_MotorClockWise(){
	HAL_GPIO_WritePin(GPIOF, B_IN_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF, B_IN_2_Pin, GPIO_PIN_SET);
}


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
