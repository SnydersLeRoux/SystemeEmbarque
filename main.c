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
#include "lcd16x2_i2c.h"
#include "stdio.h"
#include <string.h>
#include <stdlib.h>
#include <time.h>

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
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

int commandeIR = 0;
int choixMode = 1; //par défaut mode auto
int valIR = 0;
int boucle1 = 0;
int rot = 0;
int angle = 0;
int nbrMesureParAngl = 6;    // nbr de mesure par angle +1 car il faut enlever la prmière mesure de chaque angle car = a la mesure de l'angle d'avant
int pos = 0;
int theta = 0;
int nbrPts = 0;

uint8_t mesureAngle[4] = "   \n";    // attention longueur tableu
uint8_t mesureDist [4] = "    ";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

void delay (uint16_t time);
void delayS (uint16_t us);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void HCSR04_Read (void);
void stepper_set_rpm (int rpm);
void stepper_half_drive (int step);
void stepper_step_angle (float angle, int direction, int rpm);
uint32_t receive_data (void);
void convert_code (uint32_t code);
void choixNbrPts(int pos);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/******************************  UltraSonic sensor  ***********************/

uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;  // is the first value captured ?
uint8_t Distance  = 0;

#define TRIG_PIN GPIO_PIN_9     // ICI DIFFERENT QUE DANS VIDEO !!!!!!!!!!!!!!!!!!
#define TRIG_PORT GPIOA

/******************************  Stepper motor   ***********************/

#define stepsperrev 4096

/* use like delay */

/*************************** MODULE IR  ******************************/

uint8_t count = 0;
uint32_t data;

/*************************** SLEEP MODE  ******************************/

void SleepMode_ON();
void SleepMode_OFF();


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
  MX_I2C3_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim3);

  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);

  lcd16x2_i2c_init(&hi2c3);
  lcd16x2_i2c_printf("INITIALISING>>>>");
  HAL_Delay(2000);
  lcd16x2_i2c_clear();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  /******************************  My program  ***********************/
	  lcd16x2_i2c_clear();
	  lcd16x2_i2c_setCursor(0, 0);

	  lcd16x2_i2c_printf(" Mode auto");
	  lcd16x2_i2c_setCursor(1, 0);
	  lcd16x2_i2c_printf(" Mode manuel");
	  lcd16x2_i2c_setCursor(0, 0);
	  lcd16x2_i2c_printf(">");
	  choixMode = 1;           // si je fait d'abord du manu et puis que je passe en auto il faut cette ligne
	  commandeIR = 0;


	  while(commandeIR != 3){

		  while (HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_1)){}    // par défaut mode auto donc choixMode = 1
		  data = receive_data ();
		  convert_code (data);

		  if(choixMode == 1 ){
			  lcd16x2_i2c_setCursor(1, 0);
		      lcd16x2_i2c_printf(" ");
			  lcd16x2_i2c_setCursor(0, 0);
		      lcd16x2_i2c_printf(">");
		  }

		  if(choixMode == 2){
			  lcd16x2_i2c_setCursor(0, 0);
		      lcd16x2_i2c_printf(" ");
			  lcd16x2_i2c_setCursor(1, 0);
		      lcd16x2_i2c_printf(">");
		  }

	  }
	  commandeIR = 0;

	  /*********  Mode auto  *********/                                      // demander à l'utilisateur le nbr de pts qu'il veut

	  if(choixMode == 1){

		  /***** choix nbr de pts ***/

		  lcd16x2_i2c_clear();
		  lcd16x2_i2c_setCursor(0, 0);
		  lcd16x2_i2c_printf("Nbr de points :");
		  lcd16x2_i2c_setCursor(1, 0);
	      lcd16x2_i2c_printf("36 points ");

		  while(commandeIR != 3){

			  commandeIR = 0;     // si on touche une autre touche on incrémente qd même le nbrPts mais pas avec cette ligne en plus
			  while (HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_1)){}    // par défaut mode auto donc choixMode = 1
			  data = receive_data ();
			  convert_code (data);

			  if(commandeIR == 2)
				  pos--;

			  if(commandeIR == 4)
				  pos++;

			  if(pos<0)      // permet de rester dans l'intervalle du switch
				  pos = 0;
			  if(pos>5)
				  pos = 5;

			  choixNbrPts(pos);

		  }


          /****** mesures ****/

		  for(int i = 0; i < nbrPts+1 ; i++){  // nbrPts+1 mesures car pt 0° en plus


			  for(int j = 0; j<nbrMesureParAngl; j++){    //20 mesures par angle

			      lcd16x2_i2c_clear();
			      lcd16x2_i2c_printf("Dist = ");
			      HCSR04_Read();
			      lcd16x2_i2c_sendData((Distance/100) + 48);   // 100th pos
			      mesureDist[0] = (Distance/100) + 48;
			      lcd16x2_i2c_sendData(((Distance/10)%10) +48);  // 10th pos
			      mesureDist[1] = ((Distance/10)%10) +48;
			      lcd16x2_i2c_sendData((Distance%10)+48);  // 1st pos
			      mesureDist[2] = (Distance%10)+48;
			      lcd16x2_i2c_printf(" cm");
			      mesureAngle[0] = (angle/100) + 48;  // + 48 pour avoir le bon caractère uint8_t
			      mesureAngle[1] = ((angle/10)%10) +48;
		    	  mesureAngle[2] = (angle%10)+48;
			      HAL_UART_Transmit(&huart2, mesureDist, sizeof(mesureDist), 100);
			      HAL_UART_Transmit(&huart2, mesureAngle, sizeof(mesureAngle), 100);
			      HAL_Delay(100); // temps entre deux mesures

			  }

			  stepper_step_angle(theta, 0, 10);
			  angle = angle + theta;
			  HAL_Delay(500);

		  }

		  stepper_step_angle(180+theta, 1, 10);   //  repositionnement au pt 0°
		  angle = 360;

		  for(int i = 0; i < nbrPts-1; i++){  // nbrPts-1 mesures car pt 180° déja pris avant

			  stepper_step_angle(theta, 1, 10);
			  angle = angle - theta;
			  HAL_Delay(500);

			  for(int j = 0; j<nbrMesureParAngl; j++){

			      lcd16x2_i2c_clear();
			      lcd16x2_i2c_printf("Dist = ");
			      HCSR04_Read();
			      lcd16x2_i2c_sendData((Distance/100) + 48);   // 100th pos
			      mesureDist[0] = (Distance/100) + 48;
			      lcd16x2_i2c_sendData(((Distance/10)%10) +48);  // 10th pos
			      mesureDist[1] = ((Distance/10)%10) +48;
			      lcd16x2_i2c_sendData((Distance%10)+48);  // 1st pos
			      mesureDist[2] = (Distance%10)+48;
			      lcd16x2_i2c_printf(" cm");
			      mesureAngle[0] = (angle/100) + 48;  // + 48 pour avoir le bon caractère uint8_t
			      mesureAngle[1] = ((angle/10)%10) +48;
		    	  mesureAngle[2] = (angle%10)+48;
			      HAL_UART_Transmit(&huart2, mesureDist, sizeof(mesureDist), 100);
			      HAL_UART_Transmit(&huart2, mesureAngle, sizeof(mesureAngle), 100);
			      HAL_Delay(100);
			  }

		  }

		  stepper_step_angle(180-theta, 0, 10);      //repositionnemnt angle 0°
		  angle = 0;  // reset la valeur de angle
		  lcd16x2_i2c_clear();
		  lcd16x2_i2c_printf("Mesures terminee");
		  HAL_Delay(3000);

	  }

	  /*********  Mode manuel  *********/                                                           // choix angle de rotation à encoder (pas fait)


	  else{


		  while(commandeIR != 1){            // éreindre mode auto avec btn power

			  lcd16x2_i2c_clear();
              lcd16x2_i2c_printf("Positionnement");
              lcd16x2_i2c_setCursor(1, 0);
              lcd16x2_i2c_printf("Sortir > Power");

			  while (HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_1)){}    // wait for the pin to go low
			  data = receive_data ();
			  convert_code (data);

			  if(commandeIR == 2){
				  stepper_step_angle(10, 0, 10);
			  }

			  if(commandeIR == 4){
				  stepper_step_angle(10, 1, 10);
			  }

			  if(commandeIR == 3) // démarre mesure si btn pause/play enfoncé
				  boucle1 = 1;

			  while(boucle1 == 1){        // sortir de la boucle avec B1
			     lcd16x2_i2c_clear();
			     lcd16x2_i2c_printf("Dist = ");
			     HCSR04_Read();
			     lcd16x2_i2c_sendData((Distance/100) + 48);   // 100th pos
			     lcd16x2_i2c_sendData(((Distance/10)%10) +48);  // 10th pos
			     lcd16x2_i2c_sendData((Distance%10)+48);  // 1st pos
			     lcd16x2_i2c_printf(" cm");
			     HAL_Delay(500);
			     lcd16x2_i2c_setCursor(0, 0);

			  }

		  }

	  }

	  // --- Entrée en sleep mode
	 SleepMode_ON();

	 // --- Sortie du sleep mode
	 SleepMode_OFF();

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C3;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
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
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x10909CEC;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 79;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
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
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim3.Init.Prescaler = 79;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|IN4_Pin|IN3_Pin|Trig_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IR_Pin */
  GPIO_InitStruct.Pin = IR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin IN4_Pin IN3_Pin Trig_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|IN4_Pin|IN3_Pin|Trig_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : IN1_Pin */
  GPIO_InitStruct.Pin = IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IN1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IN2_Pin */
  GPIO_InitStruct.Pin = IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IN2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_Pin */
  GPIO_InitStruct.Pin = BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
/******************************  My function  ***********************/


void choixNbrPts(int pos)
{
	 switch ( pos )
	 {

		case 0:
		    nbrPts = 36/2;    // divisé par 2 car pts dans un sens puis dans l'autre
		    theta = 10;
			lcd16x2_i2c_setCursor(1, 0);
		    lcd16x2_i2c_printf("36 points ");
	        break;
		case 1:
		    nbrPts = 72/2;
			theta = 5;
			lcd16x2_i2c_setCursor(1, 0);
			lcd16x2_i2c_printf("72 points ");
			break;
		case 2:
			nbrPts = 90/2;
		    theta = 4;
			lcd16x2_i2c_setCursor(1, 0);
			lcd16x2_i2c_printf("90 points ");
			break;
		case 3:
		    nbrPts = 120/2;    // divisé par 2 car pts dans un sens puis dans l'autre
			theta = 3;
			lcd16x2_i2c_setCursor(1, 0);
			lcd16x2_i2c_printf("120 points");
			break;
		case 4:
			nbrPts = 180/2;
			theta = 2;
			lcd16x2_i2c_setCursor(1, 0);
			lcd16x2_i2c_printf("180 points");
		    break;
		case 5:
			nbrPts = 360/2;
			theta = 1;
			lcd16x2_i2c_setCursor(1, 0);
			lcd16x2_i2c_printf("360 points");
			break;
		default :
		    nbrPts = 0;
			theta = 0;
			lcd16x2_i2c_setCursor(1, 0);
		    lcd16x2_i2c_printf("0 points  ");
		}

}


/******************************  us function  ***********************/
/* houss
void delay (uint16_t time)
{
	// change your code here for the delay in microseconds
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while ((__HAL_TIM_GET_COUNTER(&htim1))<time);
}
*/
void delayS (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	while (__HAL_TIM_GET_COUNTER(&htim3) < us);
}

// *Fonction de temporisation en microsecondes*
/*void Delay_Us(uint32_t uSec)
{
	__HAL_TIM_CLEAR_FLAG(&htim4,TIM_FLAG_UPDATE);					// similaire à usTIM->SR=0;   			 	=> clear le flag d'overflow
	__HAL_TIM_SET_PRESCALER(&htim4,80-1);							// idem usTIM->PSC=80-1;	   				=> prescaler pour avoir une fréquence d'horloge de 80MHz/80 = 1MHz
	__HAL_TIM_SET_AUTORELOAD(&htim4,uSec-1);  						// idem usTIM->ARR=usec-1;   			    => le compteur va de 0 à usec-1
	__HAL_TIM_ENABLE(&htim4);   									// idem usTIM->CR1 |= TIM_CR1_CEN;  	    => début du comptage
	while(!(__HAL_TIM_GET_FLAG(&htim4,TIM_FLAG_UPDATE)));			// idem while(!(usTIM->SR & TIM_SR_UIF));   => boucle tant que le flag n'est pas enclenché
}*/

/******************************  UltraSonic sensor function  ***********************/

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
	{
		if (Is_First_Captured==0) // if the first value is not captured
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured==1)   // if the first is already captured
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2-IC_Val1;
			}

			else if (IC_Val1 > IC_Val2)
			{
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}

			Distance = Difference * .034/2;
			Is_First_Captured = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
		}
	}
}

void HCSR04_Read (void)
{
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay(10);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
}


/******************************  Stepper motor function  ***********************/

void stepper_set_rpm (int rpm)  // Set rpm--> max 13, min 1,,,  went to 14 rev/min
{
	delayS(60000000/stepsperrev/rpm);
}

void stepper_half_drive (int step)
{
	switch (step){
		case 0:
			  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);   // IN1
			  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);   // IN2
			  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);   // IN3
			  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);   // IN4
			  break;

		case 1:
			  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);   // IN1
			  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);   // IN2
			  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);   // IN3
			  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);   // IN4
			  break;

		case 2:
			  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);   // IN1
			  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);   // IN2
			  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);   // IN3
			  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);   // IN4
			  break;

		case 3:
			  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);   // IN1
			  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);   // IN2
			  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);   // IN3
			  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);   // IN4
			  break;

		case 4:
			  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);   // IN1
			  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);   // IN2
			  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);   // IN3
			  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);   // IN4
			  break;

		case 5:
			  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);   // IN1
			  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);   // IN2
			  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);   // IN3
			  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);   // IN4
			  break;

		case 6:
			  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);   // IN1
			  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);   // IN2
			  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);   // IN3
			  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);   // IN4
			  break;

		case 7:
			  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);   // IN1
			  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);   // IN2
			  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);   // IN3
			  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);   // IN4
			  break;

		}
}


void stepper_step_angle (float angle, int direction, int rpm)
{
	float anglepersequence = 0.703125;  // 360 = 512 sequences
	int numberofsequences = (int) (angle/anglepersequence);

	for (int seq=0; seq<numberofsequences; seq++)
	{
		if (direction == 0)  // for clockwise
		{
			for (int step=7; step>=0; step--)
			{
				stepper_half_drive(step);
				stepper_set_rpm(rpm);
			}

		}

		else if (direction == 1)  // for anti-clockwise
		{
			for (int step=0; step<8; step++)
			{
				stepper_half_drive(step);
				stepper_set_rpm(rpm);
			}
		}
	}
}

/*************************** MODULE IR FUNCTIONS ******************************/

uint32_t receive_data (void)
{
	uint32_t code=0;

		  /* The START Sequence begin here
	   * there will be a pulse of 9ms LOW and
	   * than 4.5 ms space (HIGH)
	   */
	  while (!(HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_1)));  // wait for the pin to go high.. 9ms LOW

	  while ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_1)));  // wait for the pin to go low.. 4.5ms HIGH

	  /* START of FRAME ends here*/

	  /* DATA Reception
	   * We are only going to check the SPACE after 562.5us pulse
	   * if the space is 562.5us, the bit indicates '0'
	   * if the space is around 1.6ms, the bit is '1'
	   */

	  for (int i=0; i<32; i++)
	  {
		  count=0;

		  while (!(HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_1))); // wait for pin to go high.. this is 562.5us LOW

		  while ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_1)))  // count the space length while the pin is high
		  {
			  count++;
			  delay(100);  // 100us delay
		  }

		  if (count > 12) // if the space is more than 1.2 ms
		  {
			  code |= (1UL << (31-i));   // write 1
		  }

		  else code &= ~(1UL << (31-i));  // write 0
	  }

		return code;
}

void convert_code (uint32_t code)
{
		switch (code)
		{
			case (0xFF30CF): // "1"
				break;

			case (0xFF18E7): // "2"
				break;

			case (0xFF7A85): // "3"
				break;

			case (0xFF10EF): // "4"
				break;

			case (0xFF38C7): // "5"
				break;

			case (0xFF5AA5): // "6"
				break;

			case (0xFF42BD): // "7"
				break;

			case (0xFF4AB5): // "8"

				break;

			case (0xFF52AD): // "9"
				break;

			case (0xFF6897): // "0"
				break;

			case (0xFFA25D): // "POWER"
		        commandeIR = 1;
				break;

			case (0xFF22DD): // "FAST BACK"
				commandeIR = 2;
				break;

			case (0xFF02FD): // "PAUSE"
			    commandeIR = 3;
				break;

			case (0xFFC23D): // "FAST FORWARD"
				commandeIR = 4;
				break;

			case (0xFFE01F): // "DOWN"
				choixMode = 2;
				break;

			case (0xFF906F): // "UP"
				choixMode = 1;
				break;

			case (0xFFA857): // "VOL-"
				break;

			case (0xFF9867): // "EQ"
				break;

			case (0xFF629D): // "VOL+"
				break;

			case (0xFFE21D): // "FUNC/STOP"
				break;

			case (0xFFB04F): // "ST/REPT"
				break;

			default :
				break;
		}

		HAL_Delay (200);     //    delay nécessaire sinon deconne
}

/*************************** INTERRUPTION FUNCTION ******************************/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	// ne pas utiliser le module IR avec interruption car déconne

	if(GPIO_Pin == BTN_Pin){
		if(!HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin)) // mettre GPIO pull-up sinon ça marche pas
			boucle1 = 0;

	}

	if(GPIO_Pin == B1_Pin){

	    // pas de lcd_print car bug (ne pas rester trop longtemps dans une interruption)
     	//sprintf(sleepModeMessage,"Desactivation du Sleep Mode...\r\n") ;
		//HAL_UART_Transmit(&huart2, (uint8_t *)sleepModeMessage, strlen (sleepModeMessage), HAL_MAX_DELAY);
		HAL_ResumeTick();																					 // réactiver le systick timer
		HAL_PWR_DisableSleepOnExit ();
	}

}

/*************************** SLEEP MODE FUNCTIONS ******************************/

// *Fonction d'activation du sleep mode*
void SleepMode_ON()
{

		  // 1. message d'activation du sleep mode

		  //sprintf(sleepModeMessage,"Activation du Sleep Mode...\r\n");      						// message à afficher sur la console
		 // HAL_UART_Transmit(&huart2, (uint8_t *)sleepModeMessage, strlen(sleepModeMessage), HAL_MAX_DELAY);			// transmettre le message au port adéquat
	      lcd16x2_i2c_clear();
	      lcd16x2_i2c_printf("Sleep mode");
	      lcd16x2_i2c_setCursor(1, 0);
	      lcd16x2_i2c_printf("activation...");
		  HAL_Delay(2000);

		  // 2. allumage de la led pendant 5 secondes => sleep mode en cours...
		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);														// allumer la led pour indiquer que la carte n'est pas encore en sleep mode
	      lcd16x2_i2c_clear();
	      lcd16x2_i2c_printf("ZZZzzzzzzz...");
		  HAL_Delay(3000);																					// pendant 5 secondes

		  // 3. suspende le systick timer sinon on sort du sleep mode au bout de 1ms
		  HAL_SuspendTick();


		  // 4. éteindre de la led pour indiquer que le sleep mode est actif
		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);														// éteindre la led

		  // 5. activer la sortie de sleep mode par interruption externe
		  HAL_PWR_EnableSleepOnExit();

		  // 6. entrer dans le sleep mode  => sortie par interruption
		  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);									// entrer en sleep mode

}

// *Fonction qui indique la sortie du sleep mode*
void SleepMode_OFF()
{
			// message de sortie du sleep mode
            lcd16x2_i2c_clear();
            lcd16x2_i2c_printf("Sleep mode");
            lcd16x2_i2c_setCursor(1, 0);
            lcd16x2_i2c_printf("desactive");
            HAL_Delay(2000);
			//sprintf(sleepModeMessage,"Sleep Mode desactive !\r\n");
			//HAL_UART_Transmit(&huart2, (uint8_t *)sleepModeMessage, strlen(sleepModeMessage), 100);

			// clignoter la led 5 fois pour indiquer la sortie du sleep mode
			for(int i=0;i<5;i++)
		 	  {
		 		  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		 		  HAL_Delay(200);
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
