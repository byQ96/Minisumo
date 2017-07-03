/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint16_t KTIR[2];
int count = 0;
volatile int status = 0;
int kolory[10][3] = {
					{50, 50 ,50},
					{150, 50 ,50},
					{250, 50 ,50},
					{50, 150 ,50},
					{50, 250 ,50},
					{50, 50 ,150},
					{50, 50 ,250},
					{150, 150 ,150},
					{250, 250 ,0},
					{0, 250 ,250},
					};
int tactic = 0;
int max_tactic = 10;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); //Blue
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); //Red
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); //Green

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);


  int direction = 0;
  int pop_direction = 0;
  int zmiana = 0;
  int q1 = 1;
  int q2 = 1;
  // -- Enables ADC DMA request

  //HAL_ADC_Start_DMA(&hadc1, (uint32_t*)KTIR, 2);
  //HAL_ADC_Start(&hadc1);
  //HAL_ADC_Start_DMA(&hadc1, KTIR, 2);
  //HAL_ADC_Start(&hadc1);
  //HAL_ADC_Start(&hadc1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	  /*HAL_ADC_Start(&hadc1);
	  while(HAL_ADC_PollForConversion(&hadc1,0) != HAL_OK);

	  while(HAL_ADC_PollForConversion(&hadc1,0) != HAL_OK);

	  HAL_ADC_Stop(&hadc1);
	  TIM1->CCR2 = KTIR[1];*/

	  //KTIR[0] = HAL_ADC_GetValue(&hadc1);
	  //KTIR[0]= HAL_ADC_GetValue(&hadc1);
	 // TIM1->CCR1 = KTIR[0];
	  /*
	  if(HAL_GPIO_ReadPin(START_BUTTON_GPIO_Port, START_BUTTON_Pin) == GPIO_PIN_SET){
		  HAL_Delay(50);
		  if(HAL_GPIO_ReadPin(START_BUTTON_GPIO_Port, START_BUTTON_Pin) == GPIO_PIN_SET){
			  if(status == 0) status = 1; else status = 0;

		  HAL_GPIO_WritePin(LED_FAIL_GPIO_Port, LED_FAIL_Pin, GPIO_PIN_SET);
		  HAL_Delay(1000);
		  HAL_GPIO_WritePin(LED_FAIL_GPIO_Port, LED_FAIL_Pin, GPIO_PIN_RESET);
		  HAL_Delay(1000);
		  HAL_GPIO_WritePin(LED_FAIL_GPIO_Port, LED_FAIL_Pin, GPIO_PIN_SET);
		  if(HAL_GPIO_ReadPin(START_BUTTON_GPIO_Port, START_BUTTON_Pin) == GPIO_PIN_SET){
			   status = 2;
		  }else{
			  HAL_Delay(1000);
			  HAL_GPIO_WritePin(LED_FAIL_GPIO_Port, LED_FAIL_Pin, GPIO_PIN_RESET);
			  HAL_Delay(1000);
			  HAL_GPIO_WritePin(LED_FAIL_GPIO_Port, LED_FAIL_Pin, GPIO_PIN_SET);
			  HAL_Delay(1000);

		  }
		  }
	  }
if(status == 2){
	 TIM1->CCR3 = 60;
	TIM1->CCR2 = 60;
	TIM1->CCR1 = 60;

}*/
	  if(status == 1){
		  status = 2;
		  for(int i=0;i<5;i++)
		  {
		  HAL_GPIO_WritePin(LED_FAIL_GPIO_Port, LED_FAIL_Pin, GPIO_PIN_SET);
		  HAL_Delay(500);
		  HAL_GPIO_WritePin(LED_FAIL_GPIO_Port, LED_FAIL_Pin, GPIO_PIN_RESET);
		  HAL_Delay(500);
		  }
	  }
if(status == 2){
	TIM4->CCR1 = 0;
		  		TIM4->CCR4 = 0;
		  		zmiana = 0;
	  //direction = 4;
	  TIM1->CCR3 = 0;
	  TIM1->CCR2 = 0;
	  TIM1->CCR1 = 0;
	  HAL_GPIO_WritePin(LED_FAIL_GPIO_Port, LED_FAIL_Pin, GPIO_PIN_RESET);

	  if(HAL_GPIO_ReadPin(SHARP1_GPIO_Port, SHARP1_Pin) == GPIO_PIN_RESET){
		  direction = 1;
		  zmiana = 1;
		  TIM1->CCR3 = 200;

	  }
	  if(HAL_GPIO_ReadPin(SHARP2_GPIO_Port, SHARP2_Pin) == GPIO_PIN_RESET){
	  		  if(direction == 1) direction = 2; else direction = 3;
	  		zmiana = 1;
		  	  TIM1->CCR2 = 200;
	  	  }

	  if(HAL_GPIO_ReadPin(SHARP3_GPIO_Port, SHARP3_Pin) == GPIO_PIN_RESET){
		  if(direction == 3) direction = 4; else direction = 5;
		  zmiana = 1;
		  TIM1->CCR1 = 200;
	  	  }

	  if(HAL_GPIO_ReadPin(SHARP4_GPIO_Port, SHARP4_Pin) == GPIO_PIN_RESET){
		  if(direction == 5) direction = 6; else direction = 7;

		  zmiana = 1;
		  HAL_GPIO_WritePin(LED_FAIL_GPIO_Port, LED_FAIL_Pin, GPIO_PIN_SET);


	  	  }

	  if(zmiana == 0){
		  int tmp = direction;
		  if(direction < 4){
			  if(pop_direction < direction) direction++;
			  else if(pop_direction > direction) direction --;
			  else {}
		  } else {
			  if(direction > 4){
			  			  if(pop_direction < direction) direction++;
			  			  else if(pop_direction > direction) direction --;
			  			  else {}
			  		  }
		  }
		  if(direction > 7) direction = 7;
		  if(direction < 1) direction = 0;
		  pop_direction = tmp;
	  }

	  switch(direction){
	  	  case 0:{
	  		HAL_GPIO_WritePin(GPIOB, DIR2A_Pin,  GPIO_PIN_RESET);
	  			  		HAL_GPIO_WritePin(GPIOB, DIR2B_Pin,  GPIO_PIN_SET);
	  			  		TIM4->CCR1 = 200;

	  			  		HAL_GPIO_WritePin(GPIOB, DIRA_Pin,  GPIO_PIN_RESET);
	  			  		HAL_GPIO_WritePin(GPIOB, DIRB_Pin,  GPIO_PIN_SET);
	  			  		TIM4->CCR4 = 200;

	  		  break;
	  	  }
	  	  case 1:{
	  		HAL_GPIO_WritePin(GPIOB, DIR2A_Pin,  GPIO_PIN_RESET);
	  		HAL_GPIO_WritePin(GPIOB, DIR2B_Pin,  GPIO_PIN_SET);
	  		TIM4->CCR1 = 250;

	  		HAL_GPIO_WritePin(GPIOB, DIRA_Pin,  GPIO_PIN_RESET);
	  		HAL_GPIO_WritePin(GPIOB, DIRB_Pin,  GPIO_PIN_SET);
	  		TIM4->CCR4 = 250;

	  		  break;
	  	  }
	  	  case 2:{
	  		HAL_GPIO_WritePin(GPIOB, DIR2A_Pin,  GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, DIR2B_Pin,  GPIO_PIN_SET);
			TIM4->CCR1 = 120;

			HAL_GPIO_WritePin(GPIOB, DIRA_Pin,  GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, DIRB_Pin,  GPIO_PIN_SET);
			TIM4->CCR4 = 250;

	  		  break;
	  	  }
	  	  case 3:{
			HAL_GPIO_WritePin(GPIOB, DIR2A_Pin,  GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, DIR2B_Pin,  GPIO_PIN_RESET);
			TIM4->CCR1 = 150;

			HAL_GPIO_WritePin(GPIOB, DIRA_Pin,  GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, DIRB_Pin,  GPIO_PIN_SET);
			TIM4->CCR4 = 250;

			  break;
		  }
	  	case 4:{
			HAL_GPIO_WritePin(GPIOB, DIR2A_Pin,  GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, DIR2B_Pin,  GPIO_PIN_RESET);
			TIM4->CCR1 = q1*250;

			HAL_GPIO_WritePin(GPIOB, DIRA_Pin,  GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, DIRB_Pin,  GPIO_PIN_SET);
			TIM4->CCR4 = q2*250;

			  break;
		  }
	  	case 5:{
			HAL_GPIO_WritePin(GPIOB, DIR2A_Pin,  GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, DIR2B_Pin,  GPIO_PIN_RESET);
			TIM4->CCR1 = 250;

			HAL_GPIO_WritePin(GPIOB, DIRA_Pin,  GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, DIRB_Pin,  GPIO_PIN_SET);
			TIM4->CCR4 = 150;

			  break;
	  	}
	  	case 6:{
			HAL_GPIO_WritePin(GPIOB, DIR2A_Pin,  GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, DIR2B_Pin,  GPIO_PIN_RESET);
			TIM4->CCR1 = 250;

			HAL_GPIO_WritePin(GPIOB, DIRA_Pin,  GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, DIRB_Pin,  GPIO_PIN_RESET);
			TIM4->CCR4 = 120;

			  break;
	   }
	  	case 7:{
			HAL_GPIO_WritePin(GPIOB, DIR2A_Pin,  GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, DIR2B_Pin,  GPIO_PIN_RESET);
			TIM4->CCR1 = 250;

			HAL_GPIO_WritePin(GPIOB, DIRA_Pin,  GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, DIRB_Pin,  GPIO_PIN_RESET);
			TIM4->CCR4 = 250;

			  break;
	   }
	  }
	  HAL_Delay(20);
  }





	  /*HAL_Delay(15);
	  if(HAL_GPIO_ReadPin(START_BUTTON_GPIO_Port, START_BUTTON_Pin) == GPIO_PIN_SET){
		  HAL_Delay(15);
		  if(HAL_GPIO_ReadPin(START_BUTTON_GPIO_Port, START_BUTTON_Pin) == GPIO_PIN_SET){
		   HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
		   }else{
		   HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
		   }
	  }
		*/

	  /*

	  HAL_GPIO_WritePin(GPIOA, LED_FAIL_Pin,  GPIO_PIN_SET);
	  	  HAL_Delay(100);
	  	  HAL_GPIO_WritePin(GPIOA, LED_FAIL_Pin,  GPIO_PIN_RESET);
	  	HAL_Delay(100);
	  	int i;
	  	for(i=0;i < 255; i++){
	  		TIM1->CCR3 = i;
	  		HAL_Delay(15);
	  	}

	  	HAL_GPIO_WritePin(GPIOB, DIR2A_Pin,  GPIO_PIN_SET);
	  	HAL_GPIO_WritePin(GPIOB, DIR2B_Pin,  GPIO_PIN_RESET);
	  	HAL_GPIO_WritePin(GPIOB, DIRA_Pin,  GPIO_PIN_SET);
	  	HAL_GPIO_WritePin(GPIOB, DIRB_Pin,  GPIO_PIN_RESET);
	  	for(i=0;i < 200; i++){
	  		  		TIM4->CCR1 = i;
	  		  		TIM4->CCR4 = i;
	  		  		HAL_Delay(15);
	  		  	}
	  	for(i=200;i > 0; i--){
	  		  		  		TIM4->CCR1 = i;
	  		  		  	TIM4->CCR4 = i;
	  		  		  		HAL_Delay(15);
	  		  		  	}
	  	HAL_Delay(2000);
	  	HAL_GPIO_WritePin(GPIOB, DIR2A_Pin,  GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(GPIOB, DIR2B_Pin,  GPIO_PIN_SET);
	    HAL_GPIO_WritePin(GPIOB, DIRA_Pin,  GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(GPIOB, DIRB_Pin,  GPIO_PIN_SET);

	    TIM4->CCR1 = 60;
	    TIM4->CCR4 = 60;
	    HAL_Delay(2000);
*/
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_InjectionConfTypeDef sConfigInjected;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Injected Channel 
    */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_0;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 2;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.AutoInjectedConv = ENABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Injected Channel 
    */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
  sConfigInjected.InjectedRank = 2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1024;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

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

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 255;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim4);

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_FAIL_GPIO_Port, LED_FAIL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIR2A_Pin|DIR2B_Pin|DIRA_Pin|DIRB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SHARP1_Pin SHARP2_Pin */
  GPIO_InitStruct.Pin = SHARP1_Pin|SHARP2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SHARP3_Pin SHARP4_Pin STOP_MODULE_Pin START_MODULE_Pin 
                           STOP_BUTTON_Pin */
  GPIO_InitStruct.Pin = SHARP3_Pin|SHARP4_Pin|STOP_MODULE_Pin|START_MODULE_Pin 
                          |STOP_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : TACTIC_BUTTON_Pin START_BUTTON_Pin */
  GPIO_InitStruct.Pin = TACTIC_BUTTON_Pin|START_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_FAIL_Pin */
  GPIO_InitStruct.Pin = LED_FAIL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_FAIL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR2A_Pin DIR2B_Pin DIRA_Pin DIRB_Pin */
  GPIO_InitStruct.Pin = DIR2A_Pin|DIR2B_Pin|DIRA_Pin|DIRB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (HAL_GPIO_ReadPin(START_BUTTON_GPIO_Port, START_BUTTON_Pin))
	{
		status=1;
	}
	if (HAL_GPIO_ReadPin(TACTIC_BUTTON_GPIO_Port, TACTIC_BUTTON_Pin))
		{

		if(tactic == max_tactic) tactic=0; else tactic++;
				TIM1->CCR3 = kolory[tactic][0];
				TIM1->CCR2 = kolory[tactic][1];
				TIM1->CCR1 = kolory[tactic][2];
		}
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
