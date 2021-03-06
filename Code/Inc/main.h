/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define SHARP1_Pin GPIO_PIN_13
#define SHARP1_GPIO_Port GPIOC
#define SHARP2_Pin GPIO_PIN_14
#define SHARP2_GPIO_Port GPIOC
#define SHARP3_Pin GPIO_PIN_12
#define SHARP3_GPIO_Port GPIOB
#define SHARP4_Pin GPIO_PIN_13
#define SHARP4_GPIO_Port GPIOB
#define STOP_MODULE_Pin GPIO_PIN_14
#define STOP_MODULE_GPIO_Port GPIOB
#define START_MODULE_Pin GPIO_PIN_15
#define START_MODULE_GPIO_Port GPIOB
#define LED_B_Pin GPIO_PIN_10
#define LED_B_GPIO_Port GPIOA
#define TACTIC_BUTTON_Pin GPIO_PIN_11
#define TACTIC_BUTTON_GPIO_Port GPIOA
#define TACTIC_BUTTON_EXTI_IRQn EXTI15_10_IRQn
#define LED_FAIL_Pin GPIO_PIN_12
#define LED_FAIL_GPIO_Port GPIOA
#define START_BUTTON_Pin GPIO_PIN_15
#define START_BUTTON_GPIO_Port GPIOA
#define START_BUTTON_EXTI_IRQn EXTI15_10_IRQn
#define STOP_BUTTON_Pin GPIO_PIN_3
#define STOP_BUTTON_GPIO_Port GPIOB
#define DIR2A_Pin GPIO_PIN_4
#define DIR2A_GPIO_Port GPIOB
#define DIR2B_Pin GPIO_PIN_5
#define DIR2B_GPIO_Port GPIOB
#define PWM2_Pin GPIO_PIN_6
#define PWM2_GPIO_Port GPIOB
#define DIRA_Pin GPIO_PIN_7
#define DIRA_GPIO_Port GPIOB
#define DIRB_Pin GPIO_PIN_8
#define DIRB_GPIO_Port GPIOB
#define PWM1_Pin GPIO_PIN_9
#define PWM1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
