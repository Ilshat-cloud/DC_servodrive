/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
   typedef struct {
     int16_t velocity;             //10HZ encoder val chenges example: speed is 1 rotation per second 360/10 == 36 velocity counts  (cnt of impulses for 0.1 s) 
     int64_t position;             //in encoder impulses
     uint16_t last_counter_value;
     int64_t position_sp;
     int16_t velocity_sp;
     int16_t I_M_sp;
     uint16_t I_M;
     uint16_t I_M_max;
     uint16_t velocity_max;
     uint8_t P_position;
     uint8_t I_position;
     uint8_t D_position;
     uint8_t P_velocity;
     uint8_t I_velocity;
     uint8_t D_velocity;
     uint8_t P_current;
     uint8_t I_current;
     uint8_t D_current; 
     int16_t PWM_out;
     int8_t curr_direction; //calculate when pwm active
   }Motor_Sruct;
   
   

#define Encoder1_Pulses_per_rotation 360
#define Max_RPM1 6000
#define Encoder2_Pulses_per_rotation 360
#define Max_RPM2 6000   
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Led_Pin GPIO_PIN_13
#define Led_GPIO_Port GPIOC
#define GPIO_OUT1_Pin GPIO_PIN_14
#define GPIO_OUT1_GPIO_Port GPIOC
#define GPIO_OUT2_Pin GPIO_PIN_15
#define GPIO_OUT2_GPIO_Port GPIOC
#define Cur1_to_ADC_Pin GPIO_PIN_0
#define Cur1_to_ADC_GPIO_Port GPIOA
#define SP_1_Pin GPIO_PIN_1
#define SP_1_GPIO_Port GPIOA
#define Cur2_to_ADC_Pin GPIO_PIN_2
#define Cur2_to_ADC_GPIO_Port GPIOA
#define SP_2_Pin GPIO_PIN_3
#define SP_2_GPIO_Port GPIOA
#define EXTI_Step2_Pin GPIO_PIN_4
#define EXTI_Step2_GPIO_Port GPIOA
#define EXTI_Step2_EXTI_IRQn EXTI4_IRQn
#define EXTI_Step1_Pin GPIO_PIN_5
#define EXTI_Step1_GPIO_Port GPIOA
#define EXTI_Step1_EXTI_IRQn EXTI9_5_IRQn
#define TIM3_Encoder1_Pin GPIO_PIN_6
#define TIM3_Encoder1_GPIO_Port GPIOA
#define TIM3_Encoder2_Pin GPIO_PIN_7
#define TIM3_Encoder2_GPIO_Port GPIOA
#define GPIO_MODE_Pin GPIO_PIN_0
#define GPIO_MODE_GPIO_Port GPIOB
#define DIR1_Pin GPIO_PIN_1
#define DIR1_GPIO_Port GPIOB
#define DIR2_Pin GPIO_PIN_2
#define DIR2_GPIO_Port GPIOB
#define EN1_Pin GPIO_PIN_10
#define EN1_GPIO_Port GPIOB
#define EN2_Pin GPIO_PIN_11
#define EN2_GPIO_Port GPIOB
#define Mot1_DIR_Pin GPIO_PIN_12
#define Mot1_DIR_GPIO_Port GPIOB
#define Mot2_DIR_Pin GPIO_PIN_13
#define Mot2_DIR_GPIO_Port GPIOB
#define sleep1_Pin GPIO_PIN_14
#define sleep1_GPIO_Port GPIOB
#define sleep2_Pin GPIO_PIN_15
#define sleep2_GPIO_Port GPIOB
#define TIM1_Encoder1_Pin GPIO_PIN_8
#define TIM1_Encoder1_GPIO_Port GPIOA
#define TIM1_Encoder2_Pin GPIO_PIN_9
#define TIM1_Encoder2_GPIO_Port GPIOA
#define PWM1_Pin GPIO_PIN_15
#define PWM1_GPIO_Port GPIOA
#define PWM2_Pin GPIO_PIN_3
#define PWM2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
