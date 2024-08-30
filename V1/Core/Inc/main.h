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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Led_Pin GPIO_PIN_13
#define Led_GPIO_Port GPIOC
#define Current_Pin GPIO_PIN_0
#define Current_GPIO_Port GPIOA
#define Speed_Pin GPIO_PIN_1
#define Speed_GPIO_Port GPIOA
#define EN_Pin GPIO_PIN_0
#define EN_GPIO_Port GPIOB
#define DIR_Pin GPIO_PIN_1
#define DIR_GPIO_Port GPIOB
#define DIP01_Pin GPIO_PIN_10
#define DIP01_GPIO_Port GPIOB
#define DIP10_Pin GPIO_PIN_11
#define DIP10_GPIO_Port GPIOB
#define TIM1_Encoder1_Pin GPIO_PIN_8
#define TIM1_Encoder1_GPIO_Port GPIOA
#define TIM1_Encoder2_Pin GPIO_PIN_9
#define TIM1_Encoder2_GPIO_Port GPIOA
#define TIM3_IC_STEP_Pin GPIO_PIN_4
#define TIM3_IC_STEP_GPIO_Port GPIOB
#define EXTI_Step_Pin GPIO_PIN_5
#define EXTI_Step_GPIO_Port GPIOB
#define EXTI_Step_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */
   typedef struct {
     int16_t velocity;             //10HZ encoder val chenges example: speed is 1 rotation per second 360/10 == 36 counts  
     int64_t position;
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
   
   

#define Encoder_Pulses_per_rotation 360
#define Max_RPM 6000
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
