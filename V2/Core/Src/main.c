/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PID_fast.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void PWM_out_H_brige(Motor_Sruct *motor, uint8_t channel);
static void init_motor(Motor_Sruct *motor,uint8_t motor_num);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//--------my global variables----------//
volatile uint16_t dma[4];
//volatile uint8_t error_my=0, feed_motion=0, M1_motion=0, polyarity,first_intrrupt=1; // 0-off 1-rigt, 2-left, 3-Rocking, 4-brake active
//volatile uint16_t PWM_M1=0; //0-1000 
//volatile uint16_t PWM_M4=0;//1000 RPS
//volatile uint32_t Freq_TIM2=10000;  //10000 is one rpm/sec or 600000 is one rpm/min
uint8_t init_state=0, tic_count = 0;
Motor_Sruct M1,M2;
uint16_t Step1_cnt_from_EXTI=0; 
uint16_t Step2_cnt_from_EXTI=0; 


//========================================//
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_IWDG_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  init_motor(&M1,1);
  init_motor(&M2,2);
  if (HAL_GPIO_ReadPin(GPIO_MODE_GPIO_Port,GPIO_MODE_Pin)){
    init_state=0;
  }else{
    init_state=1;
  }
  HAL_GPIO_WritePin(sleep1_GPIO_Port,sleep1_Pin,GPIO_PIN_SET);  //todo we may use this for some purpouses
  HAL_GPIO_WritePin(sleep2_GPIO_Port,sleep2_Pin,GPIO_PIN_SET);
  
  //check this one if you will generate code again
  if(init_state){
    /* creation of Debug_mode */
    
  }else{
    /* creation of Step_DIR */
  }  
  HAL_ADC_Stop(&hadc1);
  HAL_ADC_Stop_DMA(&hadc1);
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&dma,4);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
  HAL_Delay(1);  //ADC conversion
  uint16_t dma_I1_prev=dma[0],dma_V1_prev=dma[1];
  uint16_t dma_I2_prev=dma[0],dma_V2_prev=dma[1];
  uint8_t tic_prev=tic_count;
  TIM2->CCR1=0;
  TIM2->CCR2=0;
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  int32_t temp;
  /* USER CODE END 2 */
  
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if ((tic_prev!=tic_count)&&(tic_count==0)){  
      HAL_GPIO_TogglePin(Led_GPIO_Port,Led_Pin);
    }
    if (tic_prev!=tic_count){ //100hz
      HAL_IWDG_Refresh(&hiwdg);
      tic_prev=tic_count;
      M1.I_M=(dma_I1_prev+dma[0])>>1;
      dma_I1_prev=dma[0];
      M2.I_M=(dma_I2_prev+dma[2])>>1;
      dma_I2_prev=dma[2];
      
      switch (init_state){
      case 0:
        /* Debug_mode */
        PID_REG(&M1); // 0.01 sec
        PID_REG(&M2); // 0.01 sec
        PWM_out_H_brige(&M1,1);
        PWM_out_H_brige(&M2,2);
        break;
      case 1:
        /* Step_DIR control loop by position*/
        //------------------------------------M1--------------------------//
        if (HAL_GPIO_ReadPin(EN1_GPIO_Port,EN1_Pin)==GPIO_PIN_SET)
        {
          if (HAL_GPIO_ReadPin(DIR1_GPIO_Port,DIR1_Pin)==GPIO_PIN_SET)
          {
            if (Step1_cnt_from_EXTI>0&&Step1_cnt_from_EXTI<5000){
              M1.position_sp+=Step1_cnt_from_EXTI;
              Step1_cnt_from_EXTI=0;
            }
          }else{
            if (Step1_cnt_from_EXTI>0&&Step1_cnt_from_EXTI<5000){
              M1.position_sp-=Step1_cnt_from_EXTI;
              Step1_cnt_from_EXTI=0;
            }
          }
          PID_REG(&M1); // 0.01 sec        
        } else {
          M1.position_sp=0;
          M1.PWM_out=0;
          Step1_cnt_from_EXTI=0;
        }
        PWM_out_H_brige(&M1,1);  
        //----------------------------------------------------------------//
        
        //------------------------------------M2--------------------------//
        if (HAL_GPIO_ReadPin(EN2_GPIO_Port,EN2_Pin)==GPIO_PIN_SET)
        {
          if (HAL_GPIO_ReadPin(DIR2_GPIO_Port,DIR2_Pin)==GPIO_PIN_SET)
          {
            if (Step2_cnt_from_EXTI>0&&Step2_cnt_from_EXTI<5000){
              M2.position_sp+=Step2_cnt_from_EXTI;
              Step2_cnt_from_EXTI=0;
            }
          }else{
            if (Step2_cnt_from_EXTI>0&&Step2_cnt_from_EXTI<5000){
              M2.position_sp-=Step2_cnt_from_EXTI;
              Step2_cnt_from_EXTI=0;
            }
          }
          PID_REG(&M2); // 0.01 sec        
        } else {
          M2.position_sp=0;
          M2.PWM_out=0;
          Step2_cnt_from_EXTI=0;
        }
        PWM_out_H_brige(&M2,2); 
        //----------------------------------------------------------------//
        break;
      case 2:
        /* ManualTask control loop by velocity*/
        //------------------------------------M1--------------------------//
        if (HAL_GPIO_ReadPin(EN1_GPIO_Port,EN1_Pin)==GPIO_PIN_SET)
        {
          if (HAL_GPIO_ReadPin(DIR1_GPIO_Port,DIR1_Pin)==GPIO_PIN_SET)
          {
            temp=(M1.velocity_max*(dma_V1_prev+dma[1]))>>13;  //div 8096
            dma_V1_prev=dma[1];
            M1.velocity_sp=(temp>M1.velocity_max)?M1.velocity_max:(int16_t)temp;
          }else{
            temp=(-1)*((M1.velocity_max*(dma_V1_prev+dma[1]))>>13);  //div 8096
            dma_V1_prev=dma[1];
            M1.velocity_sp=(temp*(-1)>(M1.velocity_max))?(M1.velocity_max*(-1)):(int16_t)temp;
          }
          PID_REG(&M1); // 0.01 sec      
        }else{
          M1.PWM_out=0;
        }
        PWM_out_H_brige(&M1,1);
        //----------------------------------------------------------------//
        //------------------------------------M2--------------------------//
        if (HAL_GPIO_ReadPin(EN2_GPIO_Port,EN2_Pin)==GPIO_PIN_SET)
        {
          if (HAL_GPIO_ReadPin(DIR2_GPIO_Port,DIR2_Pin)==GPIO_PIN_SET)
          {
            temp=(M2.velocity_max*(dma_V2_prev+dma[3]))>>13;  //div 8096
            dma_V2_prev=dma[3];
            M2.velocity_sp=(temp>M2.velocity_max)?M2.velocity_max:(int16_t)temp;
          }else{
            temp=(-1)*((M2.velocity_max*(dma_V2_prev+dma[3]))>>13);  //div 8096
            dma_V2_prev=dma[3];
            M2.velocity_sp=(temp*(-1)>(M2.velocity_max))?(M2.velocity_max*(-1)):(int16_t)temp;
          }
          PID_REG(&M2); // 0.01 sec      
        }else{
          M1.PWM_out=0;
        }
        PWM_out_H_brige(&M2,2); 
        //----------------------------------------------------------------//
        break;
        default:
          break;
      } 
      //      HAL_ADC_Stop_DMA(&hadc1);
      //      HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&dma,4);  //todo check, changed to circular mode
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static void PWM_out_H_brige(Motor_Sruct *motor, uint8_t channel)
{
  if(channel==1){
    if(motor->PWM_out>=5)
    {
      HAL_GPIO_WritePin(GPIO_OUT1_GPIO_Port,GPIO_OUT1_Pin,GPIO_PIN_RESET);
      HAL_GPIO_WritePin(Mot1_DIR_GPIO_Port,Mot1_DIR_Pin,GPIO_PIN_SET);  //changeing direction of motor (dir 0 or 1) and PWM 0 equal STOP (driver mode connected to GND) 
      TIM2->CCR1=motor->PWM_out;
      motor->curr_direction=1;
    }else if (motor->PWM_out<=-5){
      HAL_GPIO_WritePin(GPIO_OUT1_GPIO_Port,GPIO_OUT1_Pin,GPIO_PIN_RESET);
      HAL_GPIO_WritePin(Mot1_DIR_GPIO_Port,Mot1_DIR_Pin,GPIO_PIN_RESET);//changeing direction of motor (dir 0 or 1) and PWM 0 equal STOP 
      TIM2->CCR1=motor->PWM_out*(-1);//0
      motor->curr_direction=-1;
    }else{      //reach control point
      TIM2->CCR1=0;
      HAL_GPIO_WritePin(GPIO_OUT1_GPIO_Port,GPIO_OUT1_Pin,GPIO_PIN_SET);
    }
  }else {
    if(motor->PWM_out>=5)
    {
      HAL_GPIO_WritePin(Mot2_DIR_GPIO_Port,Mot2_DIR_Pin,GPIO_PIN_SET);//changeing direction of motor (dir 0 or 1) and PWM 0 equal STOP 
      TIM2->CCR2=motor->PWM_out;
      motor->curr_direction=1;
      HAL_GPIO_WritePin(GPIO_OUT2_GPIO_Port,GPIO_OUT2_Pin,GPIO_PIN_RESET);
    }else if (motor->PWM_out<=-5){
      HAL_GPIO_WritePin(Mot2_DIR_GPIO_Port,Mot2_DIR_Pin,GPIO_PIN_RESET);//changeing direction of motor (dir 0 or 1) and PWM 0 equal STOP 
      TIM2->CCR2=motor->PWM_out*(-1);//0
      motor->curr_direction=-1;
      HAL_GPIO_WritePin(GPIO_OUT2_GPIO_Port,GPIO_OUT2_Pin,GPIO_PIN_RESET);
    }else{      //reach control point
      HAL_GPIO_WritePin(GPIO_OUT2_GPIO_Port,GPIO_OUT2_Pin,GPIO_PIN_SET);
      TIM2->CCR2=0;
    }
  }
}

//Function for setting default values to motors, in future probably here will be settings from flash, in this case use motor_num for defenition of which motor is it. 
static void init_motor(Motor_Sruct *motor,uint8_t motor_num){  
  motor->curr_direction=0;
  motor->position=0;
  motor->position_sp=0; 
  motor->velocity=0;
  motor->velocity_max=Encoder1_Pulses_per_rotation*Max_RPM1/600;//todo check according to motor, also use 2 different according to motor num
  motor->velocity_sp=0;
  motor->I_M_max=4096;  //todo check according to driver
  motor->I_M=0;
  motor->I_M_sp=0;
  motor->PWM_out=0;
  motor->last_counter_value=0;
  motor->D_position=(motor_num==1)?M1_PID_POS_D:M2_PID_POS_D;
  motor->I_position=(motor_num==1)?M1_PID_POS_I:M2_PID_POS_I;
  motor->P_position=(motor_num==1)?M1_PID_POS_P:M2_PID_POS_P;
  motor->D_velocity=(motor_num==1)?M1_PID_Vel_D:M2_PID_Vel_D;
  motor->I_velocity=(motor_num==1)?M1_PID_Vel_I:M2_PID_Vel_I;
  motor->P_velocity=(motor_num==1)?M1_PID_Vel_P:M2_PID_Vel_P;
  motor->D_current =(motor_num==1)?M1_PID_Cur_D:M2_PID_Cur_D;
  motor->I_current =(motor_num==1)?M1_PID_Cur_I:M2_PID_Cur_I;
  motor->P_current =(motor_num==1)?M1_PID_Cur_P:M2_PID_Cur_P;
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
