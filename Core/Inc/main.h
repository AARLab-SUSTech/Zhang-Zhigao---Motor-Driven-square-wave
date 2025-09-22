/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdbool.h"
#include "arm_math.h"
#include "math.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define TWO_PI 6.28318530718f             // 2 * PI
#define PHASE_2_32 4294967296.0            // 2^32, 用 double 提高计算精度
//#define DC_Power_ON  	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET)
//#define DC_Power_OFF  	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET)
#define DC_Power_ON  	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET)
#define DC_Power_OFF  	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET)
#define Buzzer_ON		  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
#define Buzzer_OFF		  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/* 在 main.h 的某个位置, 例如 USER CODE BEGIN EFP */
// ====================================================================
// ===         所有全局变量【声明】在这里 (使用 extern)           ===
// ====================================================================
extern uint16_t duty_TIM1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim16;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
// --- 串口资源 ---
#define RX_BUFFER_SIZE 256
extern uint8_t rx_buffer[RX_BUFFER_SIZE];
extern uint8_t process_buffer[RX_BUFFER_SIZE];

// --- 电机控制状态变量 ---
extern volatile int32_t repeated_pos_A;
extern volatile int32_t repeated_pos_B;
extern volatile uint32_t repeated_count_total;
extern volatile uint32_t repeated_count_current;
extern volatile float repeated_speed_hz;
extern volatile uint32_t phase_accumulator;
extern volatile uint32_t phase_increment;
extern volatile uint16_t pwm_duty_value;
extern volatile uint8_t step;
extern float interrupt_freq_hz;

extern volatile int32_t absolute_step_counter;
extern volatile int32_t target_step_position;
extern volatile uint32_t position_mode_increment;

extern uint32_t ADC1_RAW_data[2];
extern uint32_t ADC2_RAW_data[3];
extern float HV_V,HV_I,DC_I;

extern bool DC_ON_State;

extern COMP_HandleTypeDef hcomp1;
extern COMP_HandleTypeDef hcomp2;

typedef enum {
    MOTOR_IDLE = 0,             // 静止/待机模式
    MOTOR_ERROR,                // 错误/急停模式
    MOTOR_OPEN_SPEED,           // 速度模式
    MOTOR_OPEN_POSITION,        // 单次定位模式
	MOTOR_OPEN_REPEATED,         // 往复运动模式
	MOTOR_OVER_HV_CURRENT,
	MOTOR_OVER_HV_VOLTAGE,
	MOTOR_OVER_DC_IN_CURRENT
} Motor_mode_t;
typedef enum {
    MOTOR_REVERSE = 0,
    MOTOR_FORWARD = 1
} MotorDirection_t;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define HV1P_Pin GPIO_PIN_0
#define HV1P_GPIO_Port GPIOA
#define HV1N_Pin GPIO_PIN_1
#define HV1N_GPIO_Port GPIOA
#define HVIP_Pin GPIO_PIN_2
#define HVIP_GPIO_Port GPIOA
#define HVIN_Pin GPIO_PIN_3
#define HVIN_GPIO_Port GPIOA
#define SIN_Pin GPIO_PIN_4
#define SIN_GPIO_Port GPIOA
#define COS_Pin GPIO_PIN_5
#define COS_GPIO_Port GPIOA
#define I_O_Pin GPIO_PIN_6
#define I_O_GPIO_Port GPIOA
#define Power_ON_Pin GPIO_PIN_11
#define Power_ON_GPIO_Port GPIOB
#define Buzzer_Pin GPIO_PIN_15
#define Buzzer_GPIO_Port GPIOA
#define CH1__CTR_Pin GPIO_PIN_9
#define CH1__CTR_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
