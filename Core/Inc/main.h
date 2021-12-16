/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define VNH_3A_Pin GPIO_PIN_2
#define VNH_3A_GPIO_Port GPIOE
#define VNH_3B_Pin GPIO_PIN_3
#define VNH_3B_GPIO_Port GPIOE
#define VNH_4A_Pin GPIO_PIN_4
#define VNH_4A_GPIO_Port GPIOE
#define VNH3_PWM_Pin GPIO_PIN_5
#define VNH3_PWM_GPIO_Port GPIOE
#define VNH4_PWM_Pin GPIO_PIN_6
#define VNH4_PWM_GPIO_Port GPIOE
#define VNH_4B_Pin GPIO_PIN_13
#define VNH_4B_GPIO_Port GPIOC
#define STEPPER2_STEP_Pin GPIO_PIN_8
#define STEPPER2_STEP_GPIO_Port GPIOF
#define ENC_2B_Pin GPIO_PIN_0
#define ENC_2B_GPIO_Port GPIOA
#define ENC_2A_Pin GPIO_PIN_1
#define ENC_2A_GPIO_Port GPIOA
#define STEPPER2_DIR_Pin GPIO_PIN_11
#define STEPPER2_DIR_GPIO_Port GPIOF
#define STEPPER2_MS3_Pin GPIO_PIN_12
#define STEPPER2_MS3_GPIO_Port GPIOF
#define STEPPER2_MS2_Pin GPIO_PIN_13
#define STEPPER2_MS2_GPIO_Port GPIOF
#define STEPPER2_MS1_Pin GPIO_PIN_14
#define STEPPER2_MS1_GPIO_Port GPIOF
#define STEPPER3_DIR_Pin GPIO_PIN_15
#define STEPPER3_DIR_GPIO_Port GPIOF
#define STEPPER3_MS3_Pin GPIO_PIN_0
#define STEPPER3_MS3_GPIO_Port GPIOG
#define STEPPER3_MS2_Pin GPIO_PIN_1
#define STEPPER3_MS2_GPIO_Port GPIOG
#define STEPPER3_MS1_Pin GPIO_PIN_7
#define STEPPER3_MS1_GPIO_Port GPIOE
#define VNH_2A_Pin GPIO_PIN_12
#define VNH_2A_GPIO_Port GPIOB
#define VNH_2B_Pin GPIO_PIN_13
#define VNH_2B_GPIO_Port GPIOB
#define VNH2_PWM_Pin GPIO_PIN_14
#define VNH2_PWM_GPIO_Port GPIOB
#define sucker_cw_Pin GPIO_PIN_8
#define sucker_cw_GPIO_Port GPIOD
#define ENC_4B_Pin GPIO_PIN_12
#define ENC_4B_GPIO_Port GPIOD
#define ENC_4A_Pin GPIO_PIN_13
#define ENC_4A_GPIO_Port GPIOD
#define sucker_ccw_Pin GPIO_PIN_14
#define sucker_ccw_GPIO_Port GPIOD
#define ENC_1A_Pin GPIO_PIN_15
#define ENC_1A_GPIO_Port GPIOA
#define ENC_1B_Pin GPIO_PIN_3
#define ENC_1B_GPIO_Port GPIOB
#define ENC_3A_Pin GPIO_PIN_4
#define ENC_3A_GPIO_Port GPIOB
#define ENC_3B_Pin GPIO_PIN_5
#define ENC_3B_GPIO_Port GPIOB
#define Servo_Pin GPIO_PIN_8
#define Servo_GPIO_Port GPIOB
#define STEPPER3_STEP_Pin GPIO_PIN_9
#define STEPPER3_STEP_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
