/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : A program for omni-directional robot
 * 				The following picture shows the variable name of each wheel
 * 				 /------------------------------\
* 				/ wheel1        |        wheel2  \
* 								|
 * 								|
 * 								|
 * 								|
 * 								|
 * 								|
 * 								|
 * 							 --------
 * 							  wheel3
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
#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "robot.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MASTER_BOARD

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
double omega1;
double omega2;
double omega3;
int count_time;
int counter1 = 0;
int getCNT = 0;
struct ROBOT omni_drive;
volatile uint32_t previous_cnt, cnt = 0;
int which_motor = 0;
int rxcpltCount;

int initCheck;
int qaq = 0;
uint8_t data = 147;
int action_finished = 0;
int ouo = 0;
int i2c_cnt = 0;
double yaw_avg = 0;
double angle_integral = 0;
int transfer_finish = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim6) {
//		if(transfer_finish){
			counter1++;
			s_curve(&omni_drive, omni_drive.control.linearVelocity_x,
					omni_drive.control.linearVelocity_y,
					omni_drive.control.angularVelocity);
	//		if(omni_drive.arm_ptr->right.direction > radian(90) && omni_drive.arm_ptr->right.initialized ==1)
	//			stepper_move(omni_drive.arm_ptr, 0, PI/2);
	//		arm_move(omni_drive.arm_ptr, omni_drive.arm_ptr->cmdvel_x, omni_drive.arm_ptr->cmdvel_y);
	//		omni_drive.control.motor[0].target = 0;
	//		omni_drive.control.motor[1].target = 0;
	//		omni_drive.control.motor[2].target = 0;
			endjoint_script(&omni_drive.end_joint, omni_drive.end_joint.command);
			stepper_angle_tracking(omni_drive.arm_ptr,omni_drive.arm_ptr->left.tracking_angle,omni_drive.arm_ptr->right.tracking_angle);
			PID(&omni_drive);
			HAL_ADC_Start_IT(omni_drive.end_joint.sucker_hardware.Psensor_adc);

			action_finished = 1;
//		}
	}
	if (htim == &htim7) {
		if(!omni_drive.imu.data_processing)
			MPU6050_Read_Gyro(&hi2c1,&omni_drive.imu,&omni_drive.imu.Rec_Data[0]);
//		omni_drive.deltaXSum += omni_drive.pmw3901.deltaX;
//		omni_drive.deltaYSum += omni_drive.pmw3901.deltaY;
	}
	if(htim == &htim14){
		if(action_finished){
			transfer_finish = 0;
			Tx_Data_Update(&omni_drive);
			Uart_Transmit(&omni_drive);
			action_finished = 0;
			transfer_finish = 1;
		}
	}
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi == omni_drive.pmw3901.hspi) {
//		HAL_SPI_Transmit_DMA(omni_drive.pmw3901.hspi, &data, 4);
		ouo++;
	}
//		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi == omni_drive.pmw3901.hspi) {
////		HAL_SPI_Transmit_DMA(omni_drive.pmw3901.hspi, &data, 4);
//		qaq++;
	}
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi == omni_drive.pmw3901.hspi) {
//		HAL_SPI_Transmit_DMA(omni_drive.pmw3901.hspi, &data, 4);
//		ouo++;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_1) {
		counter1++;

	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == omni_drive.communicate.uart.huart) {
		Uart_RxCplt(&omni_drive);
	}
}



void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c == &hi2c1) {

	}
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) { /*confirm whether i2c rx function work*/
	if (hi2c == &hi2c1) {

		omni_drive.imu.Gyro_X_RAW = (int16_t)(omni_drive.imu.Rec_Data[8] << 8 | omni_drive.imu.Rec_Data[9]);
		omni_drive.imu.Gyro_Y_RAW = (int16_t)(omni_drive.imu.Rec_Data[10] << 8 | omni_drive.imu.Rec_Data[11]);
		omni_drive.imu.Gyro_Z_RAW = (int16_t)(omni_drive.imu.Rec_Data[12] << 8 | omni_drive.imu.Rec_Data[13]);



		omni_drive.imu.Accel_X_RAW = (int16_t)(omni_drive.imu.Rec_Data[0] << 8 | omni_drive.imu.Rec_Data[1]);
		omni_drive.imu.Accel_Y_RAW = (int16_t)(omni_drive.imu.Rec_Data[2] << 8 | omni_drive.imu.Rec_Data[3]);
		omni_drive.imu.Accel_Z_RAW = (int16_t)(omni_drive.imu.Rec_Data[4] << 8 | omni_drive.imu.Rec_Data[5]);


		omni_drive.imu.Ax = omni_drive.imu.Accel_X_RAW / 16384.0;
		omni_drive.imu.Ax *= 9.81;
		omni_drive.imu.Ax -= 0.35;


		omni_drive.imu.Ay = omni_drive.imu.Accel_Y_RAW / 16384.0;
		omni_drive.imu.Ay *= 9.81;
		omni_drive.imu.Ay += 0.058;

		omni_drive.imu.Gx = omni_drive.imu.Gyro_X_RAW / 131.0;
		omni_drive.imu.Gy = omni_drive.imu.Gyro_Y_RAW / 131.0;
		omni_drive.imu.Gz = omni_drive.imu.Gyro_Z_RAW / 131.0;
		omni_drive.imu.Gz *= M_PI / 180;
		omni_drive.imu.Gz += 0.0111;

//		omni_drive.communicate.uart.tx[9] = omni_drive.imu.Gz * 100.0;
//		omni_drive.communicate.uart.tx[7] = omni_drive.imu.Ax * 100.0;
//		omni_drive.communicate.uart.tx[8] = omni_drive.imu.Ay * 100.0;


		yaw_avg += omni_drive.imu.Gz;
		i2c_cnt ++;
		angle_integral +=( omni_drive.imu.Gz * 0.01) ;

		yaw_avg /= i2c_cnt;
		omni_drive.imu.data_processing = 0;
	}
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    // Update The ADC Result
//    end_joint.adc_value[sample_number%10] = HAL_ADC_GetValue(end_joint.sucker_hardware.Psensor_adc);
	 qaq++;
     omni_drive.end_joint.adc_value = HAL_ADC_GetValue(omni_drive.end_joint.sucker_hardware.Psensor_adc);
//     end_joint.cur_voltage = 3.3*value/4096;
}

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
  MX_TIM6_Init();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_CRC_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM12_Init();
  MX_TIM13_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM14_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
	Initiate(&omni_drive);
//	HAL_SPI_Transmit_DMA(omni_drive.pmw3901.hspi, &data, 4);
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_3,0);
	HAL_Delay(50);
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_3,1);
	HAL_Delay(50);
	initCheck = MPU6050_Init(&hi2c1);

//	initCheck = pmw_init(&omni_drive.pmw3901);
//  qaq ++;
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);
//    HAL_TIM_Base_Start_IT(&htim11);
    HAL_TIM_Base_Start_IT(&htim14);

//  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);
//  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
//  HAL_GPIO_EXTI_Callback(GPIO_Pin)
//  omni_drive.control.motor[0].target = 3950;
//motor max speed 3950 mm/s
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
//		HAL_GPIO_WritePin(omni_drive.end_joint.sucker_hardware.suck_port, omni_drive.end_joint.sucker_hardware.suck_pin, 1);
//		readMotionCount(&omni_drive.pmw3901);
//		stepper_move(omni_drive.arm_ptr, 0, -PI / 8);
//		HAL_SPI_Transmit(omni_drive.pmw3901.hspi, &data, 1,50);
//		MPU6050_Read_Gyro(&hi2c1,&omni_drive.imu);
//	HAL_Delay(10);
//	ouo ++;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  setMotorSpeed(&omni_drive.control.motor[which_motor], 1, 3000);
//	arm_move(omni_drive.arm_ptr, omni_drive.arm_ptr->cmdvel_x, omni_drive.arm_ptr->cmdvel_y);
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
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
	while (1) {
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
