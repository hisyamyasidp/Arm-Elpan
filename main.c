/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h> // for memset
#include <stdbool.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

int Enc_sampling1;
int Enc_Count1;

int Enc_sampling2;
int Enc_Count2;

int Enc_sampling3;
int Enc_Count3;

int encoder1;
int encoder2;
int encoder3;

int banyakdata = 20;
char PC[20];
char SendPC[50];
int32_t banyakdataSendPC = sizeof(SendPC);

int pwm1;
int pwm2;
int pwm3;

int Limit1;
int Limit2;
int Limit3;

uint32_t lastSerialTime = 0;
const uint32_t serialTimeoutMs = 500; // timeout 500 ms, bisa diubah sesuai kebutuhan

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

long map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

typedef struct {
	UART_HandleTypeDef *serial;
	uint8_t  raw_data[20];
	uint8_t  index;
	uint8_t  enable;
	char     Data1[6];   // Rotasi
	char     Data2[6];   // Updown
	char     Data3[6];   // MajuMundur
} Data_Typedef;

Data_Typedef DataPC = { .serial = &huart3, .enable = 1, .index = 0 };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */

void SerialInit() {

	strcpy(DataPC.Data1, "00000");
	strcpy(DataPC.Data2, "00000");
	strcpy(DataPC.Data3, "00000");
}

void ParsingData() {
	pwm1 = atoi(DataPC.Data1);
	pwm2 = atoi(DataPC.Data2);
	pwm3 = atoi(DataPC.Data3);
}

void SerialReset(Data_Typedef *data) {
	memset(data->raw_data, 0, sizeof(data->raw_data));
	strcpy(data->Data1, "00000");
	strcpy(data->Data2, "00000");
	strcpy(data->Data3, "00000");
	data->index = 0;
}

void Data_Receive(Data_Typedef *DataPC) {
	if (DataPC->enable) {
		if (DataPC->raw_data[DataPC->index % 20] == '*') {
			if (DataPC->raw_data[(DataPC->index + 18) % 20] == '#') {
				DataPC->Data1[0] = (char)DataPC->raw_data[(DataPC->index + 1) % 20];
				DataPC->Data1[1] = (char)DataPC->raw_data[(DataPC->index + 2) % 20];
				DataPC->Data1[2] = (char)DataPC->raw_data[(DataPC->index + 3) % 20];
				DataPC->Data1[3] = (char)DataPC->raw_data[(DataPC->index + 4) % 20];
				DataPC->Data1[4] = (char)DataPC->raw_data[(DataPC->index + 5) % 20];
				DataPC->Data1[5] = '\0';

				DataPC->Data2[0] = (char)DataPC->raw_data[(DataPC->index + 7) % 20];
				DataPC->Data2[1] = (char)DataPC->raw_data[(DataPC->index + 8) % 20];
				DataPC->Data2[2] = (char)DataPC->raw_data[(DataPC->index + 9) % 20];
				DataPC->Data2[3] = (char)DataPC->raw_data[(DataPC->index + 10) % 20];
				DataPC->Data2[4] = (char)DataPC->raw_data[(DataPC->index + 11) % 20];
				DataPC->Data2[5] = '\0';

				DataPC->Data3[0] = (char)DataPC->raw_data[(DataPC->index + 13) % 20];
				DataPC->Data3[1] = (char)DataPC->raw_data[(DataPC->index + 14) % 20];
				DataPC->Data3[2] = (char)DataPC->raw_data[(DataPC->index + 15) % 20];
				DataPC->Data3[3] = (char)DataPC->raw_data[(DataPC->index + 16) % 20];
				DataPC->Data3[4] = (char)DataPC->raw_data[(DataPC->index + 17) % 20];
				DataPC->Data3[5] = '\0';

				lastSerialTime = HAL_GetTick();
			} else {
				DataPC->index++;
				if (DataPC->index >= 20) {
					DataPC->index = 0;
				}
			}
		} else {
			DataPC->index++;
			if (DataPC->index >= 20) {
				DataPC->index = 0;
				memset(DataPC->raw_data, 0, sizeof(DataPC->raw_data));
			}
		}
	}
}

void Drive_motor1(int pwm1) {

	// Ch 1 Use CCR1
	// Ch 2 Use CCR2

	HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
	TIM14->CCR1 = abs(pwm1);

	HAL_GPIO_WritePin(MTR1A_GPIO_Port, MTR1A_Pin,
			(pwm1 > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MTR1B_GPIO_Port, MTR1B_Pin,
			(pwm1 < 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);

}

void Drive_motor2(int pwm2) {
	HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
	TIM13->CCR1 = abs(pwm2);

	HAL_GPIO_WritePin(MTR2A_GPIO_Port, MTR2A_Pin,
			(pwm2 > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MTR2B_GPIO_Port, MTR2B_Pin,
			(pwm2 < 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Drive_motor3(int pwm3) {
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
	TIM9->CCR2 = abs(pwm3);

	HAL_GPIO_WritePin(MTR3A_GPIO_Port, MTR3A_Pin,
			(pwm3 > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MTR3B_GPIO_Port, MTR3B_Pin,
			(pwm3 < 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Val_Roda (int Roda1,int Roda2,int Roda3){
	Drive_motor1(Roda1);
	Drive_motor2(Roda2);
	Drive_motor3(Roda3);
}

void Run_Limit (){
	Limit1 = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12);	  //
	Limit2 = !HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_8);	    //
	Limit3 = !HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_7);	  //
}

int32_t Get_Enc1() {
	// Compute delta using unsigned arithmetic
	uint16_t current_count = TIM1->CNT;
	int16_t delta = (int16_t)(current_count - 65535U);  // Safe overflow handling
	TIM1->CNT = 65535;  // Reset timer
	Enc_Count1 += delta; // Accumulate in 32-bit
	return Enc_Count1;
}

int32_t Get_Enc2() {
	// Compute delta using unsigned arithmetic
	uint16_t current_count = TIM8->CNT;
	int16_t delta = (int16_t)(current_count - 65535U);  // Safe overflow handling
	TIM8->CNT = 65535;  // Reset timer
	Enc_Count2 += delta; // Accumulate in 32-bit
	return Enc_Count2;
}

int32_t Get_Enc3() {
	// Compute delta using unsigned arithmetic
	uint16_t current_count = TIM4->CNT;
	int16_t delta = (int16_t)(current_count - 65535U);  // Safe overflow handling
	TIM4->CNT = 65535;  // Reset timer
	Enc_Count3 += delta; // Accumulate in 32-bit
	return Enc_Count3;
}

void HitungEncoder() {
	encoder1 = (int) Get_Enc1();  //
	encoder2 = (int) Get_Enc2();  //
	encoder3 = (int) Get_Enc3();  //
}

void Encoder_Init() {
	HAL_TIM_Encoder_Start_IT(&htim1, TIM_CHANNEL_ALL);

	HAL_TIM_Encoder_Start_IT(&htim8, TIM_CHANNEL_ALL);

	HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);

	Enc_sampling1=0;
	Enc_sampling2=0;
	Enc_sampling3=0;


	TIM1->CNT=65535;
	TIM8->CNT=65535;
	TIM4->CNT=65535;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	//	HAL_UART_Receive_IT(&huart3, (uint8_t*) MiniPC, banyakdata);
	if (huart->Instance == USART3) {
    Data_Receive(&DataPC);
		if (DataPC.enable) {
			HAL_UART_Receive_IT(DataPC.serial, DataPC.raw_data, 20);
		}
	}
}
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
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_USART3_UART_Init();
  MX_TIM9_Init();
  MX_TIM11_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */

	SerialInit();
	Encoder_Init();

	encoder1 = 0;
	encoder2 = 0;
	encoder3 = 0;

	Limit1 = 0;
	Limit2 = 0;
	Limit3 = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  if (DataPC.enable) {
	  		HAL_UART_Receive_IT(DataPC.serial, DataPC.raw_data, 19);
	  	}

	  		// Timeout: reset data jika tidak ada data baru dalam serialTimeoutMs
	  	if (HAL_GetTick() - lastSerialTime > serialTimeoutMs) {
	  		Val_Roda(0,0,0);
	  		pwm1 = 0;
	  		pwm2 = 0;
	  		pwm3 = 0;
	  		SerialReset(&DataPC);
	  	}

	  	ParsingData();
	  	HitungEncoder();
	  	Run_Limit();

	  	Val_Roda(pwm1,pwm2,pwm3);
	  	//	  strcpy()
	  	sprintf(SendPC, "%d | %d | %d | %d | %d | %d\n", encoder1, encoder2, encoder3, Limit1, Limit2, Limit3);


	  	//	  // Transmit the data using HAL_UART_Transmit
	  	banyakdataSendPC = strlen(SendPC);

	  	HAL_UART_Transmit(&huart3, (uint8_t*) SendPC, banyakdataSendPC, 1);


	      /* USER CODE BEGIN 3 */
	    }
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */


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
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
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
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 15;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 1024;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 15;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 1024;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */
  HAL_TIM_MspPostInit(&htim11);

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 15;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 1024;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */
  HAL_TIM_MspPostInit(&htim13);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 15;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 1024;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
  HAL_TIM_MspPostInit(&htim14);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 921600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, MTR4A_Pin|MTR4B_Pin|MTR3B_Pin|MTR5B_Pin
                          |MTR5A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, MTR3A_Pin|MTR1A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MTR2B_Pin|MTR2A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MTR1B_GPIO_Port, MTR1B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MTR4A_Pin MTR4B_Pin MTR3B_Pin MTR5B_Pin
                           MTR5A_Pin */
  GPIO_InitStruct.Pin = MTR4A_Pin|MTR4B_Pin|MTR3B_Pin|MTR5B_Pin
                          |MTR5A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : MTR3A_Pin MTR1A_Pin */
  GPIO_InitStruct.Pin = MTR3A_Pin|MTR1A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MTR2B_Pin MTR2A_Pin */
  GPIO_InitStruct.Pin = MTR2B_Pin|MTR2A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MTR1B_Pin */
  GPIO_InitStruct.Pin = MTR1B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MTR1B_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LIMIT3_Pin LIMIT2_Pin */
  GPIO_InitStruct.Pin = LIMIT3_Pin|LIMIT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : LIMIT1_Pin */
  GPIO_InitStruct.Pin = LIMIT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LIMIT1_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
