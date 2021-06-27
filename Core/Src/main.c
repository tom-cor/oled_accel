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


#include "../../ssd1306/ssd1306.h"
#include "../../ssd1306/FIRFilter.h"
#include "../../ssd1306/mpu6050.h"
#include "math.h"
#include "stdio.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


#define PI 3.14159265358


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */


char display_mode = 0;

static FIRFilter az_filter;	//	Acc_Z filtered

static FIRFilter angle_yx_filter;
static FIRFilter angle_xz_filter;
static FIRFilter angle_yz_filter;

static float angle_yx = 0;
static float angle_xz = 0;
static float angle_yz = 0;

typedef struct
{
	float yx;
	float xz;
	float yz;
} ANGLES;

ANGLES angle;




static MPU6050 mpu6050;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */


//static void bubbleLevel_ArtifHorizon(int16_t angle);
static void bubbleLevel_2d(float angle_xz, float angle_yz);
static void bubbleLevel_1d(float angle);


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


  FIRFilter_Init(&az_filter);
  FIRFilter_Init(&angle_yx_filter);


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  mpu6050_Init(&mpu6050);
  ssd1306_Init();

  ssd1306_Fill(Black);
  ssd1306_SetCursor(1, 11);
  ssd1306_WriteString("Proyecto final", Font_7x10, White);
  ssd1306_SetCursor(1, 27);
  ssd1306_WriteString("Sistemas Embebidos", Font_7x10, White);
  ssd1306_SetCursor(1, 43);
  ssd1306_WriteString("Tomas Cornaglia", Font_7x10, White);
  ssd1306_UpdateScreen();

  HAL_Delay(3000);

  HAL_I2C_Mem_Write(&hi2c1, (MPU6050_ADDRESS<<1) | 0, PWRMNGT1_REG, 1, 0x00, 1, 100);

  HAL_TIM_Base_Start_IT(&htim3);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	switch(display_mode)
	{
		case 1:
			bubbleLevel_1d(angle_yx);
			break;

		case 2:
			bubbleLevel_2d(angle_yz, angle_xz);
			break;
	}

	if( (az_filter.out < 0.3) && (display_mode != 1)  )	// Revisar como evaluar que el timer ya está corriendo, cosa de no iniciarlo más de una vez.
		HAL_TIM_Base_Start_IT(&htim2);

	if( (az_filter.out > 0.7) && (display_mode != 2) )
		HAL_TIM_Base_Start_IT(&htim2);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7200;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.Prescaler = 720;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 500;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{

//
//	HAL_TIM_Base_Stop_IT(&htim2);

	if(htim->Instance==TIM2)	//	Si interrupcion proviene de TIM2 -> cambio de modo de display
	{
		if(az_filter.out > 0.8)
			display_mode = 2;

		if(az_filter.out < 0.2)
			display_mode = 1;

		HAL_TIM_Base_Stop_IT(&htim2);
	}

	if(htim->Instance==TIM3)	//	Si intrerrupcion proviene de TIM3 -> lectura de MPU6050
	{
		mpu6050_Get_Accel(&mpu6050);

		FIRFilter_Update(&az_filter, mpu6050.accel_z);

		angle_yx = -1*(atan2(mpu6050.accel_y,mpu6050.accel_x)*180)/PI;
		angle_xz = (atan2(mpu6050.accel_x,mpu6050.accel_z)*180)/PI;
		angle_yz = -1*(atan2(mpu6050.accel_y,mpu6050.accel_z)*180)/PI;

		angle_yx = FIRFilter_Update(&angle_yx_filter, angle_yx);
		angle_xz = FIRFilter_Update(&angle_xz_filter, angle_xz);
		angle_yz = FIRFilter_Update(&angle_yz_filter, angle_yz);
	}

}



//static void bubbleLevel_ArtifHorizon(int16_t angle)
//{
//	static uint16_t x1, y1, x2, y2;
//	static uint16_t radius = 31;
//	static uint16_t x0 = 96;
//	static uint16_t y0 = 32;
//	static char  MSG0[4] = "";
//	static char MSG1[2] = " +";
//
//	x2 = radius*cos((angle*PI)/180) + x0;
//	y2 = radius*sin((angle*PI)/180) + y0;
//	x1 = 2*x0 - x2;
//	y1 = 2*y0 - y2;
//
//	if(angle < 0)
//	{
//		MSG1[1] = '-';
//		angle *= (-1);
//	}
//	else
//		MSG1[1] = '+';
//
//	sprintf(MSG0, "%03d", angle);
//
//	ssd1306_Fill(Black);
//	//ssd1306_SetCursor(5, 19);
//	ssd1306_SetCursor(5, 2);
//	ssd1306_WriteString(MSG1, Font_16x26, White);
//	ssd1306_SetCursor(5, 34);
//	ssd1306_WriteString(MSG0, Font_16x26, White);
//	ssd1306_DrawCircle(x0, y0, radius, White);
//	ssd1306_Line(x1, y2, x2, y1, White);
//	ssd1306_UpdateScreen();
//}


static void bubbleLevel_1d(float angle)
{

	#define COLOR	1

	char  MSG0[7] = "";

	static uint8_t radius = 9;
	static int16_t x0;
	const uint8_t y0 = 52;
	const uint8_t x1 = 10;
	const uint8_t x2 = 120;

	//angle = -170;

	//x0 = ((angle + 180)/360.0)*(x2 - x1 - 2*(radius + 1)) + (x1 + radius +1);
	//x0 = (angle/60.0)*45.0 + 65;
	x0 = (angle/60.0)*(x2 - (radius+1) - (x2 + x1)/2) + (x2 + x1)/2;

	if( (x0 - (radius +1)) <= x1 )
		x0 = x1 + (radius + 1);
										//	SON NECESARIOS ESTOS LÍMITES?
	if( (x0 + (radius +1)) >= x2 )
		x0 = x2 - (radius + 1);

	sprintf(MSG0, "%+4.1f", angle);

	ssd1306_Fill(!COLOR);
	ssd1306_DrawRectangle(x1, y0 - (radius + 1), x2, y0 + (radius +1), COLOR);
	ssd1306_DrawCircle(x0, y0, radius, COLOR);
	ssd1306_DrawCircle(x0+3, y0-3, 2, COLOR);
	ssd1306_SetCursor(26, 8);
	ssd1306_WriteString(MSG0, Font_16x26, COLOR);
	ssd1306_UpdateScreen();

}

static void bubbleLevel_2d(float angle_xz, float angle_yz)
{

	#define COLOR 	1
	#define FONT	Font_7x10

	char  MSG0[7] = "";
	char  MSG1[7] = "";

	sprintf(MSG0, "%+4.1f", angle_xz);
	sprintf(MSG1, "%+4.1f", angle_yz);

	uint8_t x0 = 95;
	uint8_t y0 = 32;

//	x0 += angle_xz;
//	y0 += angle_yz;

	//	CONVERSION A COORDENADAS POLARES

	float radius;
	float theta;

	//radius = sqrt(pow(angle_yz, 2) + pow(angle_xz, 2));
	radius = sqrt(angle_yz*angle_yz + angle_xz*angle_xz);

	if(radius > 26)
		radius = 26;

	theta = atan2(angle_xz, angle_yz);

	x0 += radius * sin(theta);
	y0 -= radius * cos(theta);

	//	FIN CONVERSION A COORDENADAS POLARES

	ssd1306_Fill(!COLOR);
	ssd1306_DrawCircle(95, 32, 31, White);
	//ssd1306_DrawRectangle(63, 1, 126, 63, COLOR);	//	Descomentar en caso de no usar coordenadas polares
	ssd1306_Line(64, 32, 126, 32, COLOR);
	ssd1306_Line(95, 1, 95, 63, COLOR);
	ssd1306_DrawRectangle(89, 26, 101, 38, COLOR);
	ssd1306_DrawCircle(x0, y0, 5, COLOR);
	ssd1306_DrawCircle(x0+2, y0-2, 1, COLOR);
	ssd1306_SetCursor(1, 1);
	ssd1306_WriteString(MSG0, FONT, COLOR);
	ssd1306_SetCursor(1, 30);
	ssd1306_WriteString(MSG1, FONT, COLOR);
	ssd1306_UpdateScreen();

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
