/* USER CODE BEGIN Header */
/*
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 * Author:	Kipling W. Dickie
 * Date:	10/06/2020
 *
 * ECET260 Lab05: adcDma
 *
 * Resources:
 * https://learn.adafruit.com/thermistor/using-a-thermistor
 * RM0090ReferenceManual.pdf
 * - DMA Controller	[P.302-314]
 * - Temperature Sensor [P.412-413]
 * STM32F407VGDatasheet.pdf
 * - Temperature Sensor Characteristics [P.138]
 *
 * Program Instructions:
 * - Configure ADC3 to read the thermistor. The thermistor needs to be read using DMA.
 * - Configure ADC1 to read the internal die temperature sensor. The internal die temperature needs to be read using DMA.
 * - Continuously read values of each into arrays of 100 samples and average them.
 * - Blink the orange LED at a 1/2 second rate.
 *
 * Hardware Specs:
 * - Thermistor Pin 1: Data Pin (PC1)
 * - Thermistor Pin 2: VCC
 * - Thermistor Pin 3: GND
 * - Thermistor Resistance @ Room Temp: ~100kohm
 * - Fixed Resistor Resistance: ~10kohm
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define samples 100		// Initialize and set # of samples that will be averaged.
#define VCC 3			// Initialize and set Voltage Source Value (3V).
#define resolution 4095	// Analog to Digital 12-bit resolution [(2^12)-1] = 4095.
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc3;

RNG_HandleTypeDef hrng;

/* USER CODE BEGIN PV */
uint16_t ADC1samples[samples];	// Initialize 16-Bit integer array: "ADC1sample[samples]". Internal Die Data. Set to 16-bit to match DMA data width.
uint16_t ADC3samples[samples];	// Initialize 16-Bit integer array: "ADC3sample[samples]". Thermistor Data. Set to 16-bit to match DMA data width.
uint8_t i;						// Initialize 8-Bit integer "i" for indexing.
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_RNG_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC3_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_DMA_Init();
  MX_RNG_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, ADC1samples, samples);	// Start ADC for address "&hadc1" and gets Internal Die data values to populate array "ADC1samples[samples]".
  HAL_ADC_Start_DMA(&hadc3, ADC3samples, samples);	// Start ADC for address "&hadc3" and gets Thermistor data value to populate array "ADC3samples[samples]".
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	HAL_GPIO_WritePin(OrangeLED_GPIO_Port, OrangeLED_Pin, GPIO_PIN_SET);	// OrangeLED ON
	HAL_Delay(250);	// Delay 250ms
	HAL_GPIO_WritePin(OrangeLED_GPIO_Port, OrangeLED_Pin, GPIO_PIN_RESET);	// OrangeLED OFF
	HAL_Delay(250);	// Delay 250ms

	// Average all the samples:
	float InternalDieData = 0;	// Initialize float "VInternalDie".
	float ThermistorData  = 0;	// Initialize float "VThermistor".
	for (i = 0; i < samples; i++) {
		InternalDieData += ADC1samples[i];	// Sum Internal Die data samples.
		ThermistorData  += ADC3samples[i];	// Sum Thermistor data samples.
	}
	InternalDieData /= samples;	// Average the Internal Die data samples.
	ThermistorData  /= samples;	// Average the Thermistor data samples.

	// Calculate Internal Die Temperature:
	float Avg_slope = 2.5;								// Average Slope (mV/degC)	[STM32F407VG_Datasheet.pdf P.138]
	float V_25 = 0.76;									// Voltage at 25 degC (V)	[STM32F407VG_Datasheet.pdf P.138]
	float Vsense = (VCC/resolution) * InternalDieData;	// Internal Die Voltage equivalent.

	float InternalDieTempC = ((Vsense-V_25)/Avg_slope) + 25;	// Internal Die Temperature in Celsius.
	float InternalDieTempK = InternalDieTempC + 273.15;			// Internal Die Temperature in Kelvin.
	float InternalDieTempF = ((9 / 5) * InternalDieTempC) + 32;	// Internal Die Temperature in Fahrenheit.

	// Print Results:
	printf("Internal Die Temperature Reading:\n");
	printf("%0.2f K\n", InternalDieTempK);	// Print Internal Die Temperature in Kelvin.
	printf("%0.2f C\n", InternalDieTempC);	// Print Internal Die Temperature in Celsius.
	printf("%0.2f F\n", InternalDieTempF);	// Print Internal Die Temperature in Fahrenheit.


	// Calculate Thermistor Temperature:
	float Rfixed = 10000;	// Fixed Resistor = 10kohm
	float RThermistor = (Rfixed * ThermistorData) / (resolution - ThermistorData); // Themistor Resistance Value.

	printf("Thermistor Resistance: %0.2f ohm\n", RThermistor);

	float RoomTempK = 298.15;	// Room Temperature = 298.15K
	int CoeffThermistor = 3950;	// Coefficient of Thermistor = 3950 [Datasheet]
	float RoomTempR = 100000;	// Thermistor Resistance @ Room Temperature = 100kohm

	float ThermistorTempK = 1/ (((log(RThermistor / RoomTempR)) / CoeffThermistor)+ (1.0 / RoomTempK));	// Thermistor Temperature in Kelvin.
	float ThermistorTempC = ThermistorTempK - 273.15;													// Thermistor Temperature in Celsius.
	float ThermistorTempF = (ThermistorTempC * (9 / 5)) + 32;											// Thermistor Temperature in Fahrenheit.

	// Print Results:
	printf("Thermistor Temperature Reading:\n");
	printf("%0.2f K\n", ThermistorTempK);	// Print Thermistor Temperature in Kelvin.
	printf("%0.2f C\n", ThermistorTempC);	// Print Thermistor Temperature in Celsius.
	printf("%0.2f F\n", ThermistorTempF);	// Print Thermistor Temperature in Fahrenheit.

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GreenLED_Pin|OrangeLED_Pin|RedLED_Pin|BlueLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BlueUserButton_Pin */
  GPIO_InitStruct.Pin = BlueUserButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BlueUserButton_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GreenLED_Pin OrangeLED_Pin RedLED_Pin BlueLED_Pin */
  GPIO_InitStruct.Pin = GreenLED_Pin|OrangeLED_Pin|RedLED_Pin|BlueLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	static int InternalDieCount = 0;
	if(hadc == &hadc1){ // if Internal Die Data:
		ADC1samples[InternalDieCount]= HAL_ADC_GetValue(hadc); // Populate ADC1samples[]
		InternalDieCount ++;
		InternalDieCount %= samples;
	}
	static int ThermistorCount = 0;
	if(hadc == &hadc3){ // if Thermistor Data:
		ADC3samples[ThermistorCount]= HAL_ADC_GetValue(hadc); // Populate ADC3samples[]
		ThermistorCount ++;
		ThermistorCount %= samples;
	}
}

int __io_putchar(int ch) {
	ITM_SendChar(ch);
	return 0;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
