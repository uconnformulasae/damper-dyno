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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MIN_VOLTAGE 0.0f
#define MAX_VOLTAGE 5.0
#define MAX_TRAVEL_MM          250
#define MAX_INT16_T_POSITIVE   32767.0f
#define MAX_INT16_T_NEGATIVE   32768.0f

#define ADS1115_6V             6.144f
#define ADS1115_250MV          0.256f
#define NUM_SAMPLES            16  // (Not used for averaging in this example)

#define LOAD_CELL_MV_V         3.2f
#define MAX_LOAD_CELL_WEIGHT   500.0f

#define DELAY_MS               250

// ADS1115 I2C address (7-bit)
#define ADS1115_ADDRESS        0b01001000
// ADS1115 default configuration (0x0EE3)
#define ADS1115_CONFIG         0b0000111011100011

// ADS1115 registers
#define CONVERSION_REGISTER    0b00000000
#define CONFIG_REGISTER        0b00000001
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static float last_linear_pot_voltage = 0.0f;
static float current_linear_pot_voltage = 0.0f;
static uint32_t last_linear_pot_readout_time = 0;
static uint32_t current_linear_pot_readout_time = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
int16_t read_ADS1115(uint8_t reg);
void write_ADS1115(uint8_t reg, uint16_t value);
float convert_to_voltage(int16_t adc_value, float reference_voltage);
float convert_to_displacement(float voltage);
float convert_to_weight(float voltage);
float calculate_linear_pot_velocity(void);
float read_load_cell(void);
float read_linear_pot(void);
float read_rotary_pot(void);
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
  char buffer[200];
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Read sensor voltages
	      float linear_pot_voltage = read_linear_pot();
	      float rotary_pot_voltage = read_rotary_pot();
	      float load_cell_voltage  = read_load_cell();

	      // Convert linear potentiometer voltage to displacement (in mm)
	      float displacement = convert_to_displacement(linear_pot_voltage);

	      // Calculate velocity in mm/s (based on successive readings)
	      float velocity = calculate_linear_pot_velocity();

	      // Format and output the results (ensure printf redirection is set up)
	      sprintf(buffer,
	              "Linear_Pot_Voltage: %.2f, Displacement: %.2f mm, Velocity: %.2f mm/s, Rotary_Pot_Voltage: %.4f, Load_Cell_Voltage: %.6f, Load_Cell_Weight: %.2f\r\n",
	              linear_pot_voltage,
	              displacement,
	              velocity,
	              rotary_pot_voltage,
	              load_cell_voltage,
	              convert_to_weight(load_cell_voltage));

	      HAL_UART_Transmit(&huart2, (uint8_t *) buffer, strlen(buffer), HAL_MAX_DELAY);

	      HAL_Delay(DELAY_MS);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Read two bytes from the specified ADS1115 register via I2C
int16_t read_ADS1115(uint8_t reg)
{
    uint8_t data[2];
    HAL_StatusTypeDef status;
    status = HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDRESS << 1, &reg, 1, HAL_MAX_DELAY);
    if(status != HAL_OK)
    {
        return 0;
    }
    status = HAL_I2C_Master_Receive(&hi2c1, ADS1115_ADDRESS << 1, data, 2, HAL_MAX_DELAY);
    if(status != HAL_OK)
    {
        return 0;
    }
    int16_t result = (int16_t)((data[0] << 8) | data[1]);
    return result;
}

// Write a 16-bit value to the specified ADS1115 register via I2C
void write_ADS1115(uint8_t reg, uint16_t value)
{
    uint8_t data[3];
    data[0] = reg;
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value & 0xFF);
    HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) ADS1115_ADDRESS << 1, data, 3, HAL_MAX_DELAY);
}

// Convert an ADC reading to a voltage
float convert_to_voltage(int16_t adc_value, float reference_voltage)
{
    if (adc_value >= 0)
        return ((float)adc_value / MAX_INT16_T_POSITIVE) * reference_voltage;
    else
        return ((float)adc_value / MAX_INT16_T_NEGATIVE) * reference_voltage;
}

// Convert a voltage reading to a displacement (mm)
float convert_to_displacement(float voltage)
{
    if (voltage < MIN_VOLTAGE)
        voltage = MIN_VOLTAGE;
    else if (voltage > MAX_VOLTAGE)
        voltage = MAX_VOLTAGE;
    return ((voltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE)) * MAX_TRAVEL_MM;
}

// Convert a load cell voltage to weight
float convert_to_weight(float voltage)
{
    float max_meter_voltage = (LOAD_CELL_MV_V * MAX_VOLTAGE) / 1000.0f;
    return -(voltage / max_meter_voltage) * MAX_LOAD_CELL_WEIGHT;
}

// Calculate velocity (mm/s) from change in displacement over time
float calculate_linear_pot_velocity(void)
{
    if (current_linear_pot_readout_time == last_linear_pot_readout_time)
        return 0.0f;
    float y2 = convert_to_displacement(current_linear_pot_voltage);
    float y1 = convert_to_displacement(last_linear_pot_voltage);
    float deltaTime = (current_linear_pot_readout_time - last_linear_pot_readout_time) / 1000.0f;
    float velocity = (y2 - y1) / deltaTime;
    last_linear_pot_readout_time = current_linear_pot_readout_time;
    last_linear_pot_voltage = current_linear_pot_voltage;
    return velocity;
}

// Read the load cell voltage using ADS1115 with modified config bits
float read_load_cell(void)
{
    const uint16_t mask = ~(0b0111111000000000);
    const uint16_t new_bits = 0b0000111000000000;
    uint16_t config = (ADS1115_CONFIG & mask) | new_bits;
    write_ADS1115(CONFIG_REGISTER, config);
    HAL_Delay(5);
    int16_t readout = read_ADS1115(CONVERSION_REGISTER);
    return convert_to_voltage(readout, ADS1115_250MV);
}

// Read the linear potentiometer voltage
float read_linear_pot(void)
{
    const uint16_t mask = ~(0b0111111000000000);
    const uint16_t new_bits = 0b0110000000000000;
    uint16_t config = (ADS1115_CONFIG & mask) | new_bits;
    write_ADS1115(CONFIG_REGISTER, config);
    HAL_Delay(5);
    int16_t readout = read_ADS1115(CONVERSION_REGISTER);
    current_linear_pot_readout_time = HAL_GetTick();
    current_linear_pot_voltage = convert_to_voltage(readout, ADS1115_6V);
    return current_linear_pot_voltage;
}

// Read the rotary potentiometer voltage
float read_rotary_pot(void)
{
    const uint16_t mask = ~(0b0111111000000000);
    const uint16_t new_bits = 0b0111000000000000;
    uint16_t config = (ADS1115_CONFIG & mask) | new_bits;
    write_ADS1115(CONFIG_REGISTER, config);
    HAL_Delay(5);
    int16_t readout = read_ADS1115(CONVERSION_REGISTER);
    return convert_to_voltage(readout, ADS1115_6V);
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
