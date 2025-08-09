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
#include <stdio.h> // Required for snprintf function
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// 1602 I2C address
#define I2C_ADDR 0x27 // I2C address
// 1602 dimensions
#define LCD_ROWS 2 	// Number of rows on the LCD
#define LCD_COLS 16 // Number of columns on the LCD
// 1602 message bit numbers
#define DC_BIT 0 // Data/Command bit (RS - Register Select bit)
#define EN_BIT 2 // Enable bit
#define BL_BIT 3 // Backlight bit
#define D4_BIT 4 // Data 4 bit (for 4-bit mode communication)
#define D5_BIT 5 // Data 5 bit (for 4-bit mode communication)
#define D6_BIT 6 // Data 6 bit (for 4-bit mode communication)
#define D7_BIT 7 // Data 7 bit (for 4-bit mode communication)

#define SAFE_DISTANCE 50 		// Distance in cm below which the buzzer and LED activate dynamically
#define ARR_MIN 10 				// Minimum Auto-Reload Register (ARR) value for TIM3, resulting in fastest toggle
#define ARR_MAX 4999 			// Maximum Auto-Reload Register (ARR) value for TIM3, resulting in slowest toggle
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */
volatile char trigger_flag = 0; 	// Flag set by TIM6 interrupt to trigger sensor measurement
volatile char echo_flag = 0; 		// Flag set by external interrupt (echo pin) to indicate echo received
volatile char update_lcd_flag = 0;  // Flag set to indicate LCD display needs updating
volatile uint32_t duration; 		// Stores the raw timer count for the echo pulse duration
volatile uint32_t distance_cm; 		// Stores the calculated distance in centimeters
volatile char buzzer_flag = 0; 		// (Not used in current implementation, buzzer/LED handled in TIM3 ISR)
uint32_t calc_arr; 					// Stores the calculated ARR value for dynamic buzzer/LED rate
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void CharLCD_Write_Nibble(uint8_t nibble, uint8_t dc); 	// Helper function to write a 4-bit nibble to LCD
void CharLCD_Send_Cmd(uint8_t cmd); 					// Sends a command byte to the LCD
void CharLCD_Send_Data(uint8_t data); 					// Sends a data (character) byte to the LCD
void CharLCD_Init(); 									// Initializes the LCD in 4-bit I2C mode
void CharLCD_Write_String(char *str); 					// Writes a null-terminated string to the LCD
void CharLCD_Set_Cursor(uint8_t row, uint8_t column);   // Sets the cursor position on the LCD
void CharLCD_Clear(void); 								// Clears the LCD display
void Trigger_Sensor(void); 								// Sends a trigger pulse to the ultrasonic sensor
void Delay_us(uint16_t us); 							// Provides a microsecond delay using TIM16
void Capture_Echo(void); 								// Measures the echo pulse duration and calculates distance
void Print_Distance(uint32_t distance); 				// Formats and prints distance to the LCD
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
  MX_I2C1_Init();
  MX_TIM16_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	CharLCD_Init(); 						// Initialize the LCD display
	CharLCD_Write_String("Initialized"); 	// Display "Initialized" on LCD upon startup

	HAL_TIM_Base_Start(&htim16); 	// Start Timer 16 (used for microsecond delays)
	HAL_TIM_Base_Start(&htim1);		// Start Timer 1 (used for measuring echo pulse length)
	HAL_TIM_Base_Start_IT(&htim3); 	// Start Timer 3 in interrupt mode (for periodic buzzer/LED toggling)
	HAL_TIM_Base_Start_IT(&htim6); 	// Start Timer 6 in interrupt mode (for periodic sensor triggering)

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		if (trigger_flag) { 															// Check if the sensor needs to be triggered (set by TIM6 ISR or button)
			Trigger_Sensor(); 															// Send the 10us trigger pulse
		}

		if (echo_flag) { 																// Check if an echo pulse has been received (set by echo pin EXTI ISR)
			Capture_Echo(); 															// Measure the echo pulse duration and calculate distance
		}

		if (update_lcd_flag) { 															// Check if the LCD display needs to be updated
			update_lcd_flag = 0; 														// Clear the flag
			if (distance_cm > 400) { 													// If distance is beyond sensor's typical max range (400cm)
				CharLCD_Set_Cursor(0, 0); 												// Set cursor to home position
				CharLCD_Write_String("Out of Range     "); 								// Display "Out of Range"
			} else {
				Print_Distance(distance_cm); 											// Display the measured distance on the LCD
			}
		}

		// Dynamic buzzer and LED rate control based on distance
		if (distance_cm < SAFE_DISTANCE) { // If distance is within the "unsafe" range
			// Calculate the new Auto-Reload Register (ARR) value for TIM3
			// As distance_cm decreases (gets closer to 0), calc_arr decreases, making the beep/blink faster.
			calc_arr = ARR_MIN + (ARR_MAX - ARR_MIN) * distance_cm / SAFE_DISTANCE;
			TIM3->ARR = calc_arr;														// Apply the new period to Timer 3
		} else { 																		// If distance is safe or out of range
			TIM3->ARR = 4999; 															// Set Timer 3 to its default slower period
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hi2c1.Init.Timing = 0x10D19CE4;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 79;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim3.Init.Prescaler = 7999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 7999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 4999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 79;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, trig_Pin|LED2_Pin|buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : trig_Pin LED2_Pin buzzer_Pin */
  GPIO_InitStruct.Pin = trig_Pin|LED2_Pin|buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : echo_Pin */
  GPIO_InitStruct.Pin = echo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(echo_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief Sends measured distance in cm to be printed on CharLCD
 * @param distance_cm: integer distance in centimeters
 * @retval None
 */
void Print_Distance(uint32_t distance_cm) {

	char msg[17]; 												 // Buffer to hold the formatted string (16 chars + null terminator)
	snprintf(msg, sizeof(msg), "Distance: %lu cm", distance_cm); // Format distance into string
	CharLCD_Clear(); 											 // Clear the entire LCD display
	CharLCD_Set_Cursor(0, 0); 									 // Set cursor to the beginning of the first line
	CharLCD_Write_String(msg); 									 // Write the formatted distance string to LCD
}

/**
 * @brief Sends 10us pulse to sensor, telling to take a measurement
 * @param None
 * @retval None
 */
void Trigger_Sensor() {

	trigger_flag = 0; 											 // Clear the trigger flag
	HAL_GPIO_WritePin(trig_GPIO_Port, trig_Pin, GPIO_PIN_SET); 	 // Set trigger pin HIGH
	Delay_us(10); 												 // Wait for 10 microseconds
	HAL_GPIO_WritePin(trig_GPIO_Port, trig_Pin, GPIO_PIN_RESET); // Set trigger pin LOW
}

/**
 * @brief Captures length of echo pulse from sensor and calculates distance
 * @param None
 * @retval None
 */
void Capture_Echo(void) {
	echo_flag = 0; 														// Clear the echo flag
	HAL_TIM_Base_Stop(&htim1);											// Stop Timer 1 to prepare for new measurement
	__HAL_TIM_SET_COUNTER(&htim1, 0); 									// Reset Timer 1's counter to 0
	HAL_TIM_Base_Start(&htim1); 										// Start Timer 1 to begin measuring the echo pulse
	while (HAL_GPIO_ReadPin(echo_GPIO_Port, echo_Pin) == GPIO_PIN_SET);	// Wait for the echo pin to go LOW (falling edge)
	duration = __HAL_TIM_GET_COUNTER(&htim1); 							// Get the counter value (duration of HIGH pulse)
	distance_cm = duration / 58; 										// Convert pulse duration to distance in centimeters (speed of sound factor)
	update_lcd_flag = 1; 												// Set flag to update the LCD with new distance
}

/**
 * @brief Implements a delay in microseconds using TIM16
 * @param us: integer value for delay in microseconds
 * @retval None
 */
void Delay_us(uint16_t us) {
	HAL_TIM_Base_Stop(&htim16); 					// Stop Timer 16 to prepare for delay
	__HAL_TIM_SET_COUNTER(&htim16, 0); 				// Reset Timer 16's counter to 0
	HAL_TIM_Base_Start(&htim16); 					// Start Timer 16
	while (__HAL_TIM_GET_COUNTER(&htim16) < us) {
		// Wait
	}
	HAL_TIM_Base_Stop(&htim16); 					// Stop Timer 16 after the delay
}

/**
 * @brief External Interrupt (EXTI) Callback function
 * @param GPIO_Pin: Pin that triggered the interrupt
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == B1_Pin) { 			// If the User Button (B1) was pushed
		trigger_flag = 1; 				// Set flag to trigger the ultrasonic sensor
	} else if (GPIO_Pin == echo_Pin) {  // If the Echo pin generated an interrupt
		echo_flag = 1; 					// Set flag to capture the echo pulse duration
	}
}

/**
 * @brief Timer Period Elapsed Callback function
 * @param htim: Pointer to the TIM handle that triggered the callback
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == htim6.Instance) { 									// If the callback is from Timer 6
		trigger_flag = 1; 														// Set flag to trigger the ultrasonic sensor
	}

	if (htim->Instance == htim3.Instance) { 									// If the callback is from Timer 3 (for buzzer/LED)
		if (distance_cm < SAFE_DISTANCE) { 										// If the measured distance is within the unsafe range
			HAL_GPIO_TogglePin(buzzer_GPIO_Port, buzzer_Pin); 					// Toggle buzzer state (beep)
			HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);						// Toggle LED2 state (blink warning)
		} else { 																// If the measured distance is safe or out of range
			HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, GPIO_PIN_RESET);	// Ensure buzzer is OFF
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET); 		// Ensure LED2 is OFF
		}
	}
}

/**
 * @brief Helper function to write a 4-bit nibble to the LCD via I2C
 * @param nibble: 4-bit data to send
 * @param dc: Data/Command select bit (0 for command, 1 for data)
 * @retval None
 */
void CharLCD_Write_Nibble(uint8_t nibble, uint8_t dc) {
	uint8_t data = nibble << D4_BIT; 									// Shift nibble to D4-D7 position for PCF8574
	data |= dc << DC_BIT; 												// Set Data/Command bit (RS)
	data |= 1 << BL_BIT; 												// Ensure backlight is ON
	data |= 1 << EN_BIT; 												// Set Enable bit HIGH (for latching data)
	HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDR << 1, &data, 1, 100); 		// Transmit data with EN high
	HAL_Delay(1);
	data &= ~(1 << EN_BIT); 											// Clear Enable bit (falling edge triggers LCD)
	HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDR << 1, &data, 1, 100); 		// Transmit data with EN low
}

/**
 * @brief Send command to LCD
 * @param cmd: 8-bit command to send to LCD controller
 * @retval None
 */
void CharLCD_Send_Cmd(uint8_t cmd) {
	uint8_t upper_nibble = cmd >> 4; 			// Extract upper 4 bits of the command
	uint8_t lower_nibble = cmd & 0x0F; 			// Extract lower 4 bits of the command
	CharLCD_Write_Nibble(upper_nibble, 0); 		// Send upper nibble as command (DC=0)
	CharLCD_Write_Nibble(lower_nibble, 0); 		// Send lower nibble as command (DC=0)
	if (cmd == 0x01 || cmd == 0x02) { 			// Special handling for Clear Display and Return Home commands
		HAL_Delay(2);
	}
}

/**
 * @brief Send data (character) to LCD
 * @param data: 8-bit character data to display
 * @retval None
 */
void CharLCD_Send_Data(uint8_t data) {
	uint8_t upper_nibble = data >> 4; 		// Extract upper 4 bits of the character
	uint8_t lower_nibble = data & 0x0F; 	// Extract lower 4 bits of the character
	CharLCD_Write_Nibble(upper_nibble, 1); 	// Send upper nibble as data (DC=1)
	CharLCD_Write_Nibble(lower_nibble, 1);	// Send lower nibble as data (DC=1)
}

/**
 * @brief Initialize LCD in 4-bit mode via I2C
 * @param None
 * @retval None
 */
void CharLCD_Init() {
	HAL_Delay(50);					 // Wait for LCD power-on reset (min 40ms)
	CharLCD_Write_Nibble(0x03, 0); 	 // Step 1 of 4-bit initialization (8-bit mode trial)
	HAL_Delay(5);
	CharLCD_Write_Nibble(0x03, 0); 	 // Step 2 of 4-bit initialization
	HAL_Delay(1);
	CharLCD_Write_Nibble(0x03, 0); 	 // Step 3 of 4-bit initialization
	HAL_Delay(1);
	CharLCD_Write_Nibble(0x02, 0); 	 // Final step: switch to 4-bit interface mode
	CharLCD_Send_Cmd(0x28); 		 // Function set: 4-bit interface, 2 lines, 5x8 dots font
	CharLCD_Send_Cmd(0x0C);		 	 // Display control: Display ON, Cursor OFF, Blink OFF
	CharLCD_Send_Cmd(0x06); 		 // Entry mode set: Increment cursor, no display shift
	CharLCD_Send_Cmd(0x01); 		 // Clear display command
	HAL_Delay(2);
}

/**
 * @brief Write string to LCD at current cursor position
 * @param str: Pointer to null-terminated string
 * @retval None
 */
void CharLCD_Write_String(char *str) {
	while (*str) { 					// Loop until null terminator is found
		CharLCD_Send_Data(*str++); 	// Send current character and advance pointer
	}
}

/**
 * @brief Set cursor position on LCD
 * @param row: Row number (0 for first line, 1 for second line)
 * @param column: Column number (0 to LCD_COLS-1)
 * @retval None
 */
void CharLCD_Set_Cursor(uint8_t row, uint8_t column) {
	uint8_t address;
	switch (row) {
	case 0:
		address = 0x00;
		break;
	case 1:
		address = 0x40;
		break;
	default:
		address = 0x00;
	}
	address += column;
	CharLCD_Send_Cmd(0x80 | address);
}

/**
 * @brief Clear LCD display and return cursor to home position
 * @param None
 * @retval None
 */
void CharLCD_Clear(void) {
	CharLCD_Send_Cmd(0x01); 	// Send Clear Display command
	HAL_Delay(2);
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
	__disable_irq(); // Disable global interrupts
	while (1) {
		// Infinite loop to halt execution in case of an error
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
