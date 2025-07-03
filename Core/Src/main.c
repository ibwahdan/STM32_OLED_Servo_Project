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
#include <string.h>
#include <stdint.h>
#include "stm32f4xx_hal_tim.h"


void handle_button_press(void); //toggles PA5 and sends button press message
void send_uart_message(char *msg);
void toggle_led(void);

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define SH1107_ADDR 0x3C //OLED I2C Address
/*
#define ROW1_PIN GPIO_PIN_0
#define ROW2_PIN GPIO_PIN_1
#define ROW3_PIN GPIO_PIN_4
#define ROW4_PIN GPIO_PIN_0

#define ROW1_PORT GPIOA
#define ROW2_PORT GPIOA
#define ROW3_PORT GPIOA
#define ROW4_PORT GPIOB

#define COL1_PIN GPIO_PIN_7
#define COL2_PIN GPIO_PIN_6
#define COL3_PIN GPIO_PIN_1
#define COL4_PIN GPIO_PIN_0

#define COL1_PORT GPIOA
#define COL2_PORT GPIOB
#define COL3_PORT GPIOC
#define COL4_PORT GPIOC
*/


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void toggle_led(void) { //Activate/Toggle LED
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}

void send_uart_message(char *msg) { //Send Message through UART
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

void handle_button_press(void) { //Button Press actions
    toggle_led();
    HAL_Delay(200); // debounce
}


/* Buzzer PWM Functions */
void buzzer_on(void) {
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 250); //Duty Cycle - 50%
}

void buzzer_off(void) {
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0); //Duty Cycle - 0% (off)
}


/* Keypad Test Scan Functions */
/*
void check_keypad_and_beep() {
    GPIO_TypeDef* row_ports[4] = {GPIOA, GPIOA, GPIOA, GPIOB};
    uint16_t row_pins[4] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_4, GPIO_PIN_0};

    GPIO_TypeDef* col_ports[4] = {GPIOA, GPIOB, GPIOC, GPIOC};
    uint16_t col_pins[4] = {GPIO_PIN_7, GPIO_PIN_6, GPIO_PIN_1, GPIO_PIN_0};
    char msg[] = "Key detected\r\n";
    for (int r = 0; r < 4; r++) {
        // Set all rows high
        for (int i = 0; i < 4; i++) {
            HAL_GPIO_WritePin(row_ports[i], row_pins[i], GPIO_PIN_SET);
        }

        // Set current row low
        HAL_GPIO_WritePin(row_ports[r], row_pins[r], GPIO_PIN_RESET);
        HAL_Delay(1);  // debounce delay

        for (int c = 0; c < 4; c++) {
            if (HAL_GPIO_ReadPin(col_ports[c], col_pins[c]) == GPIO_PIN_RESET) {
                HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
                HAL_Delay(100);
                HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
                HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
                return;
            }
        }
    }
}*/

/* OLED Command Functions */
// Function to send a command to the OLED
void SH1107_Write_Command(uint8_t cmd) {
    uint8_t buffer[2] = {0x00, cmd}; // 0x00 = command
    HAL_I2C_Master_Transmit(&hi2c1, SH1107_ADDR << 1, buffer, 2, HAL_MAX_DELAY);
}

void SH1107_Write_Data(uint8_t data) {
    uint8_t buffer[2] = {0x40, data}; // 0x40 = data
    HAL_I2C_Master_Transmit(&hi2c1, SH1107_ADDR << 1, buffer, 2, HAL_MAX_DELAY);
}

void SH1107_Init(void) {
    HAL_Delay(100);

    SH1107_Write_Command(0xAE); // Display OFF
    SH1107_Write_Command(0xDC); // Set display start line
    SH1107_Write_Command(0x00);
    SH1107_Write_Command(0x81); // Set contrast control
    SH1107_Write_Command(0x2F);
    SH1107_Write_Command(0x20); // Set memory addressing mode
    SH1107_Write_Command(0xA0); // Segment remap
    SH1107_Write_Command(0xC0); // COM scan direction normal
    SH1107_Write_Command(0xA8); // Set multiplex ratio
    SH1107_Write_Command(0x7F); // 128MUX
    SH1107_Write_Command(0xD3); // Display offset
    SH1107_Write_Command(0x60); // Suggested for 128x128
    SH1107_Write_Command(0xD5); // Display clock divide
    SH1107_Write_Command(0x50);
    SH1107_Write_Command(0xD9); // Set pre-charge
    SH1107_Write_Command(0x22);
    SH1107_Write_Command(0xDB); // Set VCOMH Deselect Level
    SH1107_Write_Command(0x35);
    SH1107_Write_Command(0xAD); // Charge Pump
    SH1107_Write_Command(0x8A);
    SH1107_Write_Command(0xAF); // Display ON
}


void SH1107_Clear(void) {
    for (int page = 0; page < 8; page++) {
        SH1107_Write_Command(0xB0 + page); // Page address
        SH1107_Write_Command(0x00);        // Lower column start
        SH1107_Write_Command(0x10);        // Upper column start
        for (int col = 0; col < 128; col++) {
            SH1107_Write_Data(0x00);       // Blank
        }
    }
}

const uint8_t font5x7[][5] = {
    // SPACE (ASCII 32)
    [0] = {0x00, 0x00, 0x00, 0x00, 0x00}, // ' '
    ['!' - 32] = {0x00, 0x00, 0x5F, 0x00, 0x00}, // !
    ['"' - 32] = {0x00, 0x07, 0x00, 0x07, 0x00}, // "
    ['#' - 32] = {0x14, 0x7F, 0x14, 0x7F, 0x14}, // #
    ['$' - 32] = {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // $
    ['%' - 32] = {0x23, 0x13, 0x08, 0x64, 0x62}, // %
    ['&' - 32] = {0x36, 0x49, 0x55, 0x22, 0x50}, // &
    ['\'' - 32] = {0x00, 0x05, 0x03, 0x00, 0x00}, // '
    ['(' - 32] = {0x00, 0x1C, 0x22, 0x41, 0x00}, // (
    [')' - 32] = {0x00, 0x41, 0x22, 0x1C, 0x00}, // )
    ['*' - 32] = {0x14, 0x08, 0x3E, 0x08, 0x14}, // *
    ['+' - 32] = {0x08, 0x08, 0x3E, 0x08, 0x08}, // +
    [',' - 32] = {0x00, 0x50, 0x30, 0x00, 0x00}, // ,
    ['-' - 32] = {0x08, 0x08, 0x08, 0x08, 0x08}, // -
    ['.' - 32] = {0x00, 0x60, 0x60, 0x00, 0x00}, // .
    ['/' - 32] = {0x20, 0x10, 0x08, 0x04, 0x02}, // /

    // A–Z
    ['A' - 32] = {0x7E, 0x11, 0x11, 0x11, 0x7E},
    ['B' - 32] = {0x7F, 0x49, 0x49, 0x49, 0x36},
    ['C' - 32] = {0x3E, 0x41, 0x41, 0x41, 0x22},
    ['D' - 32] = {0x7F, 0x41, 0x41, 0x22, 0x1C},
    ['E' - 32] = {0x7F, 0x49, 0x49, 0x49, 0x41},
    ['F' - 32] = {0x7F, 0x09, 0x09, 0x09, 0x01},
    ['G' - 32] = {0x3E, 0x41, 0x49, 0x49, 0x7A},
    ['H' - 32] = {0x7F, 0x08, 0x08, 0x08, 0x7F},
    ['I' - 32] = {0x00, 0x41, 0x7F, 0x41, 0x00},
    ['J' - 32] = {0x20, 0x40, 0x41, 0x3F, 0x01},
    ['K' - 32] = {0x7F, 0x08, 0x14, 0x22, 0x41},
    ['L' - 32] = {0x7F, 0x40, 0x40, 0x40, 0x40},
    ['M' - 32] = {0x7F, 0x02, 0x0C, 0x02, 0x7F},
    ['N' - 32] = {0x7F, 0x04, 0x08, 0x10, 0x7F},
    ['O' - 32] = {0x3E, 0x41, 0x41, 0x41, 0x3E},
    ['P' - 32] = {0x7F, 0x09, 0x09, 0x09, 0x06},
    ['Q' - 32] = {0x3E, 0x41, 0x51, 0x21, 0x5E},
    ['R' - 32] = {0x7F, 0x09, 0x19, 0x29, 0x46},
    ['S' - 32] = {0x46, 0x49, 0x49, 0x49, 0x31},
    ['T' - 32] = {0x01, 0x01, 0x7F, 0x01, 0x01},
    ['U' - 32] = {0x3F, 0x40, 0x40, 0x40, 0x3F},
    ['V' - 32] = {0x1F, 0x20, 0x40, 0x20, 0x1F},
    ['W' - 32] = {0x7F, 0x20, 0x18, 0x20, 0x7F},
    ['X' - 32] = {0x63, 0x14, 0x08, 0x14, 0x63},
    ['Y' - 32] = {0x07, 0x08, 0x70, 0x08, 0x07},
    ['Z' - 32] = {0x61, 0x51, 0x49, 0x45, 0x43},
};



void SH1107_Set_Cursor(uint8_t page, uint8_t col) {
    SH1107_Write_Command(0xB0 + page);             // Set page address
    SH1107_Write_Command(col & 0x0F);              // Set lower column address
    SH1107_Write_Command(0x10 | (col >> 4));       // Set higher column address
}

void SH1107_Write_Char(char c) {
    if (c < 32 || c > 126) c = ' '; // fallback to space
    const uint8_t *bitmap = font5x7[c - 32];
    for (int i = 0; i < 5; i++) {
        SH1107_Write_Data(bitmap[i]);
    }
    SH1107_Write_Data(0x00); // Space between characters
}

void SH1107_Write_String(const char* str) {
    while (*str) {
        SH1107_Write_Char(*str++);
    }
}

void SH1107_Unlock_Animation(void) {
    for (int page = 0; page < 8; page++) {  // SH1107 usually has 8 pages (each 8px tall)
        SH1107_Set_Cursor(0, page);
        for (int col = 0; col < 128; col++) {
            SH1107_Write_Data(0xFF);  // Fill this line
            HAL_Delay(1);             // Delay for visual effect
        }
    }
    HAL_Delay(500);
    SH1107_Clear();
}


uint32_t  buttonCount = 0;

/* Servo Functions*/
void servo_set_angle(uint8_t angle) {
	 if (angle > 180) angle = 180;
	 uint16_t pulse = ((angle * 1000) / 180) + 1000;  // maps 0–180° to 1000–2000
	 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse);
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
  HAL_Delay(500);
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // Start PWM TIM3_CH1
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // Start PWM TIM1_CH2


/*  for (uint8_t addr = 0x3C; addr <= 0x3F; addr++) {
      if (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 3, 100) == HAL_OK) {
          char buf[32];
          sprintf(buf, "I2C device found at 0x%02X\r\n", addr);
          send_uart_message(buf);
      } else {
          char buf[32];
          sprintf(buf, "No device at 0x%02X\r\n", addr);
          send_uart_message(buf);
      }
  } */

  SH1107_Init();
  SH1107_Clear();


  // Fill first page with solid pixels
/*  for (uint8_t page = 0; page < 8; page++) {
      SH1107_Write_Command(0xB0 + page); // Set page address
      SH1107_Write_Command(0x02);        // Set lower column address
      SH1107_Write_Command(0x10);        // Set higher column address

      for (uint8_t col = 0; col < 128; col++) {
          SH1107_Write_Data(0xFF);       // White pixels
      }
  }*/



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  	  /* Toggle LED Pin if button is pressed  */
	  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
		  handle_button_press();
		  servo_set_angle(0);    // Far left
		  SH1107_Clear();
		  SH1107_Set_Cursor(0, 0); //page 0 column 0
		  buttonCount++;
		  if(buttonCount >= 5 && buttonCount < 8) {
			  SH1107_Unlock_Animation();
			  SH1107_Set_Cursor(0, 0);
			  SH1107_Write_String("UNLOCKED!");
			  send_uart_message("Unlocked Device!\r\n");
			  servo_set_angle(90);   // Center
			  for(int i = 0; i < 5; i++) {
				  buzzer_on();
				  HAL_Delay(100);
				  buzzer_off();
				  HAL_Delay(100);
				  servo_set_angle(20);
				  HAL_Delay(100);
			  }
		  } else {
			  buzzer_on();
			  HAL_Delay(100);
			  buzzer_off();
			  send_uart_message("Device Locked!\r\n");
			  SH1107_Write_String("LOCKED!");
			  servo_set_angle(180);  // Far right

		  }

	  } else {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		  buzzer_off();
	  }

	  if(buttonCount >= 8) {buttonCount = 0;}




	 // check_keypad_and_beep();
	 // HAL_Delay(50);


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 500-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 250;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
