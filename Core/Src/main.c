/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <stdio.h>
#include <stdbool.h>
#include "OLED_1602.h"
#include "Key.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define POLLING_TIME    10 //keys checked every 10ms
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
//Keys declaration
key_t preload_key, start_key, return_key, stop_key, lock_key, window_key;

bool f_polling; //true if polling counter is equal to polling period
uint32_t polling_cnt; //polling counter
uint32_t polling_period; //10ms if SysTick configured for 1ms tick interrupt

char oled_str[LCD_STR_LEN];
bool f_update_str1, f_update_str2;
uint8_t menu_index;
uint8_t max_index;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

//Keys handlers and initialize methods
static void Keys_Init(void);
static void Keys_Polling(void);

void Preload_Key_Handler(key_state_e);
void Start_Key_Handler(key_state_e);
void Return_Key_Handler(key_state_e);
void Stop_Key_Handler(key_state_e);
void Lock_Key_Handler(key_state_e);
void Window_Key_Handler(key_state_e);

void OLED_Str1_Handler(char*, uint8_t);
void OLED_Str2_Handler(char*);

extern void HAL_TIM_PeriodElapsedCallback_Modbus(TIM_HandleTypeDef*);
extern void HAL_UART_RxCpltCallback_modbus(UART_HandleTypeDef*);
extern void HAL_UART_TxCpltCallback_modbus(UART_HandleTypeDef *huart);

static void StatusRegister_Handler(uint32_t);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//ModbusRTU variables
//
//Modbus holding registers and registers array
uint16_t pressure_M1, pressure_M2, pressure_M3, pressure_M4, accum_voltage, tempr_compr_RT1, error_code;

uint16_t *modbus_holding_regs[] = {&pressure_M1, //M1, main receiver 0.0 - 25.0Bar
                                   &pressure_M2, //M2, control receiver 0.0 - 25.0Bar
                                   &pressure_M3, //M3, pressure in the rod cavity area of the working cylinder xx.xBar
                                   &pressure_M4, //M4, working cylinder pressure 0.0-25.0Bar
                                   &accum_voltage, //Compressor accumulator voltage 0.0-16.0V
                                   &tempr_compr_RT1, //Compressor body temperature 0-120C
                                   &error_code}; //Error code E00-E99 (E00 - no error)

//Modbus coils - status register
uint8_t status_register;

#define MANUAL_MODE_BIT     0x01 // Mode for testing the system
#define START_MODE_BIT      0x02 // Mode for running the system up
#define FAILURE_BIT         0x04 // Emergency system mode
#define CHOCK_SET_BIT       0x08 // To set if the chock set in the carriage lock

//Modbus contacts - control register
uint8_t control_register;

#define PRELOAD_BIT         0x01 // To supply a low pressure to the main cylinder
#define START_BIT           0x02 // To supply a full pressure to the main cylinder
#define RETURN_BIT          0x04 // To release the pressure in the main cylinder and supply a low pressure to the rod cavity area for return the system to the initial state
#define STOP_BIT            0x08 // To set all valves to the safe state with fixation
#define LOCK_BIT            0x10 // To lock the remote control all operations

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
  f_polling =  false;
  polling_cnt = 0;
  polling_period = POLLING_TIME/uwTickFreq;

  pressure_M1 = pressure_M2 = pressure_M3 = pressure_M4 = accum_voltage = tempr_compr_RT1 = error_code = 0x00;
  status_register = 0x00;
  control_register = 0x00;

  f_update_str1 = f_update_str2 = true; //first time OLED display should be updated
  menu_index = 0;
  max_index = sizeof(modbus_holding_regs) / 4 - 1;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_LockPin(OSC_En_GPIO_Port, OSC_En_Pin); //LOCK this pin settings because they are important - this pin enables HSE generator
  HAL_GPIO_LockPin(EN_5EXT_GPIO_Port, EN_5EXT_Pin); //LOCK this pin - this pin enables 5V circuit

  Keys_Init(); //Initialize keys

  OLED_Init();       //Initialize OLED display
  OLED_Clear();      //Clear display
  OLED_HideCursor(); //Hide cursor

  //Initialize ModbusRTU module and enable it
  __disable_irq();
  MT_PORT_SetTimerModule(&htim3);
  MT_PORT_SetUartModule(&huart1); //use uart1 for debug purposes
  eMBErrorCode eStatus;
  eStatus = eMBInit(MB_RTU, MODBUS_SLAVE_ADDRESS, 0, huart1.Init.BaudRate, MB_PAR_NONE);
  eStatus = eMBEnable();
  if (eStatus != MB_ENOERR)
  {
      // Error handling
  }
  __enable_irq();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
      eMBPoll(); //Modbus polling function

      //Here checked all keys, set all LEDs, etc
      if (f_polling == true)
      {
          f_polling = false;

          Keys_Polling();

          if(f_update_str1 == true)
          {
              f_update_str1 = false;
              OLED_Str1_Handler(oled_str, menu_index);
              OLED_GoTo(0x00);
              OLED_PutStr(oled_str);
          }

          if(f_update_str2 == true)
          {
              f_update_str2 = false;
              OLED_Str2_Handler(oled_str);
              OLED_GoTo(0x40);
              OLED_PutStr(oled_str);
          }

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
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
  htim3.Init.Prescaler = 25;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 50;
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
  huart1.Init.BaudRate = 115200;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EN_5EXT_GPIO_Port, EN_5EXT_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OSC_En_GPIO_Port, OSC_En_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, OLED_RS_Pin|OLED_DB7_Pin|OLED_DB5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OLED_RW_GPIO_Port, OLED_RW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OLED_E_Pin|OLED_DB4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OLED_DB6_Pin|LedOut_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : STOP_KEY_Pin RETURN_KEY_Pin LOCK_KEY_Pin */
  GPIO_InitStruct.Pin = STOP_KEY_Pin|RETURN_KEY_Pin|LOCK_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : START_KEY_Pin PRELOAD_KEY_Pin */
  GPIO_InitStruct.Pin = START_KEY_Pin|PRELOAD_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : EN_5EXT_Pin */
  GPIO_InitStruct.Pin = EN_5EXT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EN_5EXT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OSC_En_Pin */
  GPIO_InitStruct.Pin = OSC_En_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OSC_En_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OLED_RS_Pin OLED_DB7_Pin OLED_DB5_Pin */
  GPIO_InitStruct.Pin = OLED_RS_Pin|OLED_DB7_Pin|OLED_DB5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : OLED_RW_Pin */
  GPIO_InitStruct.Pin = OLED_RW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(OLED_RW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OLED_E_Pin OLED_DB4_Pin */
  GPIO_InitStruct.Pin = OLED_E_Pin|OLED_DB4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : WINDOW_KEY_Pin */
  GPIO_InitStruct.Pin = WINDOW_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(WINDOW_KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OLED_DB6_Pin */
  GPIO_InitStruct.Pin = OLED_DB6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(OLED_DB6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LedOut_Pin */
  GPIO_InitStruct.Pin = LedOut_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LedOut_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_SYSTICK_Callback(void)
{
    if(++polling_cnt == polling_period)
    {
        f_polling = true;
        polling_cnt = 0;
    }
}

/**
  * @brief UART RX complete callback
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    HAL_UART_RxCpltCallback_modbus(huart);
}

/**
  * @brief UART TX complete callback
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    HAL_UART_TxCpltCallback_modbus(huart);
}

/**
  * @brief Timer tim10 elapsed period callback
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        HAL_TIM_PeriodElapsedCallback_Modbus(htim);
    }
}

/**
  * @brief Initialize keys structures
  * @param None
  * @retval None
  */
static void Keys_Init(void)
{
    Key_Init(&preload_key, PRELOAD_KEY_GPIO_Port, PRELOAD_KEY_Pin, LO_LEVEL, &Preload_Key_Handler);
    Key_Init(&start_key  , START_KEY_GPIO_Port  , START_KEY_Pin  , LO_LEVEL, &Start_Key_Handler);
    Key_Init(&return_key , RETURN_KEY_GPIO_Port , RETURN_KEY_Pin , LO_LEVEL, &Return_Key_Handler);
    Key_Init(&stop_key   , STOP_KEY_GPIO_Port   , STOP_KEY_Pin   , LO_LEVEL, &Stop_Key_Handler);
    Key_Init(&lock_key   , LOCK_KEY_GPIO_Port   , LOCK_KEY_Pin   , LO_LEVEL, &Lock_Key_Handler);
    Key_Init(&window_key , WINDOW_KEY_GPIO_Port , WINDOW_KEY_Pin , LO_LEVEL, &Window_Key_Handler);
}

/**
  * @brief Check all keys state in polling
  * @param None
  * @retval None
  */
static void Keys_Polling(void)
{
    Key_CheckState(&preload_key);
    Key_CheckState(&start_key);
    Key_CheckState(&return_key);
    Key_CheckState(&stop_key);
    Key_CheckState(&lock_key);
    Key_CheckState(&window_key);
}

/**
  * @brief Preload key handler
  * @param state: state of the key - pressed/released
  * @retval None
  */
void Preload_Key_Handler(key_state_e state)
{
    if (state == PRESSED)
    {
        SET_BIT(control_register, PRELOAD_BIT);
    }
}

/**
  * @brief Start key handler
  * @param state: state of the key - pressed/released
  * @retval None
  */
void Start_Key_Handler(key_state_e state)
{
    if (state == PRESSED)
    {
        SET_BIT(control_register, START_BIT);
    }
}

/**
  * @brief Return key handler
  * @param state: state of the key - pressed/released
  * @retval None
  */
void Return_Key_Handler(key_state_e state)
{
    if (state == PRESSED)
    {
        SET_BIT(control_register, RETURN_BIT);
    }
}

/**
  * @brief Stop key handler
  * @param state: state of the key - pressed/released
  * @retval None
  */
void Stop_Key_Handler(key_state_e state)
{
    if (state == PRESSED)
    {
        SET_BIT(control_register, STOP_BIT);
    }
}

/**
  * @brief Lock key handler
  * @param state: state of the key - pressed/released
  * @retval None
  */
void Lock_Key_Handler(key_state_e state)
{
    if (state == PRESSED)
    {
        SET_BIT(control_register, LOCK_BIT);
    }
}

/**
  * @brief Window key handler
  * @param state: state of the key - pressed/released
  * @retval None
  */
void Window_Key_Handler(key_state_e state)
{
    if (state == PRESSED)
    {
        if (++menu_index == max_index)
        {
            menu_index = 0;
        }

        f_update_str1 = true;
    }
}

/**
  * @brief OLED string-1 handler
  * @param p_str: pointer to string
  * @retval None
  */
void OLED_Str1_Handler(char* p_str, uint8_t index)
{
    switch (index)
    {
        case 0:
            sprintf(oled_str, "M1Pres=%2d.%01dBar ", pressure_M1 / 10, pressure_M1 % 10);
            break;

        case 1:
            sprintf(oled_str, "M2Pres=%2d.%01dBar ", pressure_M2 / 10, pressure_M2 % 10);
            break;

        case 2:
            sprintf(oled_str, "M3Pres=%2d.%01dBar ", pressure_M3 / 10, pressure_M3 % 10);
            break;

        case 3:
            sprintf(oled_str, "M4Pres=%2d.%01dBar ", pressure_M4 / 10, pressure_M4 % 10);
            break;

        case 4:
            sprintf(oled_str, "Voltage=%2d.%01dV  ", accum_voltage / 10, accum_voltage % 10);
            break;

        case 5:
            sprintf(oled_str, "RT1Temp=%3dC  ", tempr_compr_RT1);
            break;

        default:

    }
}

/**
  * @brief OLED string-2 handler
  * @param p_str: pointer to string
  * @retval None
  */
void OLED_Str2_Handler(char*)
{
    sprintf(oled_str, "Status-E%02d     ", error_code);
}

/**
  * @brief
  * @retval eMBErrorCode
  */
eMBErrorCode eMBRegInputCB(uint8_t *pucRegBuffer, uint16_t usAddress, uint16_t usNRegs)
{
    eMBErrorCode eStatus = MB_ENOERR;
    uint32_t iRegIndex;

    if ((usAddress >= REG_INPUT_START) &&
        (usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS))
    {
        iRegIndex = (uint32_t)(usAddress - REG_INPUT_START);
        while(usNRegs > 0)
        {
            *pucRegBuffer++ = *modbus_holding_regs[iRegIndex] >> 8;
            *pucRegBuffer++ = *modbus_holding_regs[iRegIndex] & 0xFF;
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

/**
  * @brief
  * @retval eMBErrorCode
  */
eMBErrorCode eMBRegHoldingCB(uint8_t * pucRegBuffer, uint16_t usAddress, uint16_t usNRegs, eMBRegisterMode eMode)
{
    eMBErrorCode eStatus = MB_ENOERR;
    uint32_t iRegIndex;

    if ( (usAddress >= REG_HOLDING_START) && (usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS) )
    {
        iRegIndex = (uint32_t)(usAddress - REG_HOLDING_START);
        switch (eMode)
        {
            /* Pass current register values to the protocol stack. */
            case MB_REG_READ:
                while (usNRegs > 0)
                {
                    *pucRegBuffer++ = *modbus_holding_regs[iRegIndex] >> 8;
                    *pucRegBuffer++ = *modbus_holding_regs[iRegIndex] & 0xFF;
                    iRegIndex++;
                    usNRegs--;
                }
                break;

                /* Update current register values with new values from the
                 * protocol stack. */
            case MB_REG_WRITE:
                while (usNRegs > 0)
                {
                    uint16_t old_data = *modbus_holding_regs[iRegIndex];

                    *modbus_holding_regs[iRegIndex] = *pucRegBuffer++ << 8;
                    *modbus_holding_regs[iRegIndex] |= *pucRegBuffer++;

                    //The Update OLED display flag set if currently displayed data were changed
                    if (iRegIndex == menu_index && old_data != *modbus_holding_regs[iRegIndex])
                    {
                        f_update_str1 = true;
                    }else if (iRegIndex == max_index && old_data != *modbus_holding_regs[iRegIndex])
                    {
                        f_update_str2 = true;
                    }

                    iRegIndex++;
                    usNRegs--;
                }
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

/**
  * @brief
  * @retval eMBErrorCode
  */
eMBErrorCode eMBRegCoilsCB(uint8_t * pucRegBuffer, uint16_t usAddress, uint16_t usNCoils, eMBRegisterMode eMode)
{
    eMBErrorCode eStatus = MB_ENOERR;
    uint32_t iBitIndex;

    if ( (usAddress >= REG_COILS_START) && (usAddress + usNCoils <= REG_COILS_START + REG_COILS_NREGS) )
    {
        iBitIndex = (uint32_t)(usAddress - REG_COILS_START);

        switch (eMode)
        {
            case MB_REG_READ:
            {
                while (usNCoils > 0)
                {
                    UCHAR ucResult = xMBUtilGetBits(&status_register, iBitIndex, 1);
                    xMBUtilSetBits(pucRegBuffer, iBitIndex - (usAddress - REG_COILS_START), 1, ucResult);
                    iBitIndex++;
                    usNCoils--;
                }
                break;
            }
            case MB_REG_WRITE:
            {
                while (usNCoils > 0)
                {
                    UCHAR ucResult = xMBUtilGetBits(pucRegBuffer, iBitIndex - (usAddress - REG_COILS_START), 1);
                    xMBUtilSetBits(&status_register, iBitIndex, 1, ucResult );

                    StatusRegister_Handler(iBitIndex); //Status register bits handler

                    iBitIndex++;
                    usNCoils--;
                }
                break;
            }
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;

}

/**
  * @brief
  * @retval eMBErrorCode
  */
eMBErrorCode eMBRegDiscreteCB(uint8_t * pucRegBuffer, uint16_t usAddress, uint16_t usNDiscrete)
{
    eMBErrorCode eStatus = MB_ENOERR;
    uint32_t iBitIndex;

    if ( (usAddress >= REG_DISCRETE_COILS_START) && (usAddress + usNDiscrete <= REG_DISCRETE_COILS_START + REG_DISCRETE_NREGS) )
    {
        iBitIndex = (uint32_t)(usAddress - REG_DISCRETE_COILS_START);

         while (usNDiscrete > 0)
         {
             UCHAR ucResult = xMBUtilGetBits(&control_register, iBitIndex, 1);
             xMBUtilSetBits(pucRegBuffer, iBitIndex - (usAddress - REG_DISCRETE_COILS_START), 1, ucResult);

             //Some bits should be reseted after they were set as it requested in specification
             if ( (1 << iBitIndex) == PRELOAD_BIT)
             {
                 CLEAR_BIT(control_register, PRELOAD_BIT);
             }
             else if ( (1 << iBitIndex) == START_BIT)
             {
                 CLEAR_BIT(control_register, START_BIT);
             }
             else if ( (1 << iBitIndex) == RETURN_BIT)
             {
                 CLEAR_BIT(control_register, RETURN_BIT);
             }

             iBitIndex++;
             usNDiscrete--;
         }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

/**
  * @brief Status Register Handler
  * @retval None
  */
static void StatusRegister_Handler(uint32_t bit)
{
    switch(bit)
    {
        case 0: //MANUAL_MODE_BIT

            break;

        case 1: //START_MODE_BIT

            break;

        case 2: //FAILURE_BIT

            break;

        case 3: //CHOCK_SET_BIT

            break;

        default:
    }
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
