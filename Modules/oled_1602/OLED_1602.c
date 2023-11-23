/*
  ******************************************************************************
 * @file           : OLED_1602.c
 * @brief          : Subroutines for OLED display (Winstar WEH001602AGPP5N00001),
 *                   type 1602(16 char, 2 string, 4 bit mode)
 ******************************************************************************
 *  Created on: November 22, 2023
 *      Author: KiteBuilder
 */

#include "stm32f4xx_hal.h"
#include "OLED_1602.h"
#include "Delay_DWT.h"

#define _delay_ms(ms) HAL_Delay(ms)
#define _delay_us(us) delayUS_DWT(us)

//OLED Controls
#define OLED_RW(state)      HAL_GPIO_WritePin(OLED_RW_GPIO_Port, OLED_RW_Pin, state)
#define OLED_E(state)       HAL_GPIO_WritePin(OLED_E_GPIO_Port, OLED_E_Pin, state)
#define OLED_RS(state)      HAL_GPIO_WritePin(OLED_RS_GPIO_Port, OLED_RS_Pin, state)
//OLED data Bus
//Write
#define OLED_W_DB0(state)   HAL_GPIO_WritePin(OLED_DB4_GPIO_Port, OLED_DB4_Pin, state)
#define OLED_W_DB1(state)   HAL_GPIO_WritePin(OLED_DB5_GPIO_Port, OLED_DB5_Pin, state)
#define OLED_W_DB2(state)   HAL_GPIO_WritePin(OLED_DB6_GPIO_Port, OLED_DB6_Pin, state)
#define OLED_W_DB3(state)   HAL_GPIO_WritePin(OLED_DB7_GPIO_Port, OLED_DB7_Pin, state)
//Read
#define OLED_R_DB0()        HAL_GPIO_ReadPin(OLED_DB4_GPIO_Port, OLED_DB4_Pin)
#define OLED_R_DB1()        HAL_GPIO_ReadPin(OLED_DB5_GPIO_Port, OLED_DB5_Pin)
#define OLED_R_DB2()        HAL_GPIO_ReadPin(OLED_DB6_GPIO_Port, OLED_DB6_Pin)
#define OLED_R_DB3()        HAL_GPIO_ReadPin(OLED_DB7_GPIO_Port, OLED_DB7_Pin)

static void OLED_Latch(void);
static uint8_t OLED_ExecCmd(uint16_t, uint8_t);
static void OLED_WaitUntilBusy(void);
static void OLED_BusToRead(void);
static void OLED_BusToWrite(void);
static void OLED_BusSetVal(uint8_t);
static uint8_t OLED_BusGetVal(void);

/**
  * @brief Initialize LCD display - two string 16 character mode, 4 bit bus, 5x8 font, EN/RU character table
  * @param None
  * @retval None
  */
void OLED_Init(void)
{
    DWT_Delay_Init();

    OLED_RW(GPIO_PIN_RESET);
    OLED_E(GPIO_PIN_RESET);
    OLED_RS(GPIO_PIN_RESET);
    OLED_BusSetVal(0x04);
    
    _delay_ms(10);
    
    OLED_ExecCmd(FUNCTION_SET, N_BIT | FT1);        //set up functionality:
    OLED_WaitUntilBusy();                           //4 bit mode
                                                    //2 string/16 char,
                                                    //5x8 - font, EN/RU character table
      
    OLED_ExecCmd(DISPLAY_ONOFF_CMD, 0);             //display off
    OLED_WaitUntilBusy();
    
    OLED_ExecCmd(DISPLAY_ONOFF_CMD, D_BIT|B_BIT);   //display on, cursor on, blinking cursor on
    OLED_WaitUntilBusy();
    
    OLED_ExecCmd(ENTRYMODE_SET_CMD, ID_BIT);         //set up input mode:
    OLED_WaitUntilBusy();                            //DD RAM address increment (to shift the cursor one position right)
                                                     //screen dosn't shift
        
    OLED_ExecCmd(RETURN_HOME_CMD, 0);                //return to the beginning of the screen
    OLED_WaitUntilBusy();
}

/**
  * @brief Execute Read or Write command with parameters
  * @param cmd: display command value
  *        opt: option bits
  * @retval None
  */
static uint8_t OLED_ExecCmd(uint16_t cmd, uint8_t opt)
{
    uint8_t tmp = (uint8_t)cmd | opt;
  
    if ((cmd & RS_BIT) != 0)
    {
        OLED_RS(GPIO_PIN_SET);
    }

    if ((cmd & RW_BIT) != 0) //Read
    {
        //adjusting bus for read operation
        OLED_BusToRead();

        tmp = 0;

        //do read operation
        OLED_RW(GPIO_PIN_SET);
        //read MSB part of byte
        OLED_E(GPIO_PIN_SET);
        _delay_us(2);
        tmp = OLED_BusGetVal() << 4; // to take MSB part
        OLED_E(GPIO_PIN_RESET);
        _delay_us(2);
        //read LSB part of byte        
        OLED_E(GPIO_PIN_SET);
        _delay_us(2);
        tmp |= OLED_BusGetVal() ; // to take LSB part
        OLED_E(GPIO_PIN_RESET);
        OLED_RW(GPIO_PIN_RESET);

        //adjusting bus for write operation
        OLED_BusToWrite();
    }
    else //Write
    {
       OLED_BusSetVal(tmp >> 4);
       OLED_Latch();
       OLED_BusSetVal(tmp);
       OLED_Latch();
       _delay_us(2);
       tmp = 0;
    }
    
    OLED_RS(GPIO_PIN_RESET);
   
   return tmp;
}

/**
  * @brief Generate write strobe
  * @param None
  * @retval None
  */
void OLED_Latch(void)
{
   OLED_E(GPIO_PIN_SET);
   _delay_us(2);
   OLED_E(GPIO_PIN_RESET);
}

/**
  * @brief Wait until busy flag will be reset or until timeout
  * @param None
  * @retval None
  */
#define LCD_BUSY_TIMEOUT 100

static void OLED_WaitUntilBusy(void)
{
    uint8_t tmp = 0;
  
    do
    {
        __ASM volatile ("NOP");
    }while ( (OLED_ExecCmd(READ_BF_AND_ADDR_CMD, 0) & BF_BIT) && (++tmp < LCD_BUSY_TIMEOUT) );
}

/**
  * @brief Print string to the LCD display from the current position
  * @param str: pointer to '\0' terminated string
  * @retval None
*/
void OLED_PutStr(char *str)
{
    uint8_t i;

    i = 0;
    while ((*(str + i) != '\0') && (i < LCD_STR_LEN))
    {
        OLED_ExecCmd(WRITE_RAM_CMD, *(str + i));
        OLED_WaitUntilBusy();
        i++;
    }
}

/**
  * @brief Print one chatracter to the LCD from the current position
  * @param chr: character ASCII code
  * @retval None
*/
void OLED_PutCh(char chr)
{
     OLED_ExecCmd(WRITE_RAM_CMD, chr);
     OLED_WaitUntilBusy();
}

/**
  * @brief Set up DD RAM address. For two string display
  * @param addr: string address
  *              first string address from 00� to 27�,
  *              second string address from 40� to 67�
  * @retval None
  */
void OLED_GoTo(uint8_t addr)
{
     OLED_ExecCmd(SET_DDRAM_ADDR_CMD, addr & DDRAM_ADDR_MSK);
     OLED_WaitUntilBusy();
}

/**
  * @brief Clear display
  * @param None
  * @retval None
  */
void OLED_Clear(void)
{
     OLED_ExecCmd(CLEAR_DISPLAY_CMD,  0);
     OLED_WaitUntilBusy();
     _delay_ms(5);
}

/**
  * @brief Hide cursor
  * @param None
  * @retval None
  */
void OLED_HideCursor(void)
{
    OLED_ExecCmd(DISPLAY_ONOFF_CMD, D_BIT);
    OLED_WaitUntilBusy();
}

/**
  * @brief Show blinking cursor
  * @param None
  * @retval None
  */
void OLED_ShowCursor(void)
{
    OLED_ExecCmd(DISPLAY_ONOFF_CMD,  D_BIT | B_BIT | C_BIT);
    OLED_WaitUntilBusy();
}

/**
  * @brief Adjust data bus for read operations
  * @param None
  * @retval None
  */
static void OLED_BusToRead(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(OLED_DB4_GPIO_Port, OLED_DB4_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(OLED_DB5_GPIO_Port, OLED_DB5_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(OLED_DB6_GPIO_Port, OLED_DB6_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(OLED_DB7_GPIO_Port, OLED_DB7_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : OLED_DB4_Pin */
    GPIO_InitStruct.Pin = OLED_DB4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(OLED_DB4_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : OLED_DB5_Pin */
    GPIO_InitStruct.Pin = OLED_DB5_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(OLED_DB5_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : OLED_DB6_Pin */
    GPIO_InitStruct.Pin = OLED_DB6_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(OLED_DB6_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : OLED_DB7_Pin */
    GPIO_InitStruct.Pin = OLED_DB7_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(OLED_DB7_GPIO_Port, &GPIO_InitStruct);
}

/**
  * @brief Adjust data bus for write operations
  * @param None
  * @retval None
  */
static void OLED_BusToWrite(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(OLED_DB4_GPIO_Port, OLED_DB4_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(OLED_DB5_GPIO_Port, OLED_DB5_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(OLED_DB6_GPIO_Port, OLED_DB6_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(OLED_DB7_GPIO_Port, OLED_DB7_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : OLED_DB4_Pin */
    GPIO_InitStruct.Pin = OLED_DB4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(OLED_DB4_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : OLED_DB5_Pin */
    GPIO_InitStruct.Pin = OLED_DB5_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(OLED_DB5_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : OLED_DB6_Pin */
    GPIO_InitStruct.Pin = OLED_DB6_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(OLED_DB6_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : OLED_DB7_Pin */
    GPIO_InitStruct.Pin = OLED_DB7_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(OLED_DB7_GPIO_Port, &GPIO_InitStruct);
}

/**
  * @brief Set OLED Display bus with a 4 bit value
  * @param data: data byte with valuable 4 lsb bits
  * @retval None
  */
static void OLED_BusSetVal(uint8_t data)
{
    GPIO_PinState pin_state[4];

    for (uint8_t i = 0; i < 4; i++)
    {
        pin_state[i] = (data & (1 << i)) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    }

    OLED_W_DB0(pin_state[0]);
    OLED_W_DB1(pin_state[1]);
    OLED_W_DB2(pin_state[2]);
    OLED_W_DB3(pin_state[3]);
}

/**
  * @brief Get OLED Display 4bit bus value
  * @param None
  * @retval None
  */
static uint8_t OLED_BusGetVal(void)
{
    uint8_t tmp = 0;

    tmp |= OLED_R_DB0();
    tmp |= OLED_R_DB1() << 1;
    tmp |= OLED_R_DB2() << 2;
    tmp |= OLED_R_DB3() << 3;

    return tmp;
}
