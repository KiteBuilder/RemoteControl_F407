/*
  ******************************************************************************
 * @file           : oled_1602.h
 * @brief          : Header for oled_1620.c file.
 *                   Subroutines for OLED display (Winstar WEH001602AGPP5N00001),
 *                   type 1602(16 char, 2 string, 4 bit mode)
 ******************************************************************************
 *  Created on: November 22, 2023
 *      Author: KiteBuilder
 */

#ifndef _OLED_1602_MODULE_H_
#define _OLED_1602_MODULE_H_

//OLED display pins mapping
#define OLED_RW_Pin         GPIO_PIN_7
#define OLED_RW_GPIO_Port   GPIOA

#define OLED_RS_Pin         GPIO_PIN_9
#define OLED_RS_GPIO_Port   GPIOF

#define OLED_E_Pin          GPIO_PIN_1
#define OLED_E_GPIO_Port    GPIOB

#define OLED_DB4_Pin        GPIO_PIN_0
#define OLED_DB4_GPIO_Port  GPIOB

#define OLED_DB5_Pin        GPIO_PIN_15
#define OLED_DB5_GPIO_Port  GPIOF

#define OLED_DB6_Pin        GPIO_PIN_7
#define OLED_DB6_GPIO_Port  GPIOE

#define OLED_DB7_Pin        GPIO_PIN_12
#define OLED_DB7_GPIO_Port  GPIOF

//Instruction codes
#define CLEAR_DISPLAY_CMD       0x0001 //2-6ms delay required after this command

#define RETURN_HOME_CMD         0x0002

#define ENTRYMODE_SET_CMD       0x0004
  #define ID_BIT                0x02   //1-Increment/0-Decrement DDRAM address, change cursor position
  #define S_BIT                 0x01   //S=0 - shift of entire display is not performed
                                       //S=1 - shift of entire display is performed according ID value

#define DISPLAY_ONOFF_CMD       0x0008
  #define D_BIT                 0x04   //1-entire display is turned on, 0-turned off
  #define C_BIT                 0x02   //0-cursor is turned on, 1-turned off
  #define B_BIT                 0x01   //1-cursor blink is on, 0-is off

#define CUR_DISP_SHIFT_CMD      0x0010
  #define SC_BIT                0x08   //1-display shift, 0-cursor move
  #define RL_BIT                0x04   //1-shift to right, 0-shift to left
  #define SHIFT_CUR_LEFT        0x00
  #define SHIFT_CUR_RIGHT       0x04
  #define SHIFT_DISP_LEFT       0x08
  #define SHIFT_DISP_RIGHT      0x0C

#define FUNCTION_SET            0x0020
  #define FT0                   0x01   //FT1:FT0 sets font table
  #define FT1                   0x02   //0:0-EN/Japan, 0:1-WestEur1, 1:0-En/Rus, 1:1-WestEur2
  #define F_BIT                 0x04   //1 - 5*10 dots, 0 - 5*8 dots
  #define N_BIT                 0x08   //1-two lines, 0-one line
  #define DL_BIT                0x10   //bus line width 1 - 8bit, 0 - 4bit

#define SET_CGRAM_ADDR          0x0040
  #define CGRAM_ADDR_MSK        0x3F

#define SET_DDRAM_ADDR_CMD      0x0080
  #define DDRAM_ADDR_MSK        0x7F

#define READ_BF_AND_ADDR_CMD    0x0100
  #define BF_BIT                0x80   //1-Busy, 0-Not busy

#define WRITE_RAM_CMD           0x0200

#define READ_RAM_CMD            0x0300

  #define RS_BIT                0x200 //RS bit position in the command code
  #define RW_BIT                0x100 //R/W bit position in the command code

#define LCD_STR_LEN 16

void OLED_Init(void);           //Initialize OLED display - two string mode, 4bit bus
void OLED_GoTo(uint8_t);        //Set up DD RAM address 00 - 0F first string, 40 - 4F second string
void OLED_PutStr(char *);       //Print string to the OLED display from current position
void OLED_PutCh(char);          //Print one chatracter to the OLED from current position
void OLED_Clear(void);          //Clear display
void OLED_HideCursor(void);     //Hide cursor
void OLED_ShowCursor(void);     //Show cursor

#endif
