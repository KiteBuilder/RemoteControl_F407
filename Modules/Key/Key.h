/*
  ******************************************************************************
 * @file           : Key.h
 * @brief          : Header for Key.c file.
 *                   Subroutines for keys support
 ******************************************************************************
 *  Created on: November 22, 2023
 *      Author: KiteBuilder
 */

#ifndef KEY_H_
#define KEY_H_

typedef enum _key_state_ {RELEASED = 0, PRESSED = 1} key_state_e;

typedef enum _key_active_level_ {LO_LEVEL = 0, HI_LEVEL = 1} key_active_level_e;

#define GUARD_DELAY   10 //Depends on the polling period. The guard time designates the time period during which the button can't be pressed or released
#define TREMBLE_THRESHOLD 3 //Key can be detected as pressed or released if it detected in this state TREMBLE_THRESHOLD times

typedef void (*p_key_handler)(key_state_e);

typedef struct _key_st_
{
    key_state_e key_state;
    uint8_t tremble_cnt;
    uint8_t guard_cnt;

    GPIO_TypeDef* gpio;
    uint16_t pin;
    key_active_level_e level;

    p_key_handler p_handler;
} key_t;

void Key_Init(key_t*, GPIO_TypeDef*, uint16_t, key_active_level_e, p_key_handler);
void Key_CheckState(key_t*);
key_state_e Key_InstantCheck(key_t*);

#endif /* KEY_H_ */
