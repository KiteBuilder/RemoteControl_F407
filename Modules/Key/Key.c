/*
  ******************************************************************************
 * @file           : Key.c
 * @brief          : Header for Key.c file.
 *                   Subroutines for keys support
 ******************************************************************************
 *  Created on: November 22, 2023
 *      Author: KiteBuilder
 */
#include "stm32f4xx_hal.h"
#include "Key.h"

/**
  * @brief Key initialize routine
  * @param key: pointer to the key_t structure
  *        gpio: pointer to GPIO_TypeDef structure
  *        pin:  gpio pin, should be adjusted as an input with pull-up
  *        p_handler: pointer to the key handler routine or NULL
  * @retval None
  */
void Key_Init(key_t *p_key, GPIO_TypeDef* p_gpio, uint16_t pin, key_active_level_e level, p_key_handler p_handler)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    p_key->key_state = RELEASED;
    p_key->tremble_cnt = 0;
    p_key->guard_cnt = 0;
    p_key->gpio =  p_gpio;
    p_key->pin =  pin;
    p_key->level = level;
    p_key->p_handler = p_handler;

    GPIO_InitStruct.Pin = p_key->pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = (p_key->level == LO_LEVEL) ? GPIO_PULLUP : GPIO_PULLDOWN;
    HAL_GPIO_Init(p_key->gpio, &GPIO_InitStruct);
}

/**
  * @brief Check keys state
  * @param *key: key structure pointer
  * @retval None
  */
void Key_CheckState(key_t *p_key)
{
    if (p_key->guard_cnt == 0) //during GUARD_DELAY key can't be detected as pressed or released
    {
        if ( HAL_GPIO_ReadPin(p_key->gpio, p_key->pin) ==  (p_key->level == LO_LEVEL ? GPIO_PIN_RESET : GPIO_PIN_SET) )
        {
            if (p_key->key_state == RELEASED)
            {
                if (++p_key->tremble_cnt == TREMBLE_THRESHOLD) //key should be detected as pressed this TREMBLE_THRESHOLD times to say that it's pressed
                {
                    p_key->tremble_cnt = 0;
                    p_key->key_state = PRESSED;
                    p_key->guard_cnt = GUARD_DELAY;
                    if (p_key->p_handler != NULL)
                    {
                        p_key->p_handler(p_key->key_state);
                    }
                }
            }
        }
        else
        {
            if (p_key->key_state == PRESSED)
            {
                if(++p_key->tremble_cnt == TREMBLE_THRESHOLD) //key should be detected as released this TREMBLE_THRESHOLD times to say that it's pressed
                {
                    p_key->tremble_cnt = 0;
                    p_key->key_state = RELEASED;
                    p_key->guard_cnt = GUARD_DELAY;
                    if (p_key->p_handler != NULL)
                    {
                        p_key->p_handler(p_key->key_state);
                    }
                }
            }
            else
            {
                p_key->tremble_cnt = 0;
            }
        }
    }
    else
    {
        --p_key->guard_cnt;
    }
}
