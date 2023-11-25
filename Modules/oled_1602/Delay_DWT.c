/*
  ******************************************************************************
 * @file           : Delay_DWT.c
 * @brief          : Subroutines for making delay in microseconds
 ******************************************************************************
 *  Created on: November 22, 2023
 *      Author: KiteBuilder
 */

#include "stm32f4xx_hal.h"
#include "Delay_DWT.h"

#pragma GCC push_options
#pragma GCC optimize ("O3")
/**
 * @brief  Initializes DWT_Clock_Cycle_Count for DWT_Delay_us function
 * @return Error DWT counter
 *         1: clock cycle counter not started
 *         0: clock cycle counter works
 */
uint32_t DWT_Delay_Init(void)
{
    /* Disable TRC */
    CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk; // ~0x01000000;
    /* Enable TRC */
    CoreDebug->DEMCR |=  CoreDebug_DEMCR_TRCENA_Msk; // 0x01000000;

    /* Disable clock cycle counter */
    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; //~0x00000001;
    /* Enable  clock cycle counter */
    DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk; //0x00000001;

    /* Reset the clock cycle counter value */
    DWT->CYCCNT = 0;

    /* 3 NO OPERATION instructions */
    __ASM volatile ("NOP");
    __ASM volatile ("NOP");
    __ASM volatile ("NOP");

    /* Check if clock cycle counter has started */
    if (DWT->CYCCNT)
    {
        return 0; /*clock cycle counter started*/
    }
    else
    {
        return 1; /*clock cycle counter not started*/
    }
}


/**
  * @brief Delay in microseconds
  * @param us: delay in microseconds
  * @retval None
  */
inline void delayUS_DWT(uint32_t us)
{
    volatile uint32_t cycles = (SystemCoreClock/1000000L) * us; // Go to number of cycles for system
    volatile uint32_t start = DWT->CYCCNT;

    do
    {

    } while(DWT->CYCCNT - start < cycles);
}

#pragma GCC pop_options
