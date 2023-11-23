/*
 * mt_port.h
 *
 *  Created on: Oct 20, 2023
 *      Author:
 */

#ifndef MODBUS_PORT_MT_PORT_H_
#define MODBUS_PORT_MT_PORT_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Declarations and definitions ----------------------------------------------*/
/* Functions -----------------------------------------------------------------*/
extern void MT_PORT_SetTimerModule(TIM_HandleTypeDef* timer);
extern void MT_PORT_SetUartModule(UART_HandleTypeDef* uart);

#endif /* MODBUS_PORT_MT_PORT_H_ */
