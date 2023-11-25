/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id$
 */

#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "stm32f4xx_hal.h"

/* ----------------------- static functions ---------------------------------*/
static void prvvUARTTxReadyISR( void );
static void prvvUARTRxISR( void );

/* ----------------------- Variables ----------------------------------------*/
extern UART_HandleTypeDef* modbusUart;
uint8_t txByte = 0x00;
uint8_t rxByte = 0x00;

/* ----------------------- Start implementation -----------------------------*/
void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
    if (xRxEnable == FALSE)
    {
      HAL_UART_AbortReceive_IT(modbusUart);
    }
    else
    {
      HAL_UART_Receive_IT(modbusUart, &rxByte, 1);
    }
    if (xTxEnable == FALSE)
    {
      HAL_UART_AbortTransmit_IT(modbusUart);
    }
    else
    {
      if (modbusUart->gState == HAL_UART_STATE_READY)
      {
        prvvUARTTxReadyISR();
      }
    }
}

BOOL
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
    return TRUE;
}

BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
    txByte = ucByte;
    HAL_UART_Transmit_IT(modbusUart, &txByte, 1);
    return TRUE;
}

BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
    *pucByte = rxByte;
    HAL_UART_Receive_IT(modbusUart, &rxByte, 1);
    return TRUE;
}

/* Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call 
 * xMBPortSerialPutByte( ) to send the character.
 */
static void prvvUARTTxReadyISR( void )
{
    pxMBFrameCBTransmitterEmpty(  );
}

/* Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
static void prvvUARTRxISR( void )
{
    pxMBFrameCBByteReceived(  );
}

/* --------------------------------------------------------------------------*/
void HAL_UART_TxCpltCallback_modbus(UART_HandleTypeDef *huart)
{
  if (huart->Instance == modbusUart->Instance)
  {
      prvvUARTTxReadyISR();
  }
}
/* --------------------------------------------------------------------------*/
void HAL_UART_RxCpltCallback_modbus(UART_HandleTypeDef *huart)
{
  if (huart->Instance == modbusUart->Instance)
  {
      prvvUARTRxISR();
  }
}

