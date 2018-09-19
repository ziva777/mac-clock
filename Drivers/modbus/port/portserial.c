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
 * File: $Id: portserial.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

#include "port.h"
#include "stm32f4xx_hal.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- static functions ---------------------------------*/
UART_HandleTypeDef huart_m;
HAL_StatusTypeDef USART_Init( UART_HandleTypeDef* huart);
void USART_MspInit( UART_HandleTypeDef* huart );
static void USART_SetConfig( UART_HandleTypeDef* huart );

/* ----------------------- Start implementation -----------------------------*/
void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
    /* If xRXEnable enable serial receive interrupts. If xTxENable enable
     * transmitter empty interrupts.
     */
  if(xRxEnable)
  {
    __HAL_UART_ENABLE_IT(&huart_m, UART_IT_RXNE);
  }
  else
  {
    __HAL_UART_DISABLE_IT(&huart_m, UART_IT_RXNE);
  }

  if(xTxEnable)
  {
    __HAL_UART_ENABLE_IT(&huart_m, UART_IT_TC);
    HAL_GPIO_WritePin(USART3_DE_GPIO_Port, USART3_DE_Pin, GPIO_PIN_SET);
  }
  else
  {
    __HAL_UART_DISABLE_IT(&huart_m, UART_IT_TC);
    HAL_GPIO_WritePin(USART3_DE_GPIO_Port, USART3_DE_Pin, GPIO_PIN_RESET);
  }
}

BOOL
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
	  switch (ucPORT)
	  {
		case 0:
	          huart_m.Instance = USART1;
	          break;
	 	case 1:
	          huart_m.Instance = USART2;
	          break;
	 	case 2:
	          huart_m.Instance = USART3;
	          break;
        default:
	          return FALSE;
	  }
	  huart_m.Init.BaudRate = ulBaudRate;
	  switch (ucDataBits)
	  {
	        case 8:
	                huart_m.Init.WordLength = UART_WORDLENGTH_8B;
	                break;
	        default:
	                return FALSE;
	  }
	  switch (eParity)
	  {
	    case MB_PAR_NONE:
	            huart_m.Init.Parity = UART_PARITY_NONE;
	            break;
	    case MB_PAR_EVEN:
	            huart_m.Init.Parity = UART_PARITY_EVEN;
	            break;
	    case MB_PAR_ODD:
	            huart_m.Init.Parity = UART_PARITY_ODD;
	            break;
	    default:
	            return FALSE;
	  }
	  huart_m.Init.StopBits = UART_STOPBITS_1;
	  huart_m.Init.Mode = UART_MODE_TX_RX;
	  huart_m.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	  huart_m.Init.OverSampling = UART_OVERSAMPLING_16;

	  return (HAL_OK == USART_Init(&huart_m/*, UART_DE_POLARITY_HIGH, 0, 0*/));
}

BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
    /* Put a byte in the UARTs transmit buffer. This function is called
     * by the protocol stack if pxMBFrameCBTransmitterEmpty( ) has been
     * called. */
    huart_m.Instance->DR = ucByte;
    return TRUE;
}

BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
    /* Return the byte in the UARTs receive buffer. This function is called
     * by the protocol stack after pxMBFrameCBByteReceived( ) has been called.
     */
    if(huart_m.Init.Parity == UART_PARITY_NONE)
    {
        *pucByte = (uint8_t)(huart_m.Instance->DR & (uint8_t)0x00FF);
    }
    else
    {
        *pucByte = (uint8_t)(huart_m.Instance->DR & (uint8_t)0x007F);
    }
    return TRUE;
}

/* Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call 
 * xMBPortSerialPutByte( ) to send the character.
 */
/*static*/ void prvvUARTTxReadyISR( void )
{
    pxMBFrameCBTransmitterEmpty(  );
}

/* Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
/*static*/ void prvvUARTRxISR( void )
{
    pxMBFrameCBByteReceived(  );
}

HAL_StatusTypeDef USART_Init(UART_HandleTypeDef *huart)
{
  /* Check the UART handle allocation */
  if(huart == NULL)
  {
    return HAL_ERROR;
  }

  /* Check the parameters */
  if(huart->Init.HwFlowCtl != UART_HWCONTROL_NONE)
  {
    /* The hardware flow control is available only for USART1, USART2, USART3 */
    assert_param(IS_UART_HWFLOW_INSTANCE(huart->Instance));
    assert_param(IS_UART_HARDWARE_FLOW_CONTROL(huart->Init.HwFlowCtl));
  }
  else
  {
    assert_param(IS_UART_INSTANCE(huart->Instance));
  }

  assert_param(IS_UART_WORD_LENGTH(huart->Init.WordLength));
  assert_param(IS_UART_OVERSAMPLING(huart->Init.OverSampling));

  if(huart->gState == HAL_UART_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    huart->Lock = HAL_UNLOCKED;

    /* Init the low level hardware */
    USART_MspInit(huart);
  }

  huart->gState = HAL_UART_STATE_BUSY;

  /* Disable the peripheral */
  __HAL_UART_DISABLE(huart);

  /* Set the UART Communication parameters */
  USART_SetConfig(huart);

  /* In asynchronous mode, the following bits must be kept cleared:
     - LINEN and CLKEN bits in the USART_CR2 register,
     - SCEN, HDSEL and IREN  bits in the USART_CR3 register.*/
  CLEAR_BIT(huart->Instance->CR2, (USART_CR2_LINEN | USART_CR2_CLKEN));
  CLEAR_BIT(huart->Instance->CR3, (USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN));

  /* Enable the peripheral */
  __HAL_UART_ENABLE(huart);

  /* Initialize the UART state */
  huart->ErrorCode = HAL_UART_ERROR_NONE;
  huart->gState= HAL_UART_STATE_READY;

  return HAL_OK;
}

void USART_MspInit(UART_HandleTypeDef* huart)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	if (huart->Instance == USART3) {
		/* USART3 clock enable */
		__HAL_RCC_USART3_CLK_ENABLE();

		/**USART3 GPIO Configuration
		 PB10     ------> USART3_TX
		 PB11     ------> USART3_RX
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		HAL_GPIO_WritePin(USART3_DE_GPIO_Port, USART3_DE_Pin, GPIO_PIN_RESET);

		/* USART3 interrupt Init */
		HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(USART3_IRQn);
	} else if (huart->Instance == USART2) {
	}
}

static void USART_SetConfig(UART_HandleTypeDef* huart)
{
    uint32_t tmpreg = 0x00;

  /* Check the parameters */
  assert_param(IS_UART_BAUDRATE(huart->Init.BaudRate));
  assert_param(IS_UART_STOPBITS(huart->Init.StopBits));
  assert_param(IS_UART_PARITY(huart->Init.Parity));
  assert_param(IS_UART_MODE(huart->Init.Mode));

  /*------- UART-associated USART registers setting : CR2 Configuration ------*/
  /* Configure the UART Stop Bits: Set STOP[13:12] bits according
   * to huart->Init.StopBits value */
  MODIFY_REG(huart->Instance->CR2, USART_CR2_STOP, huart->Init.StopBits);

  /*------- UART-associated USART registers setting : CR1 Configuration ------*/
  /* Configure the UART Word Length, Parity and mode:
     Set the M bits according to huart->Init.WordLength value
     Set PCE and PS bits according to huart->Init.Parity value
     Set TE and RE bits according to huart->Init.Mode value */
  tmpreg = (uint32_t)huart->Init.WordLength | huart->Init.Parity | huart->Init.Mode ;
  MODIFY_REG(huart->Instance->CR1,
             (uint32_t)(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS | USART_CR1_TE | USART_CR1_RE),
             tmpreg);

  /*------- UART-associated USART registers setting : CR3 Configuration ------*/
  /* Configure the UART HFC: Set CTSE and RTSE bits according to huart->Init.HwFlowCtl value */
  MODIFY_REG(huart->Instance->CR3, (USART_CR3_RTSE | USART_CR3_CTSE), huart->Init.HwFlowCtl);

  /*------- UART-associated USART registers setting : BRR Configuration ------*/
  if((huart->Instance == USART1))
  {
    huart->Instance->BRR = UART_BRR_SAMPLING16(HAL_RCC_GetPCLK2Freq(), huart->Init.BaudRate);
  }
  else
  {
    huart->Instance->BRR = UART_BRR_SAMPLING16(HAL_RCC_GetPCLK1Freq(), huart->Init.BaudRate);
  }
}

