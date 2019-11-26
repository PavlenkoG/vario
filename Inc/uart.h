/*
 * uart.h
 *
 *  Created on: Nov 21, 2019
 *      Author: user
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include "stm32f0xx_hal.h"
#include "platform_def.h"

extern USART_HandleTypeDef USART_DEBUG_HandleStruct;
void USART_DBG_Init(void);


#endif /* INC_UART_H_ */
