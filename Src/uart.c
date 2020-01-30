/*
 * uart.c
 *
 *  Created on: Nov 21, 2019
 *      Author: user
 */
#include "uart.h"


USART_HandleTypeDef USART_DEBUG_HandleStruct;
void HAL_USART_MspInit(USART_HandleTypeDef *husart);

void USART_DBG_Init (void);


void USART_DBG_Init(void) {
    USART_InitTypeDef USART_InitStruct;
    __HAL_RCC_USART2_CLK_ENABLE();

    USART_InitStruct.BaudRate = 115200;
    USART_InitStruct.WordLength = 8;
    USART_InitStruct.StopBits = USART_STOPBITS_1;
    USART_InitStruct.Mode = USART_MODE_TX_RX;
    USART_InitStruct.Parity = USART_PARITY_NONE;

    USART_DEBUG_HandleStruct.Instance = USART2;
    USART_DEBUG_HandleStruct.Init = USART_InitStruct;
    HAL_USART_Init(&USART_DEBUG_HandleStruct);

}
void HAL_USART_MspInit(USART_HandleTypeDef *husart){
	if (husart->Instance == USART2) {
		GPIO_InitTypeDef GPIO_InitStruct;
		__HAL_RCC_GPIOA_CLK_ENABLE();
		GPIO_InitStruct.Pin = UART_DBG_RX_PIN | UART_DBG_TX_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF1_USART2;

		HAL_GPIO_Init(UART_DBG_PORT, &GPIO_InitStruct);
	}
}






