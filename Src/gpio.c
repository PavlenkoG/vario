/*
 * gpio.c
 *
 *  Created on: 18.06.2021
 *      Author: GRPA
 */
#include "gpio.h"

extern TIM_HandleTypeDef tim17;
void initLedButton ()
{

		// init user button
		GPIO_InitTypeDef GPIO_InitStruct;

		__HAL_USER_BUTTON_ENA;

		GPIO_InitStruct.Pin = USER_BUTTON_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

		HAL_GPIO_Init(USER_BUTTON_PORT, &GPIO_InitStruct);

		// init user led
		__HAL_USER_LED_ENA;
		GPIO_InitStruct.Pin = USER_LED_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

		HAL_GPIO_Init(USER_LED_PORT, &GPIO_InitStruct);

		HAL_NVIC_SetPriority(EXTI4_15_IRQn, 3, 0);
		HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	HAL_TIM_OC_Start_IT(&tim17,TIM_CHANNEL_1);

}
