#include "tim.h"
#include "main.h"


TIM_Base_InitTypeDef TIM_BaseStruct;
TIM_HandleTypeDef tim3;
TIM_OC_InitTypeDef sConfig;

void MX_TIM3_Init (void) {
	__HAL_RCC_TIM3_CLK_ENABLE();
	tim3.Instance = TIM3;
	tim3.Init.Prescaler = 3;
	tim3.Init.Period = 0x9C40;
	tim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	tim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	tim3.Init.RepetitionCounter = 0;
	tim3.Init.AutoReloadPreload = TIM_AUTOMATICOUTPUT_DISABLE;
	HAL_TIM_PWM_Init(&tim3);

	sConfig.OCMode = TIM_OCMODE_PWM1;
	sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
	sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
	sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
	sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;

	sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;
	sConfig.Pulse = 0x4E20;
	HAL_TIM_PWM_ConfigChannel(&tim3, &sConfig, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&tim3,TIM_CHANNEL_1);

}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim) {
	GPIO_InitTypeDef GPIO_InitStruct;
	__HAL_RCC_GPIOC_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF0_TIM3;
	HAL_GPIO_Init(GPIOC,&GPIO_InitStruct);
}

