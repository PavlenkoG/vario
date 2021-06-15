#include "tim.h"
#include "main.h"


TIM_Base_InitTypeDef TIM_BaseStruct;
TIM_HandleTypeDef tim2;
TIM_HandleTypeDef tim3;
TIM_OC_InitTypeDef sConfig;
uint8_t onOff;

struct varioTone tone;

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

}

void MX_TIM2_Init (void) {
	__HAL_RCC_TIM2_CLK_ENABLE();
	onOff = 0;
	tim2.Instance = TIM2;
	tim2.Init.Prescaler = 3;
	tim2.Init.Period = getCycle(&tone);//6000000; //48000000/4*0.001 = 12000 1 ms.
	tim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	tim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	tim2.Init.RepetitionCounter = 0;
	tim2.Init.AutoReloadPreload = TIM_AUTOMATICOUTPUT_ENABLE;

	sConfig.OCMode = TIM_OCMODE_TIMING;
	sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
	sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;

	sConfig.Pulse = 0x2EE0;
	TIM_BaseStruct.AutoReloadPreload = getCycle(&tone);
	TIM_BaseStruct.Period = getCycle(&tone);

	TIM_Base_SetConfig(TIM2, &TIM_BaseStruct);
	HAL_TIM_OC_ConfigChannel(&tim2, &sConfig, TIM_CHANNEL_1);

	HAL_NVIC_SetPriority(TIM2_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);

	HAL_TIM_OC_Start_IT(&tim2,TIM_CHANNEL_1);
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM3) {
		GPIO_InitTypeDef GPIO_InitStruct;
		__HAL_RCC_GPIOC_CLK_ENABLE();
		GPIO_InitStruct.Pin = GPIO_PIN_6;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF0_TIM3;
		HAL_GPIO_Init(GPIOC,&GPIO_InitStruct);
	}

}

void TimStart (TIM_HandleTypeDef *timer, uint32_t period) {
	HAL_TIM_PWM_Start(&tim3,TIM_CHANNEL_1);
	timer->Instance->ARR = period;
	timer->Instance->CCR1 = period >> 1;
}

void TimStop (void) {
	HAL_TIM_PWM_Stop(&tim3,TIM_CHANNEL_1);
}
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
	if (!onOff) {
		TIM_BaseStruct.AutoReloadPreload = onTime(&tone);
		TIM_BaseStruct.Period = onTime(&tone);
		TIM_Base_SetConfig(TIM2, &TIM_BaseStruct);

		TimStart(&tim3,(12000000/tone.toneFreq));
		onOff = 1;
	}
	else {
		TIM_BaseStruct.AutoReloadPreload = offTime(&tone);
		TIM_BaseStruct.Period = offTime(&tone);
		TimStop();
		onOff = 0;
	}

}
