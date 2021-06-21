#include "tim.h"
#include "main.h"


TIM_Base_InitTypeDef TIM_BaseStruct;
TIM_HandleTypeDef tim2;
TIM_HandleTypeDef tim3;
TIM_HandleTypeDef tim17;

TIM_OC_InitTypeDef sConfig;
uint8_t onOff;

extern struct varioTone tone;

void MX_TIM3_Init (void) {
	// TIMER3 used for buzz signal
	__HAL_RCC_TIM3_CLK_ENABLE();
	tim3.Instance = TIM3;
	tim3.Init.Prescaler = 3;
	tim3.Init.Period = 0x9C40; //48MHz/4 = 12, 9C40=40000, = 300Hz
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

void MX_TIM2_Init (void) {
	// TIMER2 for periodical sound
	__HAL_RCC_TIM2_CLK_ENABLE();
	onOff = 0;

	tim2.Instance = TIM2;

	TIM_BaseStruct.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	TIM_BaseStruct.Period = 100;
	TIM_BaseStruct.Prescaler = 3;
	TIM_BaseStruct.CounterMode = TIM_COUNTERMODE_UP;

	TIM_Base_SetConfig(TIM2, &TIM_BaseStruct);

	HAL_NVIC_SetPriority(TIM2_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);

}

void MX_TIM17_Init (void) {
	// TIMER17 used for user pin debounce
	__HAL_RCC_TIM17_CLK_ENABLE();
	tim17.Instance = TIM17;
	tim17.Init.Prescaler = 3;
	tim17.Init.Period = 0x9C40; //48MHz/4 = 12, 9C40=40000, = 300Hz
	tim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	tim17.Init.CounterMode = TIM_COUNTERMODE_UP;
	tim17.Init.RepetitionCounter = 0;
	tim17.Init.AutoReloadPreload = TIM_AUTOMATICOUTPUT_ENABLE;
	HAL_TIM_PWM_Init(&tim17);

	sConfig.OCMode = TIM_OCMODE_INACTIVE;
	sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
	sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;

	sConfig.Pulse = 0x2EE0;
	TIM_BaseStruct.AutoReloadPreload = 1000;
	TIM_BaseStruct.Period = 1000;

	TIM_Base_SetConfig(TIM17, &TIM_BaseStruct);
	HAL_TIM_OC_ConfigChannel(&tim17, &sConfig, TIM_CHANNEL_1);

	HAL_NVIC_SetPriority(TIM17_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(TIM17_IRQn);

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

void TimStart (uint32_t period) {
	__HAL_TIM_SET_AUTORELOAD(&tim3, period);
	__HAL_TIM_SET_COMPARE(&tim3,TIM_CHANNEL_1, period>>1);
	HAL_TIM_PWM_Start(&tim3,TIM_CHANNEL_1);
}

void setPwmFreq (uint32_t period) {
	__HAL_TIM_SET_AUTORELOAD(&tim3, period);
	__HAL_TIM_SET_COMPARE(&tim3,TIM_CHANNEL_1, period>>1);
}

void TimStop (void) {
	HAL_TIM_PWM_Stop(&tim3,TIM_CHANNEL_1);
}
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		//HAL_TIM_OC_Stop(&tim2, TIM_CHANNEL_1);
		struct timerSound timersound = getTimerSoundConfig(&tone);
		if (!onOff) {
			__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_1, timersound.onCount);
			__HAL_TIM_SET_AUTORELOAD(htim, timersound.onCount);
			//HAL_TIM_OC_Start_IT(&tim2,TIM_CHANNEL_1);
			TimStart(12000000/tone.toneFreq);
			onOff = 1;
		}
		else {
			if (timersound.offCount) {
				__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_1, timersound.offCount);
				__HAL_TIM_SET_AUTORELOAD(htim, timersound.offCount);
				//HAL_TIM_OC_Start_IT(&tim2,TIM_CHANNEL_1);
				TimStop();
			}
			onOff = 0;
		}
	}

	if (htim->Instance == TIM17) {
		if (HAL_GPIO_ReadPin(USER_BUTTON_PORT, USER_BUTTON_PIN)){
			HAL_GPIO_TogglePin(USER_LED_PORT, USER_LED_PIN);
		}
	}
}
