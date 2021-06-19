/**
 ******************************************************************************
 * @file    Templates/Src/main.c
 * @author  MCD Application Team
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ssd1306.h"
#include "bmp280.h"
#include "MPU-9250.h"
#include "math.h"
#include <stdio.h>  /*rtt*/
#include <stdlib.h> /*rtt*/

/** @addtogroup STM32F0xx_HAL_Examples
 * @{
 */

/** @addtogroup Templates
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
double BME280_CalcTf(double pressure);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
#define HIGH_FREQ = 1600
#define LOW_FREQ = 200

#define HIGH_FREQ_PERIOD  (48000000/4)/1600
#define LOW_FREQ_PERIOD  (48000000/4)/200
#define ALTITUDE_SAMPLES_FILTER 10
extern __IO uint32_t uwTick;

struct varioTone tone;

int main(void) {

    int rslt;
    uint8_t bmp280Period;
    struct bmp280_dev bmp;
    struct bmp280_config conf;
    struct bmp280_uncomp_data ucomp_data;
    struct bmp280_status bmpStatus;
    int32_t temp = 0;
    double alti = 0.0;
    double altiVector[ALTITUDE_SAMPLES_FILTER] = {0};
    double averageSpeed = 0.0;
    double speed = 0.0;
    double averageAlti = 0.0;
    double averageAltiOld = 0.0;
    uint8_t arrayIndex = 0;
    double pres = 0.0;
    MPU9250_gyro_val gyro;
    MPU9250_accel_val accel;
    MPU9250_magnetometer_val magn;

	uint32_t period = 0x9C40;
	uint32_t timer_old = 0;
	uint32_t timer_new = 0;
	uint32_t timer = 0;
	tone.averageSpeed = 0.0;
	tone.cycle = 700;
	tone.toneDutyCycle = 50;
	SimpleKalman kalman;
	initKalman(&kalman);

	altimeterHight altimeter;
	double verticalSpeed;

	uint8_t buttonPressed = 0;
	uint8_t soundOnOff = 0;
	uint8_t soundCounter = 0;

	HAL_Init();

	/* Configure the system clock to have a system clock = 48 Mhz */
	SystemClock_Config();

	USART_DBG_Init();
  	SSD1306_Init();
	MX_I2C1_Init();
    MX_TIM3_Init();
    MX_TIM2_Init();
    MX_TIM17_Init();
    initLedButton();

    bmp.delay_ms = HAL_Delay;
    bmp.dev_id = 0xec;//BMP280_I2C_ADDR_SEC;
    /* Select the interface mode as I2C */
    bmp.intf = BMP280_I2C_INTF;

    /* Map the I2C read & write function pointer with the functions responsible for I2C bus transfer */
    bmp.read = I2C_Read;
    bmp.write = I2C_Write;
    rslt = bmp280_init(&bmp);
    rslt = bmp280_get_config(&conf, &bmp);
	conf.filter = BMP280_FILTER_COEFF_2;
	conf.os_pres = BMP280_OS_2X;
	conf.os_temp = BMP280_OS_1X;
	conf.odr = BMP280_ODR_0_5_MS;
	rslt = bmp280_set_config(&conf, &bmp);

//	rslt = MPU9250_drv_init();

	if (rslt == BMP280_OK) {
		printf("Pressure sensor OK\r\n");
	} else {
		printf("Error by pressure sensor init\r\n");
	}

	bmp280Period = bmp280_compute_meas_time(&bmp);
	printf ("Pressure sensor period = %d\r\n",bmp280Period);
	bmp280_set_power_mode(BMP280_NORMAL_MODE,&bmp);

//	MPU9250_drv_start_maesure(MPU9250_BIT_GYRO_FS_SEL_250DPS,MPU9250_BIT_ACCEL_FS_SEL_8G,MPU9250_BIT_DLPF_CFG_5HZ,MPU9250_BIT_A_DLPFCFG_5HZ);

	alti = BME280_CalcTf(pres);
	HAL_Delay(1000);
	rslt = bmp280_get_uncomp_data(&ucomp_data, &bmp);
	rslt = bmp280_get_comp_pres_double(&pres, ucomp_data.uncomp_press, &bmp);
	alti = BME280_CalcTf(pres);
//kalman.Eest = alti;
    kalman.ESTt0 = 0.0;
    kalman.ESTt = 0.0;

	while (!bmpStatus.im_update){
		bmp280Period = bmp280_get_status(&bmpStatus, &bmp);
	}


	averageAltiOld = alti;
	timer_new = uwTick;
	while (1) {
		rslt = bmp280_get_uncomp_data(&ucomp_data, &bmp);
		rslt = bmp280_get_comp_pres_double(&pres, ucomp_data.uncomp_press, &bmp);
    	rslt = bmp280_get_comp_temp_32bit(&temp,ucomp_data.uncomp_temp,&bmp);
//		MPU9250_drv_read_gyro(&gyro);
//		MPU9250_drv_read_accel(&accel);

//		alti = BME280_CalcTf(pres);

		alti = BME280_CalcTf(pres);
//		alti = simpleKalman(&kalman,BME280_CalcTf(pres));
		if (arrayIndex < ALTITUDE_SAMPLES_FILTER) {
			arrayIndex ++;
			averageAlti = averageAlti + alti;
		} else {
			averageAlti = averageAlti/ALTITUDE_SAMPLES_FILTER;
			if (timer) {
				speed = (averageAlti - averageAltiOld)/((double)timer /1000);
				averageSpeed = simpleKalman(&kalman,speed);
				tone.averageSpeed = averageSpeed;
			}
			timer_new = uwTick;
			if (timer_new > timer_old) {
				timer = timer_new - timer_old;
			} else {
				timer = timer_old - timer_new;
			}
			timer_old = timer_new;

			//printf("$%.4f %f \r\n", averageSpeed, averageAlti);
			/*
			getVarioTone(averageSpeed, &tone);
			arrayIndex = 0;
			averageAltiOld = averageAlti;
			averageAlti = 0.0;
			*/
		}

		if (HAL_GPIO_ReadPin(USER_LED_PORT, USER_LED_PIN)) {
			if (buttonPressed == 0) {
				buttonPressed = 1;
				TimStart(&tim3,(12000000/tone.toneFreq));
				soundCounter ++;
			}
		} else {
			buttonPressed = 0;
			TimStop();
		}

    	char sPres[8];
		sprintf(sPres, "%x", pres);

		printf("PRS %x\n",pres);
		SSD1306_GotoXY(0,0);
		SSD1306_Puts(sPres,&Font_7x10,1);
		SSD1306_UpdateScreen();
		/*
		if (averageSpeed > 0.2)
			TimStart(&tim3,(12000000/tone.toneFreq));
		else
			TimStop();
		*/
		HAL_Delay(10);
	}
}

/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow :
 *            System Clock source            = PLL (HSI48)
 *            SYSCLK(Hz)                     = 48000000
 *            HCLK(Hz)                       = 48000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 1
 *            HSI Frequency(Hz)              = 48000000
 *            PREDIV                         = 2
 *            PLLMUL                         = 2
 *            Flash Latency(WS)              = 1
 * @param  None
 * @retval None
 */
static void SystemClock_Config(void) {
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;

	/* Select HSI48 Oscillator as PLL source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI48;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/* Select PLL as system clock source and configure the HCLK and PCLK1 clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
static void Error_Handler(void) {
	/* User may add here some code to deal with this error */
	while (1) {
	}
}

int __io_putchar(int ch) {
	HAL_USART_Transmit(&USART_DEBUG_HandleStruct, (uint8_t*) &ch, 1, 0xFFFF);
	return ch;
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
 * @}
 */
double BME280_CalcTf(double pressure) {
    return (44330 * (1.0 - pow(((pressure / 100) / 1013.25), 0.1903)));
}
/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
