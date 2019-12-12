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
#include "cmsis_os.h"
#include <stdio.h>  /*rtt*/
#include <stdlib.h> /*rtt*/
#include "FreeRTOSConfig.h"
#include "cmsis_os.h"

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
osThreadId printHandle1;
osThreadId printHandle2;

void printTask1(void *argument);
void printTask2(void *argument);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main(void) {

	/* STM32F0xx HAL library initialization:
	 - Configure the Flash prefetch
	 - Systick timer is configured by default as source of time base, but user
	 can eventually implement his proper time base source (a general purpose
	 timer for example or other time source), keeping in mind that Time base
	 duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
	 handled in milliseconds basis.
	 - Low Level Initialization
	 */
	HAL_Init();

	/* Configure the system clock to have a system clock = 48 Mhz */
	SystemClock_Config();

	USART_DBG_Init();
	SSD1306_Init();
	MX_I2C1_Init();
//  X_TIM3_Init();

	/* Add your application code here
	 */
#if 1
    int rslt;
    struct bmp280_dev bmp;
    struct bmp280_config conf;
    struct bmp280_uncomp_data ucomp_data;
    int32_t temp = 0;
    double alti = 0.0;
    double pres;
    MPU9250_gyro_val gyro;
    MPU9250_accel_val accel;
    MPU9250_magnetometer_val magn;

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
	conf.os_temp = BMP280_OS_2X;
	conf.odr = BMP280_ODR_0_5_MS;
	rslt = bmp280_set_config(&conf, &bmp);

	/* Always set the power mode after setting the configuration */
	rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);
	SSD1306_GotoXY(0, 0);
	char cAlti[12];
	char cTemp[12];
	char cPress[12];
	char cGyroX[12];
	char cGyroY[12];
	char cGyroZ[12];
	char cAccelX[12];
	char cAccelY[12];
	char cAccelZ[12];
	SSD1306_UpdateScreen();

	rslt = MPU9250_drv_init();
	if (rslt == MPU9250_OK) {
		SSD1306_Puts("MPU9250 Ok", &Font_7x10, SSD1306_COLOR_WHITE);
	}
	if (rslt == MPU9250_NOT_FOUND) {
		SSD1306_Puts("MPU9250 Not found", &Font_7x10, SSD1306_COLOR_WHITE);
	}
	if (rslt == MPU9250_INIT_ERROR) {
		SSD1306_Puts("MPU9250 Init Err", &Font_7x10, SSD1306_COLOR_WHITE);
	}
	if (rslt == AK8963_INIT_ERROR) {
		SSD1306_Puts("AK8963 Not found", &Font_7x10, SSD1306_COLOR_WHITE);
	}
	SSD1306_UpdateScreen();
	HAL_Delay(2000);
	MPU9250_drv_start_maesure(MPU9250_BIT_GYRO_FS_SEL_250DPS,MPU9250_BIT_ACCEL_FS_SEL_8G,MPU9250_BIT_DLPF_CFG_5HZ,MPU9250_BIT_A_DLPFCFG_5HZ);
#endif
#if 0
	osThreadDef(firstTask, printTask1, osPriorityHigh, 0, 300);
	osThreadDef(secondTask, printTask2, osPriorityNormal, 0, 128);

	osThreadCreate(osThread(firstTask),NULL);
	osThreadCreate(osThread(secondTask),NULL);
	osKernelStart ();
#endif
	/* Infinite loop */
	while (1) {
#if 1
		rslt = bmp280_get_uncomp_data(&ucomp_data, &bmp);
		rslt = bmp280_get_comp_pres_double(&pres, ucomp_data.uncomp_press, &bmp);
  		rslt = bmp280_get_comp_temp_32bit(&temp,ucomp_data.uncomp_temp,&bmp);
		MPU9250_drv_read_gyro(&gyro);
		MPU9250_drv_read_accel(&accel);
		MPU9250_drv_read_magnetometer(&magn);

		alti = BME280_CalcTf(pres);
/*
		sprintf(cAlti, "%.1f", alti);
		sprintf(cTemp, "%d", temp);
		sprintf(cPress, "%.3f", pres/100);
		sprintf(cGyroX, "%.3f", gyro.x);
		sprintf(cGyroY, "%.3f", gyro.y);
		sprintf(cGyroZ, "%.3f", gyro.z);
		sprintf(cAccelX, "%.3f", accel.x);
		sprintf(cAccelY, "%.3f", accel.y);
		sprintf(cAccelZ, "%.3f", accel.z);
		SSD1306_Fill(SSD1306_COLOR_BLACK);
		SSD1306_GotoXY(0, 0);
		SSD1306_Puts(cAlti, &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_GotoXY(0, 8);
		SSD1306_Puts(cTemp, &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_GotoXY(0, 16);
		SSD1306_Puts(cPress, &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_GotoXY(0, 24);
		SSD1306_Puts(cGyroX, &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_GotoXY(64, 24);
		SSD1306_Puts(cAccelX, &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_GotoXY(0, 32);
		SSD1306_Puts(cGyroY, &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_GotoXY(64, 32);
		SSD1306_Puts(cAccelY, &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_GotoXY(0, 40);
		SSD1306_Puts(cGyroZ, &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_GotoXY(64, 40);
		SSD1306_Puts(cAccelZ, &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_UpdateScreen();
		*/
		printf("$%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.2f %.2f %.2f\r\n",\
				alti, pres/100,gyro.x,gyro.y,gyro.z,accel.x,accel.y,accel.z, magn.x, magn.y, magn.z);
		bmp.delay_ms(20);
//		printf("\033[36maltitude\033[0m = %f \033[36mtemp\033[0m = %d\r\n",alti,temp);
#endif
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

void printTask1 (void *argument) {
    int rslt;
    struct bmp280_dev bmp;
    struct bmp280_config conf;
    struct bmp280_uncomp_data ucomp_data;
    int32_t temp = 0;
    double alti = 0.0;
    double pres;
    MPU9250_gyro_val gyro;
    MPU9250_accel_val accel;
    MPU9250_magnetometer_val magn;

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
	conf.os_temp = BMP280_OS_2X;
	conf.odr = BMP280_ODR_0_5_MS;
	rslt = bmp280_set_config(&conf, &bmp);

	/* Always set the power mode after setting the configuration */
	rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);
	rslt = MPU9250_drv_init();
	MPU9250_drv_start_maesure(MPU9250_BIT_GYRO_FS_SEL_250DPS,MPU9250_BIT_ACCEL_FS_SEL_8G,MPU9250_BIT_DLPF_CFG_5HZ,MPU9250_BIT_A_DLPFCFG_5HZ);

	for (;;) {
		rslt = bmp280_get_uncomp_data(&ucomp_data, &bmp);
		rslt = bmp280_get_comp_pres_double(&pres, ucomp_data.uncomp_press, &bmp);
  		rslt = bmp280_get_comp_temp_32bit(&temp,ucomp_data.uncomp_temp,&bmp);
		MPU9250_drv_read_gyro(&gyro);
		MPU9250_drv_read_accel(&accel);
		MPU9250_drv_read_magnetometer(&magn);

		alti = BME280_CalcTf(pres);
		printf("$%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.2f %.2f %.2f\r\n",\
				alti, pres/100,gyro.x,gyro.y,gyro.z,accel.x,accel.y,accel.z, magn.x, magn.y, magn.z);
		vTaskDelay(20);

	}
	osThreadTerminate(NULL);
}
void printTask2 (void *argument) {
	for (;;) {
		SSD1306_Fill(SSD1306_COLOR_BLACK);
		SSD1306_GotoXY(0, 0);
		SSD1306_Puts("Task 2", &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_UpdateScreen();
		vTaskDelay(1000);
	}
	osThreadTerminate(NULL);
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
