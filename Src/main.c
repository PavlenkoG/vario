/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#if defined(__GNUC__)
#include <stdio.h>  /*rtt*/
#include <stdlib.h> /*rtt*/
#endif
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "platform_def.h"
#include "bmp180.h"
#include "sdcard.h"
#include "ff.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

static I2C_HandleTypeDef I2C_HandleStruct;
static USART_HandleTypeDef USART_HandleStruct;
SPI_HandleTypeDef SPI_HandleStruct;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static void LED_GPIO_Init(void);
static void UART_Init();
static void I2C_GPIO_Init(void);
static void I2C_Init(void);
static void SPI_GPIO_Init(void);
static void SPI_Init(void);
float BME280_CalcTf(s32 UT);

s8 I2C_Read(u8 device_addr, u8 register_addr, u8* rdata, u8 len);
s8 I2C_Write(u8 device_addr, u8 register_addr, u8* rdata, u8 len);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
    /* USER CODE BEGIN 1 */
    u32 upress = 0;
    u16 utemp = 0;
    u32 press = 0;
    s16 temp = 0;
    float alti = 0.0;

    FATFS fs;
    FRESULT res;

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    /* USER CODE BEGIN 2 */
//  LED_GPIO_Init();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    UART_Init();
    I2C_GPIO_Init();
    I2C_Init();
    SPI_GPIO_Init();
    SPI_Init();
    __HAL_SPI_ENABLE(&SPI_HandleStruct);
    HAL_Delay(250);
    printf("Start\r\n");
    SDCARD_Unselect();
    int code = SDCARD_Init();
    if (code < 0) {
        printf("SDCARD_Init() failed, code = %d\r\n", code);
        return -1;
    }
    struct bmp180_t bmp180;
    bmp180.bus_read = I2C_Read;
    bmp180.bus_write = I2C_Write;
    bmp180.delay_msec = HAL_Delay;
    bmp180.dev_addr = 0xef;
    bmp180_init(&bmp180);
    bmp180.oversamp_setting = 3;
    bmp180.sw_oversamp = 1;

    res = f_mount(&fs, "", 0);
    if (res != FR_OK) {
        printf("f_mount() failed, res = %d\r\n", res);
    } else {
        printf("f_mount() done!\r\n", res);
    }
     printf("Reading file...\r\n");
     FIL msgFile;
     res = f_open(&msgFile, "hello.txt", FA_READ);
     if(res != FR_OK) {
         printf("f_open() failed, res = %d\r\n", res);
     }

     printf("File is opened...\r\n");
     char readBuff[128];
     unsigned int bytesRead;
     res = f_read(&msgFile, readBuff, sizeof(readBuff)-1, &bytesRead);
     if(res != FR_OK) {
         printf("f_read() failed, res = %d\r\n", res);
     }

     readBuff[bytesRead] = '\0';
     printf("```\r\n%s\r\n```\r\n", readBuff);

     res = f_close(&msgFile);
     if(res != FR_OK) {
         printf("f_close() failed, res = %d\r\n", res);
     }

    res = f_mount(NULL, "", 0);
    if (res != FR_OK) {
        printf("Unmount failed, res = %d\r\n", res);
    }
    printf("Done!\r\n");
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */
        HAL_Delay(500);
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        upress = bmp180_get_uncomp_pressure();
        utemp = bmp180_get_uncomp_temperature();
        press = bmp180_get_pressure(upress);
        temp = bmp180_get_temperature(utemp);
        alti = BME280_CalcTf(press);
        printf("PRS %x\r\n",press);

//      printf("\033[2J");
//      printf("\033[36mtemperature\033[0m = %d \033[36mpressure\033[0m = %d \033[36malti\033[0m = %f\r\n",temp,press,alti);
    }
    /* USER CODE BEGIN 3 */
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    /** Initializes the CPU, AHB and APB busses clocks
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB busses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
            | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
static void LED_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

static void I2C_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_I2C_BARO_GPIO_RCC_ENA
    ;
    GPIO_InitStruct.Pin = I2C_BARO_SDA_PIN | I2C_BARO_SCL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_I2C1;

    HAL_GPIO_Init(I2C_BARO_PORT, &GPIO_InitStruct);
}

static void SPI_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_SPI_INTF_GPIO_RCC_ENA;
    __HAL_SPI_CS_GPIO_RCC_ENA;

    GPIO_InitStruct.Pin = SPI_SCK_PIN | SPI_MISO_PIN | SPI_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
    HAL_GPIO_Init(SPI_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SPI_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SPI_CS_PORT, &GPIO_InitStruct);
}

static void SPI_Init(void) {
    __HAL_SPI1_RCC_ENA ;
    SPI_HandleStruct.Instance = SPI1;
    SPI_HandleStruct.Init.Mode = SPI_MODE_MASTER;
    SPI_HandleStruct.Init.Direction = SPI_DIRECTION_2LINES;
    SPI_HandleStruct.Init.DataSize = SPI_DATASIZE_8BIT;
    SPI_HandleStruct.Init.CLKPolarity = SPI_POLARITY_LOW;
    SPI_HandleStruct.Init.CLKPhase = SPI_PHASE_1EDGE;
    SPI_HandleStruct.Init.NSS = SPI_NSS_SOFT;
    SPI_HandleStruct.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    SPI_HandleStruct.Init.FirstBit = SPI_FIRSTBIT_MSB;
    SPI_HandleStruct.Init.TIMode = SPI_TIMODE_DISABLE;
    SPI_HandleStruct.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    SPI_HandleStruct.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&SPI_HandleStruct) != HAL_OK) {
        printf("SPI Init error \r\n");
    }
}

static void I2C_Init(void) {
    I2C_InitTypeDef I2C_InitStruct;
    __HAL_RCC_I2C1_CLK_ENABLE()
    ;
    I2C_InitStruct.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    I2C_InitStruct.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    I2C_InitStruct.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    I2C_InitStruct.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    I2C_InitStruct.Timing = (uint32_t) 0x00B00000;

    I2C_HandleStruct.Instance = I2C1;
    I2C_HandleStruct.Init = I2C_InitStruct;

    HAL_I2C_Init(&I2C_HandleStruct);

}
s8 I2C_Read(u8 device_addr, u8 register_addr, u8* rdata, u8 len) {

    uint8_t status = 0;
    status = HAL_I2C_Mem_Read(&I2C_HandleStruct, (uint8_t) device_addr,
            (uint8_t) register_addr, 1, (uint8_t*) rdata, (uint8_t) len, 100);
    return (u8) status;

}
s8 I2C_Write(u8 device_addr, u8 register_addr, u8* rdata, u8 len) {
    uint8_t status = 0;
    status = HAL_I2C_Mem_Write(&I2C_HandleStruct, (uint8_t) device_addr,
            (uint8_t) register_addr, 1, (uint8_t*) rdata, (uint8_t) len, 100);
    return (u8) status;
}

static void UART_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    __HAL_UART_DBG_GPIO_RCC_ENA ;
    GPIO_InitStruct.Pin = UART_DBG_RX_PIN | UART_DBG_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_USART2;

    HAL_GPIO_Init(UART_DBG_PORT, &GPIO_InitStruct);

    USART_InitStruct.BaudRate = 115200;
    USART_InitStruct.WordLength = 8;
    USART_InitStruct.StopBits = USART_STOPBITS_1;
    USART_InitStruct.Mode = USART_MODE_TX_RX;
    USART_InitStruct.Parity = USART_PARITY_NONE;

    USART_HandleStruct.Instance = USART2;
    USART_HandleStruct.Init = USART_InitStruct;
    HAL_USART_Init(&USART_HandleStruct);
}

float BME280_CalcTf(s32 pressure) {
    float altitude = 0.0;
    float fPressure = 1.0 * pressure;
    altitude = 44330 * (1.0 - pow((fPressure / 100) / 1013.25, 0.1903));
    return altitude;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */

    /* USER CODE END Error_Handler_Debug */
}

int __io_putchar(int ch) {
    HAL_USART_Transmit(&USART_HandleStruct, (uint8_t *) &ch, 1, 0xFFFF);
    return ch;
}
/**
 * @brief Retargets the C library printf function to the USART.
 * @param None
 * @retval None
 */
#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
