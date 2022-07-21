/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define I2C_TIMEOUT_BASE 10
#define I2C_TIMEOUT_BYTE 1
/// AS5600地址
#define AS5600_RAW_ADDR 0X36
/// STM32 HAL库中的设备地址需要源地址左移一位
#define AS5600_ADDR (AS5600_RAW_ADDR << 1)
#define AS5600_WR_ADDR (AS5600_RAW_ADDR << 1)
#define AS5600_RD_ADDR (AS5600_RAW_ADDR << 1 | 1)
/// AS5600编码器精度
#define AS5600_RESOLUTION 4096//12bit/lines resolution
#define AS5600_RAW_ANGLE_REG 0X0C

#define abs(x) ((x)>0?(x):(-x))
#define _2PI 6.28318530718
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
/// 上一次编码器值
static double angle_data_prev;
/// 编码器偏移值
static double full_rotation_offset;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void MX_GPIO_Init(void);

static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
extern void initialise_monitor_handles();

void as5600_init(void);

uint16_t as5600GetRawAngle(void);

double as5600GetAngle(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * 初始化
 */
void as5600_init(void) {
    full_rotation_offset = 0;
    angle_data_prev = as5600GetAngle();
}
/**
 * 写I2C设备寄存器值
 * @param dev_addr 设备I2C地址
 * @param pData 数据
 * @param count 大小
 * @return 寄存器值
 */
static int i2cWrite(uint8_t dev_addr, uint8_t *pData, uint32_t count) {
    int status;
    int i2c_timeout = I2C_TIMEOUT_BASE + count * I2C_TIMEOUT_BYTE;

    status = HAL_I2C_Master_Transmit(&hi2c1, dev_addr, pData, count, i2c_timeout);
    return status;
}
/**
 * 读取I2C设备寄存器值
 * @param dev_addr 设备I2C地址
 * @param pData 数据
 * @param count 大小
 * @return 寄存器值
 */
static int i2cRead(uint8_t dev_addr, uint8_t *pData, uint32_t count) {
    int status;
    int i2c_timeout = I2C_TIMEOUT_BASE + count * I2C_TIMEOUT_BYTE;
    status = HAL_I2C_Master_Receive(&hi2c1, (dev_addr | 1), pData, count, i2c_timeout);
    return status;
}
/**
 * 获取原始角度值0~4095
 * @return 单圈角度值
 */
uint16_t as5600GetRawAngle(void) {
    uint16_t raw_angle;
    uint8_t buffer[2] = {0};
    uint8_t raw_angle_reg = AS5600_RAW_ANGLE_REG;

    i2cWrite(AS5600_ADDR, &raw_angle_reg, 1);
    i2cRead(AS5600_ADDR, buffer, 2);
    raw_angle = ((uint16_t) buffer[0] << 8) | (uint16_t) buffer[1];
    return raw_angle;
}
/**
 * 获取经过转换后的角度，支持多圈
 * @return 多圈角度值
 */
double as5600GetAngle(void) {
    double angle_data = as5600GetRawAngle();

    double d_angle = angle_data - angle_data_prev;
    if (abs(d_angle) > (0.8 * AS5600_RESOLUTION)) {
        full_rotation_offset += (double) (d_angle > 0 ? -_2PI : _2PI);
    }
    angle_data_prev = angle_data;
    return (full_rotation_offset + (angle_data / AS5600_RESOLUTION) * _2PI);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
    /* USER CODE BEGIN 1 */
    initialise_monitor_handles();

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
    MX_GPIO_Init();
    MX_I2C1_Init();
    /* USER CODE BEGIN 2 */
    as5600_init();
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        uint16_t angle_raw = as5600GetRawAngle();
        double angle = as5600GetAngle();
        HAL_Delay(100);
        printf("%d, %f\n", angle_raw, angle);
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void) {

    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 400000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
