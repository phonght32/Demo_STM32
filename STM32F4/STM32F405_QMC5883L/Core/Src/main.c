/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "qmc5883l.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2C_ADDR_QMC5883L                   (QMC5883L_I2C_ADDR<<1)
#define QMC5883L_I2C                        hi2c2

#define UART_DEBUG                          huart1

#define CONFIG_QMC5883L_RANGE               QMC5883L_RANGE_8G
#define CONFIG_QMC5883L_OPR_MODE            QMC5883L_OPR_MODE_CONTINUOUS
#define CONFIG_QMC5883L_DATA_RATE           QMC5883L_DATA_RATE_50HZ
#define CONFIG_QMC5883L_SAMPLES             QMC5883L_SAMPLE_RATE_512
#define CONFIG_QMC5883L_INTR_ENABLE         QMC5883L_INTERRUPT_DISABLE
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
qmc5883l_handle_t qmc5883l_handle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
err_code_t hw_intf_uart_debug_send(uint8_t *log_buf);
err_code_t hw_intf_qmc5883l_i2c_send(uint8_t reg_addr, uint8_t *buf, uint16_t len);
err_code_t hw_intf_qmc5883l_i2c_recv(uint8_t reg_addr, uint8_t *buf, uint16_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

    /* USER CODE BEGIN 1 */

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
    MX_I2C2_Init();
    MX_USART1_UART_Init();
    /* USER CODE BEGIN 2 */
    qmc5883l_handle = qmc5883l_init();
    qmc5883l_cfg_t qmc5883l_cfg = {
        .range          = CONFIG_QMC5883L_RANGE,
        .opr_mode       = CONFIG_QMC5883L_OPR_MODE,
        .data_rate      = CONFIG_QMC5883L_DATA_RATE,
        .sample_rate    = CONFIG_QMC5883L_SAMPLES,
        .intr_en        = CONFIG_QMC5883L_INTR_ENABLE,
        .hard_bias_x     = 154.7851961238576,
        .hard_bias_y     = -12.923460697083387,
        .hard_bias_z     = -16.002331463439372,
        .soft_bias_c11   = 0.6232115045089357,
        .soft_bias_c12   = 0.004997471269166363,
        .soft_bias_c13   = 0.022147854060797133,
        .soft_bias_c21   = 0.004997471269166429,
        .soft_bias_c22   = 0.6324010352739647,
        .soft_bias_c23   = -0.00080240879779977,
        .soft_bias_c31   = 0.02214785406079713,
        .soft_bias_c32   = -0.000802408797799873,
        .soft_bias_c33   = 0.7179876576597246,
        .i2c_send       = hw_intf_qmc5883l_i2c_send,
        .i2c_recv       = hw_intf_qmc5883l_i2c_recv,
        .delay          = HAL_Delay
    };
    qmc5883l_set_config(qmc5883l_handle, qmc5883l_cfg);
    qmc5883l_config(qmc5883l_handle);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {

        // int16_t raw_mag_x = 0, raw_max_y = 0, raw_mag_z = 0;
        // qmc5883l_get_mag_raw(qmc5883l_handle, &raw_mag_x, &raw_max_y, &raw_mag_z);

        // uint8_t log_buf[100];
        // sprintf((char *)log_buf, "\n%i,%i,%i", raw_mag_x, raw_max_y, raw_mag_z);
        // hw_intf_uart_debug_send(log_buf);

        float calib_mag_x = 0, calib_max_y = 0, calib_mag_z = 0;
        qmc5883l_get_mag_calib(qmc5883l_handle, &calib_mag_x, &calib_max_y, &calib_mag_z);

        uint8_t log_buf[100];
        sprintf((char *)log_buf, "\n%f,%f,%f", calib_mag_x, calib_max_y, calib_mag_z);
        hw_intf_uart_debug_send(log_buf);

        HAL_Delay(100);
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
err_code_t hw_intf_qmc5883l_i2c_send(uint8_t reg_addr, uint8_t *buf, uint16_t len)
{
    uint8_t buf_send[len + 1];
    buf_send[0] = reg_addr;
    for (uint8_t i = 0; i < len; i++)
    {
        buf_send[i + 1] = buf[i];
    }

    HAL_I2C_Master_Transmit(&QMC5883L_I2C, I2C_ADDR_QMC5883L, buf_send, len + 1, 100);

    return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_qmc5883l_i2c_recv(uint8_t reg_addr, uint8_t *buf, uint16_t len)
{
    uint8_t buffer[1];
    buffer[0] = reg_addr;

    HAL_I2C_Master_Transmit(&QMC5883L_I2C, I2C_ADDR_QMC5883L, buffer, 1, 100);
    HAL_I2C_Master_Receive(&QMC5883L_I2C, I2C_ADDR_QMC5883L, buf, len, 100);

    return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_uart_debug_send(uint8_t *log_buf)
{
    uint16_t len = strlen((char*)log_buf);
    HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)log_buf, len, 100);

    return ERR_CODE_SUCCESS;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
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
