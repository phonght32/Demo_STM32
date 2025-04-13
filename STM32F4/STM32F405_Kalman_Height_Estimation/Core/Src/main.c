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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "err_code.h"
#include "bmp280.h"
#include "icm42688.h"
#include "kalman_height_estimation.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define CONFIG_BMP280_OPR_MODE              BMP280_OPR_MODE_NORMAL
#define CONFIG_BMP280_FILTER                BMP280_FILTER_OFF
#define CONFIG_BMP280_OVER_SAMPLING_TEMP    BMP280_OVER_SAMPLING_STANDARD
#define CONFIG_BMP280_OVER_SAMPLING_HUMD    BMP280_OVER_SAMPLING_STANDARD
#define CONFIG_BMP280_OVER_SAMPLING_PRES    BMP280_OVER_SAMPLING_STANDARD
#define CONFIG_BMP280_STANDBY_TIME          BMP280_STANDBY_TIME_0_5MS
#define CONFIG_BMP280_COMM_MODE             BMP280_COMM_MODE_I2C

#define CONFIG_ICM42688_GYRO_MODE           ICM42688_GYRO_MODE_LOW_NOISE
#define CONFIG_ICM42688_GYRO_FS_SEL         ICM42688_GFS_SEL_2000dps
#define CONFIG_ICM42688_GYRO_ODR            ICM42688_GYRO_ODR_1kHz
#define CONFIG_ICM42688_ACCEL_MODE          ICM42688_ACCEL_MODE_LOW_NOISE
#define CONFIG_ICM42688_ACCEL_FS_SEL        ICM42688_ACCEL_FS_SEL_16G
#define CONFIG_ICM42688_ACCEL_ODR           ICM42688_ACCEL_ODR_1kHz
#define CONFIG_ICM42688_COMM_MODE           ICM42688_COMM_MODE_SPI

#define ICM42688_SPI                        hspi1
#define ICM42688_GPIO_PORT_CS               GPIOA
#define ICM42688_GPIO_PIN_CS                GPIO_PIN_4

#define BMP280_I2C                          hi2c2
#define I2C_ADDR_BMP280                     (BMP280_I2C_ADDR_0<<1)

#define UART_DEBUG                          huart1
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
bmp280_handle_t bmp280_handle = NULL;
icm42688_handle_t icm42688_handle;
kalman_height_estimation_handle_t kalman_height_estimation_handle = NULL;
uint8_t log_buf[100];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
err_code_t hw_intf_icm42688_spi_send(uint8_t *buf_send, uint16_t len);
err_code_t hw_intf_icm42688_spi_recv(uint8_t *buf_recv, uint16_t len);
err_code_t hw_intf_icm42688_set_cs(uint8_t level);
err_code_t hw_intf_bmp280_i2c_send(uint8_t reg_addr, uint8_t *buf, uint16_t len);
err_code_t hw_intf_bmp280_i2c_recv(uint8_t reg_addr, uint8_t *buf, uint16_t len);
err_code_t hw_intf_uart_debug_send(uint8_t *log_buf);
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
    MX_SPI1_Init();
    MX_USART1_UART_Init();
    /* USER CODE BEGIN 2 */
    icm42688_handle = icm42688_init();
    icm42688_cfg_t icm42688_cfg = {
        .gyro_mode      = CONFIG_ICM42688_GYRO_MODE,
        .gyro_fs_sel    = CONFIG_ICM42688_GYRO_FS_SEL,
        .gyro_odr       = CONFIG_ICM42688_GYRO_ODR,
        .accel_mode     = CONFIG_ICM42688_ACCEL_MODE,
        .accel_fs_sel   = CONFIG_ICM42688_ACCEL_FS_SEL,
        .accel_odr      = CONFIG_ICM42688_ACCEL_ODR,
        .accel_bias_x   = 19,
        .accel_bias_y   = 21,
        .accel_bias_z   = 26,
        .gyro_bias_x    = 12,
        .gyro_bias_y    = 0,
        .gyro_bias_z    = 17,
        .comm_mode      = CONFIG_ICM42688_COMM_MODE,
        .spi_send       = hw_intf_icm42688_spi_send,
        .spi_recv       = hw_intf_icm42688_spi_recv,
        .set_cs         = hw_intf_icm42688_set_cs,
        .delay          = HAL_Delay
    };
    icm42688_set_config(icm42688_handle, icm42688_cfg);
    icm42688_config(icm42688_handle);

    bmp280_cfg_t bmp280_cfg = {
        .opr_mode                   = CONFIG_BMP280_OPR_MODE,
        .filter                     = CONFIG_BMP280_FILTER,
        .over_sampling_pressure     = CONFIG_BMP280_OVER_SAMPLING_PRES,
        .over_sampling_temperature  = CONFIG_BMP280_OVER_SAMPLING_TEMP,
        .over_sampling_humidity     = CONFIG_BMP280_OVER_SAMPLING_HUMD,
        .standby_time               = CONFIG_BMP280_STANDBY_TIME,
        .comm_mode                  = CONFIG_BMP280_COMM_MODE,
        .i2c_send                   = hw_intf_bmp280_i2c_send,
        .i2c_recv                   = hw_intf_bmp280_i2c_recv,
        .delay                      = HAL_Delay,
    };
    bmp280_handle = bmp280_init();
    bmp280_set_config(bmp280_handle, bmp280_cfg);
    bmp280_config(bmp280_handle);

    kalman_height_estimation_handle = kalman_height_estimation_init();
    kalman_height_estimation_cfg_t kalman_height_estimation_cfg = {
        .dt = 0.04f,
    };
    kalman_height_estimation_set_config(kalman_height_estimation_handle, kalman_height_estimation_cfg);
    kalman_height_estimation_config(kalman_height_estimation_handle);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        float pressure = 0.0;
        float altitude = 0.0, altitude_kalman = 0.0;
        float accel_x, accel_y, accel_z;

        bmp280_get_pressure(bmp280_handle, &pressure);
        bmp280_convert_pressure_to_altitude(bmp280_handle, pressure / 100, &altitude);

        icm42688_get_accel_scale(icm42688_handle, &accel_x, &accel_y, &accel_z);

        kalman_height_estimation_update(kalman_height_estimation_handle, accel_z, altitude);
        kalman_height_estimation_get_height(kalman_height_estimation_handle, &altitude_kalman);

//        sprintf((char *)log_buf, "\n%f,%f,%f,%f,%f,%f,%f,%f,%f,0",
//                accel_x, accel_y, accel_z,
//                0.0f, 0.0f, 0.0f,
//				0.0f, 0.0f, 0.0f);

        sprintf((char *)log_buf, "%f,%f\n", pressure, altitude_kalman);

        hw_intf_uart_debug_send(log_buf);

        HAL_Delay(40);
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
err_code_t hw_intf_icm42688_spi_send(uint8_t *buf_send, uint16_t len)
{
    HAL_SPI_Transmit(&ICM42688_SPI, buf_send, len, 100);

    return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_icm42688_spi_recv(uint8_t *buf_recv, uint16_t len)
{
    HAL_SPI_Receive(&ICM42688_SPI, buf_recv, len, 100);

    return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_icm42688_set_cs(uint8_t level)
{
    HAL_GPIO_WritePin(ICM42688_GPIO_PORT_CS, ICM42688_GPIO_PIN_CS, level);

    return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_bmp280_i2c_send(uint8_t reg_addr, uint8_t *buf, uint16_t len)
{
    uint8_t buf_send[len + 1];
    buf_send[0] = reg_addr;
    for (uint8_t i = 0; i < len; i++)
    {
        buf_send[i + 1] = buf[i];
    }

    HAL_I2C_Master_Transmit(&BMP280_I2C, I2C_ADDR_BMP280, buf_send, len + 1, 100);

    return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_bmp280_i2c_recv(uint8_t reg_addr, uint8_t *buf, uint16_t len)
{
    uint8_t buffer[1];
    buffer[0] = reg_addr;

    HAL_I2C_Master_Transmit(&BMP280_I2C, I2C_ADDR_BMP280, buffer, 1, 100);
    HAL_I2C_Master_Receive(&BMP280_I2C, I2C_ADDR_BMP280, buf, len, 100);

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
