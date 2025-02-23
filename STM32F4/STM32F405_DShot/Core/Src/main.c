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
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "err_code.h"
#include "esc_dshot.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FL_ESC_DSHOT_TIM                htim3
#define FL_ESC_DSHOT_TIM_CHNL           TIM_CHANNEL_3

#define FR_ESC_DSHOT_TIM                htim3
#define FR_ESC_DSHOT_TIM_CHNL           TIM_CHANNEL_4

#define BL_ESC_DSHOT_TIM                htim5
#define BL_ESC_DSHOT_TIM_CHNL           TIM_CHANNEL_4

#define BR_ESC_DSHOT_TIM                htim2
#define BR_ESC_DSHOT_TIM_CHNL           TIM_CHANNEL_3

#define DMA_BUF_SIZE                    18
#define CONFIG_ESC_DSHOT_TYPE           ESC_DSHOT_TYPE_600
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
esc_dshot_handle_t fl_esc_dshot_handle;
esc_dshot_handle_t fr_esc_dshot_handle;
esc_dshot_handle_t bl_esc_dshot_handle;
esc_dshot_handle_t br_esc_dshot_handle;

uint32_t fl_esc_dshot_dmabuffer[DMA_BUF_SIZE];
uint32_t fr_esc_dshot_dmabuffer[DMA_BUF_SIZE];
uint32_t bl_esc_dshot_dmabuffer[DMA_BUF_SIZE];
uint32_t br_esc_dshot_dmabuffer[DMA_BUF_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
err_code_t hw_intf_fl_esc_dshot_send_dma(uint32_t *packet_dma);
err_code_t hw_intf_fr_esc_dshot_send_dma(uint32_t *packet_dma);
err_code_t hw_intf_bl_esc_dshot_send_dma(uint32_t *packet_dma);
err_code_t hw_intf_br_esc_dshot_send_dma(uint32_t *packet_dma);
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
    MX_DMA_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM5_Init();
    /* USER CODE BEGIN 2 */
    fl_esc_dshot_handle = esc_dshot_init();
    fr_esc_dshot_handle = esc_dshot_init();
    bl_esc_dshot_handle = esc_dshot_init();
    br_esc_dshot_handle = esc_dshot_init();

    esc_dshot_cfg_t fl_esc_dshot_cfg = {
        .dshot_type = CONFIG_ESC_DSHOT_TYPE,
        .tick_bit   = 560,
        .send_dma   = hw_intf_fl_esc_dshot_send_dma
    };

    esc_dshot_cfg_t fr_esc_dshot_cfg = {
        .dshot_type = CONFIG_ESC_DSHOT_TYPE,
        .tick_bit   = 560,
        .send_dma   = hw_intf_fr_esc_dshot_send_dma
    };

    esc_dshot_cfg_t bl_esc_dshot_cfg = {
        .dshot_type = CONFIG_ESC_DSHOT_TYPE,
        .tick_bit   = 560,
        .send_dma   = hw_intf_bl_esc_dshot_send_dma
    };

    esc_dshot_cfg_t br_esc_dshot_cfg = {
        .dshot_type = CONFIG_ESC_DSHOT_TYPE,
        .tick_bit   = 560,
        .send_dma   = hw_intf_br_esc_dshot_send_dma
    };

    esc_dshot_set_config(fl_esc_dshot_handle, fl_esc_dshot_cfg);
    esc_dshot_set_config(fr_esc_dshot_handle, fr_esc_dshot_cfg);
    esc_dshot_set_config(bl_esc_dshot_handle, bl_esc_dshot_cfg);
    esc_dshot_set_config(br_esc_dshot_handle, br_esc_dshot_cfg);

    esc_dshot_config(fl_esc_dshot_handle);
    esc_dshot_config(fr_esc_dshot_handle);
    esc_dshot_config(bl_esc_dshot_handle);
    esc_dshot_config(br_esc_dshot_handle);

    uint16_t throttle = 500;
    esc_dshot_prepare_packet(fl_esc_dshot_handle, throttle, fl_esc_dshot_dmabuffer);
    esc_dshot_prepare_packet(fr_esc_dshot_handle, throttle, fr_esc_dshot_dmabuffer);
    esc_dshot_prepare_packet(bl_esc_dshot_handle, throttle, bl_esc_dshot_dmabuffer);
    esc_dshot_prepare_packet(br_esc_dshot_handle, throttle, br_esc_dshot_dmabuffer);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        esc_dshot_send_packet(fl_esc_dshot_handle, fl_esc_dshot_dmabuffer);
        esc_dshot_send_packet(fr_esc_dshot_handle, fr_esc_dshot_dmabuffer);
        esc_dshot_send_packet(bl_esc_dshot_handle, bl_esc_dshot_dmabuffer);
        esc_dshot_send_packet(br_esc_dshot_handle, br_esc_dshot_dmabuffer);

        HAL_Delay(500);
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
err_code_t hw_intf_fl_esc_dshot_send_dma(uint32_t *packet_dma)
{
    HAL_TIM_PWM_Start_DMA(&FL_ESC_DSHOT_TIM, FL_ESC_DSHOT_TIM_CHNL, packet_dma, DMA_BUF_SIZE);
    return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_fr_esc_dshot_send_dma(uint32_t *packet_dma)
{
    HAL_TIM_PWM_Start_DMA(&FR_ESC_DSHOT_TIM, FR_ESC_DSHOT_TIM_CHNL, packet_dma, DMA_BUF_SIZE);
    return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_bl_esc_dshot_send_dma(uint32_t *packet_dma)
{
    HAL_TIM_PWM_Start_DMA(&BL_ESC_DSHOT_TIM, BL_ESC_DSHOT_TIM_CHNL, packet_dma, DMA_BUF_SIZE);
    return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_br_esc_dshot_send_dma(uint32_t *packet_dma)
{
    HAL_TIM_PWM_Start_DMA(&BR_ESC_DSHOT_TIM, BR_ESC_DSHOT_TIM_CHNL, packet_dma, DMA_BUF_SIZE);
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
