/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "adc.h"
#include "fdcan.h"
#include "i2c.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */
#define TMP101_I2C_ADDR_7BIT        0x48U
#define TMP101_REG_TEMP             0x00U

#define LIS3DH_WHO_AM_I_REG         0x0FU
#define LIS3DH_CTRL_REG1_REG        0x20U
#define LIS3DH_CTRL_REG4_REG        0x23U
#define LIS3DH_OUT_X_L_REG          0x28U

#define LIS3DH_SPI_READ_BIT         0x80U
#define LIS3DH_SPI_MS_BIT           0x40U
#define LIS3DH_WHO_AM_I_VAL         0x33U

#define LIS3DH_CTRL_REG1_100HZ_XYZ  0x57U
#define LIS3DH_CTRL_REG4_2G_HR      0x88U

typedef struct
{
  int16_t x_raw;
  int16_t y_raw;
  int16_t z_raw;
  float x_g;
  float y_g;
  float z_g;
} LIS3DH_AccelData_t;

uint8_t tmp101_raw[2] = { 0 };
float temperature_c = 0.0f;
HAL_StatusTypeDef tmp101_status = HAL_ERROR;

uint8_t lis3dh_who_am_i = 0;
HAL_StatusTypeDef lis3dh_spi_status = HAL_ERROR;
HAL_StatusTypeDef lis3dh_init_status = HAL_ERROR;
HAL_StatusTypeDef lis3dh_xyz_status = HAL_ERROR;
LIS3DH_AccelData_t lis3dh_accel = { 0 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static float TMP101_ReadCelsius(I2C_HandleTypeDef *hi2c, HAL_StatusTypeDef *status)
{
  HAL_StatusTypeDef ret;

  if ((hi2c == NULL) || (status == NULL))
  {
    if (status != NULL)
    {
      *status = HAL_ERROR;
    }
    return 0.0f;
  }

  ret = HAL_I2C_Mem_Read(hi2c, (uint16_t)(TMP101_I2C_ADDR_7BIT << 1),
                         TMP101_REG_TEMP, I2C_MEMADD_SIZE_8BIT,
                         tmp101_raw, 2U, 100U);
  *status = ret;

  if (ret != HAL_OK)
  {
    return 0.0f;
  }

  /* Convert raw 12-bit temperature register data to degrees Celsius */
  int16_t raw_12bit = (int16_t)(((uint16_t)tmp101_raw[0] << 8) | tmp101_raw[1]) >> 4;
  if (raw_12bit & 0x0800)
  {
    raw_12bit |= (int16_t)0xF000;
  }

  return (float)raw_12bit * 0.0625f;
}

static HAL_StatusTypeDef LIS3DH_ReadWhoAmI(SPI_HandleTypeDef *hspi, uint8_t *p_id)
{
  uint8_t tx_data[2] = { (uint8_t)(LIS3DH_SPI_READ_BIT | LIS3DH_WHO_AM_I_REG), 0x00U };
  uint8_t rx_data[2] = { 0x00U, 0x00U };
  HAL_StatusTypeDef status;

  if (p_id == NULL)
  {
    return HAL_ERROR;
  }

  /* Assert CS low to start SPI transaction */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
  status = HAL_SPI_TransmitReceive(hspi, tx_data, rx_data, 2U, 100U);
  /* Deassert CS high after SPI transaction completes */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

  if (status == HAL_OK)
  {
    *p_id = rx_data[1];
  }

  return status;
}

static HAL_StatusTypeDef LIS3DH_WriteReg(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t value)
{
  uint8_t tx_data[2] = { (uint8_t)(reg & 0x3FU), value };
  HAL_StatusTypeDef status;

  /* Assert CS low to start SPI write transaction */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
  status = HAL_SPI_Transmit(hspi, tx_data, 2U, 100U);
  /* Deassert CS high after SPI write transaction */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

  return status;
}

static HAL_StatusTypeDef LIS3DH_InitAccel(SPI_HandleTypeDef *hspi)
{
  HAL_StatusTypeDef status;

  status = LIS3DH_WriteReg(hspi, LIS3DH_CTRL_REG4_REG, LIS3DH_CTRL_REG4_2G_HR);
  if (status != HAL_OK)
  {
    return status;
  }

  return LIS3DH_WriteReg(hspi, LIS3DH_CTRL_REG1_REG, LIS3DH_CTRL_REG1_100HZ_XYZ);
}

static HAL_StatusTypeDef LIS3DH_ReadAccelXYZ(SPI_HandleTypeDef *hspi, LIS3DH_AccelData_t *p_accel)
{
  /* Read 6 consecutive bytes starting at OUT_X_L (0x28) with READ + MS bits set */
  uint8_t tx_data[7] = {
    (uint8_t)(LIS3DH_SPI_READ_BIT | LIS3DH_SPI_MS_BIT | LIS3DH_OUT_X_L_REG),
    0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U
  };
  uint8_t rx_data[7] = { 0 };
  HAL_StatusTypeDef status;

  if (p_accel == NULL)
  {
    return HAL_ERROR;
  }

  /* Assert CS low to start multi-byte SPI read transaction */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
  status = HAL_SPI_TransmitReceive(hspi, tx_data, rx_data, 7U, 100U);
  /* Deassert CS high after multi-byte SPI read transaction */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

  if (status == HAL_OK)
  {
    /* rx_data[1..2]: X_L, X_H | rx_data[3..4]: Y_L, Y_H | rx_data[5..6]: Z_L, Z_H */
    p_accel->x_raw = (int16_t)(((uint16_t)rx_data[2] << 8) | rx_data[1]);
    p_accel->y_raw = (int16_t)(((uint16_t)rx_data[4] << 8) | rx_data[3]);
    p_accel->z_raw = (int16_t)(((uint16_t)rx_data[6] << 8) | rx_data[5]);

    /* In +/-2g mode, scale raw value to approximate g */
    p_accel->x_g = ((float)p_accel->x_raw / 16.0f) * 0.001f;
    p_accel->y_g = ((float)p_accel->y_raw / 16.0f) * 0.001f;
    p_accel->z_g = ((float)p_accel->z_raw / 16.0f) * 0.001f;
  }

  return status;
}
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
  MX_ADC1_Init();
  MX_FDCAN1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  lis3dh_spi_status = LIS3DH_ReadWhoAmI(&hspi1, &lis3dh_who_am_i);
  lis3dh_init_status = LIS3DH_InitAccel(&hspi1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
    temperature_c = TMP101_ReadCelsius(&hi2c1, &tmp101_status);
    lis3dh_xyz_status = LIS3DH_ReadAccelXYZ(&hspi1, &lis3dh_accel);
    HAL_Delay(500);
    /* USER CODE END 3 */
  }
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
#ifdef USE_FULL_ASSERT
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
