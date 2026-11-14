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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU_ADDR          (0x68 << 1)
#define GYRO_LSB_PER_DPS  131.0f   /* +/-250 dps (default). If you switch to 0x08 (+/-500) use 65.5f */
#define RAD2DEG           57.2957795f
#define ALPHA             0.98f    /* gyro trust. Lower = trust accel more */
#define ALPHA_HI          0.998f   /* gyro trust while accel is disturbed (|a| != 1 g) */
#define ACC_TOL_G         0.03f    /* how far |a| may stray from 1 g before accel is distrusted */
#define LOOP_MS           5        /* 200 Hz */
#define PRINT_EVERY       4        /* print at 200/4 = 50 Hz */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int32_t gx_bias = 0, gy_bias = 0, gz_bias = 0;
float roll_cf = 0.0f, pitch_cf = 0.0f;   /* filtered angles (deg) */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void mpu_write(uint8_t reg, uint8_t val)
{
    HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* Real elapsed time since the last call, in seconds (84 MHz core clock) */
static float dt_seconds(void)
{
    static uint32_t last = 0;
    uint32_t now = DWT->CYCCNT;
    float dt = (uint32_t)(now - last) / (float)SystemCoreClock;
    last = now;
    return dt;
}

/* Start the filter at the accel angle instead of 0 so it doesn't have to converge */
static void seed_filter(void)
{
    uint8_t b[14];
    if (HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, 0x3B, I2C_MEMADD_SIZE_8BIT, b, 14, 100) == HAL_OK)
    {
        float ax = (int16_t)((b[0] << 8) | b[1]);
        float ay = (int16_t)((b[2] << 8) | b[3]);
        float az = (int16_t)((b[4] << 8) | b[5]);
        roll_cf  = atan2f(ay, az) * RAD2DEG;
        pitch_cf = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD2DEG;
    }
}

void calibrate_gyro(void)
{
    int32_t sx = 0, sy = 0, sz = 0;
    uint8_t cb[14];
    char m[64];
    int l;
    const int N = 400;   /* 400 x 5 ms = 2 s, way faster than before */

    HAL_UART_Transmit(&huart2, (uint8_t*)"Keep still...\r\n", 15, 100);
    HAL_Delay(1000);
    for (int i = 0; i < N; i++)
    {
        HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, 0x3B, I2C_MEMADD_SIZE_8BIT, cb, 14, 100);
        sx += (int16_t)((cb[8] << 8) | cb[9]);
        sy += (int16_t)((cb[10] << 8) | cb[11]);
        sz += (int16_t)((cb[12] << 8) | cb[13]);
        HAL_Delay(5);
    }
    gx_bias = sx / N;
    gy_bias = sy / N;
    gz_bias = sz / N;

    l = snprintf(m, sizeof(m), "Gyro bias raw: %ld %ld %ld\r\n",
                 (long)gx_bias, (long)gy_bias, (long)gz_bias);
    HAL_UART_Transmit(&huart2, (uint8_t*)m, l, 100);
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  char msg[64];
  int len;
  uint8_t who = 0;
  HAL_StatusTypeDef st;

  len = snprintf(msg, sizeof(msg), "\r\nI2C scan start\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);

  for (uint8_t addr = 1; addr < 128; addr++)
  {
      if (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 2, 10) == HAL_OK)
      {
          len = snprintf(msg, sizeof(msg), "Found device at 0x%02X\r\n", addr);
          HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
      }
  }

  len = snprintf(msg, sizeof(msg), "Scan done\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);

  st = HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, 0x75, I2C_MEMADD_SIZE_8BIT, &who, 1, 100);
  len = snprintf(msg, sizeof(msg), "WHO_AM_I status=%d value=0x%02X\r\n", st, who);
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);

  /* Wake + configure */
  mpu_write(0x6B, 0x01);   /* PWR_MGMT_1: wake, clock = gyro X PLL (better than internal 8 MHz) */
  HAL_Delay(50);
  mpu_write(0x1A, 0x03);   /* CONFIG: DLPF ~42 Hz, gyro output rate = 1 kHz */
  mpu_write(0x19, 4);      /* SMPLRT_DIV: 1 kHz / (1+4) = 200 Hz */
  mpu_write(0x1B, 0x00);   /* GYRO_CONFIG: +/-250 dps  -> 131 LSB/dps */
  mpu_write(0x1C, 0x00);   /* ACCEL_CONFIG: +/-2 g     -> 16384 LSB/g */

  calibrate_gyro();

  dwt_init();
  seed_filter();
  dt_seconds();            /* prime the timer */

  uint32_t next_tick = HAL_GetTick();
  uint8_t  print_cnt = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  next_tick += LOOP_MS;

	  uint8_t rx;
	  if (HAL_UART_Receive(&huart2, &rx, 1, 0) == HAL_OK && rx == 'c')
	  {
	      calibrate_gyro();
	      seed_filter();
	      dt_seconds();
	      next_tick = HAL_GetTick() + LOOP_MS;
	  }

	  uint8_t buf[14];
	  char line[100];

	  if (HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, 0x3B, I2C_MEMADD_SIZE_8BIT, buf, 14, 100) == HAL_OK)
	  {
		  float dt = dt_seconds();

		  int16_t ax = (int16_t)((buf[0] << 8) | buf[1]);
		  int16_t ay = (int16_t)((buf[2] << 8) | buf[3]);
		  int16_t az = (int16_t)((buf[4] << 8) | buf[5]);
		  int16_t gx = (int16_t)((buf[8] << 8) | buf[9]);
		  int16_t gy = (int16_t)((buf[10] << 8) | buf[11]);

		  /* Gyro rates in deg/s (bias removed) */
		  float gx_dps = (gx - gx_bias) / GYRO_LSB_PER_DPS;
		  float gy_dps = (gy - gy_bias) / GYRO_LSB_PER_DPS;

		  /* Accel-only angles (what you had before) */
		  float roll_acc  = atan2f((float)ay, (float)az) * RAD2DEG;
		  float pitch_acc = atan2f(-(float)ax, sqrtf((float)ay*ay + (float)az*az)) * RAD2DEG;

		  /* Complementary filter with adaptive alpha:
		     if total accel is not ~1 g, the accel angle is unreliable, so trust the gyro */
		  float amag = sqrtf((float)ax*ax + (float)ay*ay + (float)az*az) / 16384.0f;
		  float a = (fabsf(amag - 1.0f) > ACC_TOL_G) ? ALPHA_HI : ALPHA;
		  roll_cf  = a * (roll_cf  + gx_dps * dt) + (1.0f - a) * roll_acc;
		  pitch_cf = a * (pitch_cf + gy_dps * dt) + (1.0f - a) * pitch_acc;

		  if (++print_cnt >= PRINT_EVERY)
		  {
		      print_cnt = 0;
		      /* CSV in centi-degrees: roll_acc, roll_cf, pitch_acc, pitch_cf */
		      int n = snprintf(line, sizeof(line), "%ld,%ld,%ld,%ld\r\n",
		                       (long)(roll_acc * 100),  (long)(roll_cf * 100),
		                       (long)(pitch_acc * 100), (long)(pitch_cf * 100));
		      HAL_UART_Transmit(&huart2, (uint8_t*)line, n, 20);
		  }
	  }
	  else
	  {
	      HAL_UART_Transmit(&huart2, (uint8_t*)"read fail\r\n", 11, 100);
	  }

	  /* Hold a fixed 200 Hz loop */
	  while ((int32_t)(HAL_GetTick() - next_tick) < 0) {}
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
