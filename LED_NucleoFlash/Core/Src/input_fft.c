#include "stm32l4xx_hal.h"
#include "arm_math.h"

#define ADC_BUFFER_SIZE 1024  // Size of the ADC buffer
#define FFT_SIZE ADC_BUFFER_SIZE / 2  // Size of the FFT buffer

/* ADC handle and buffer */
ADC_HandleTypeDef hadc1;
uint16_t adc_buffer[ADC_BUFFER_SIZE];

/* FFT buffers */
float32_t fft_input[ADC_BUFFER_SIZE];
float32_t fft_output[ADC_BUFFER_SIZE];

/* UART handle */
UART_HandleTypeDef huart2;

/* Function prototypes */
void SystemClock_Config(void);
void ADC1_Init(void);
void GPIO_Init(void);
void UART2_Init(void);
void UART_SendFloatArray(float32_t *array, uint16_t size);

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  /* Initialize ADC, GPIO, and UART */
  GPIO_Init();
  ADC1_Init();
  UART2_Init();

  /* Initialize the FFT configuration */
  arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE);

  while (1)
  {
    /* Start ADC conversion */
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, ADC_BUFFER_SIZE);

    /* Wait for ADC conversion to complete */
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);

    /* Convert ADC data to floating-point voltage values */
    float32_t voltage[ADC_BUFFER_SIZE];
    for (int i = 0; i < ADC_BUFFER_SIZE; i++)
    {
      voltage[i] = (float32_t)adc_buffer[i] * 3.3f / 4095.0f;  // Convert ADC value to voltage (assuming 12-bit ADC and 3.3V reference)
    }

    /* Perform the FFT */
    arm_rfft_fast_f32(&fft_instance, voltage, fft_output, 0);

    /* Send the FFT data over UART */
    UART_SendFloatArray(fft_output, FFT_SIZE);

    /* Stop ADC conversion */
    HAL_ADC_Stop_DMA(&hadc1);
  }
}

void ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_2CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_2CYCLES_5;
  hadc1.Init.OversamplingMode = DISABLE;

  sConfig.Channel = ADC_CHANNEL_6;  // Configure for PA6 pin
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;

  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

void GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PB6 (UART TX) and PB7 (UART RX) */
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void UART2_Init(void)
{
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
}

void UART_SendFloatArray(float32_t *array, uint16_t size)
{
  char buffer[32];  // Assuming a maximum float value length of 32 characters

  for (int i = 0; i < size; i++)
  {
    sprintf(buffer, array[i]);
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
  }
}

void Error_Handler(void)
{
  while (1)
  {
    /* Error occurred */
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
