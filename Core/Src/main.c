/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
  Off,
  On,
  Blink
} STATE;

typedef struct LEDState
{
  STATE state;
  float frequency;
  uint32_t brightness;
} LEDState;

/* Private define ------------------------------------------------------------*/
#define DEAFULT_FREQUENCY 2.0
#define DEAFULT_BRIGHTNESS 100

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

PCD_HandleTypeDef hpcd_USB_FS;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for uartTask */
osThreadId_t uartTaskHandle;
const osThreadAttr_t uartTask_attributes = {
    .name = "uartTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityBelowNormal,
};
/* Definitions for BinarySemaphoreUART */
osSemaphoreId_t BinarySemaphoreUARTHandle;
const osSemaphoreAttr_t BinarySemaphoreUART_attributes = {
    .name = "BinarySemaphoreUART"};

/* Private values -----------------------------------------------------------*/
int8_t RxByte;
uint8_t RxBuff[127] = {0};
uint8_t RxIndex = 0;
uint8_t aRxBuffer[128] = {0};
uint16_t oldPos = 0;
uint16_t newPos = 0;

volatile LEDState ledState;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
void StartDefaultTask(void *argument);
void StartUARTTask(void *argument);

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

/* Private user code ---------------------------------------------------------*/
void setDeafultValues(void)
{
  ledState.brightness = DEAFULT_BRIGHTNESS;
  ledState.frequency = DEAFULT_FREQUENCY;
  ledState.state = Blink;
  printf("RESET\r\n");
}

uint32_t hz_to_msec(float hz)
{
  uint32_t msec = (uint32_t)(1000.0f / hz);
  if ((msec > 0) && (msec <= 2000))
    return (uint32_t)(msec);
  return 500; // Default value
}

uint32_t brightness_to_TIM_CCR1(uint32_t brightness)
{
  uint32_t ticks = (uint32_t)((65535 * brightness) / 100);
  if (ticks > 65535)
    return 65535;
  return ticks;
}

void retStatus(void)
{
  printf("STATUS;state=%d;brightness=%ld;frequency=%.4f\r\n", ledState.state, ledState.brightness, ledState.frequency);
}

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USB_PCD_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* Initialize PWM */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  /* Initialize UART DMA */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, RxBuff, 127);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

  /* Init scheduler */
  osKernelInitialize();

  /* Create the semaphores(s) */
  /* creation of BinarySemaphoreUART */
  BinarySemaphoreUARTHandle = osSemaphoreNew(1, 1, &BinarySemaphoreUART_attributes);

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of uartTask */
  uartTaskHandle = osThreadNew(StartUARTTask, NULL, &uartTask_attributes);

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB | RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim1);
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief USB Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_PCD_Init(void)
{
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin | LD5_Pin | LD7_Pin | LD9_Pin | LD10_Pin | LD8_Pin | LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin | MEMS_INT3_Pin | MEMS_INT4_Pin | MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD5_Pin LD7_Pin LD9_Pin
                           LD10_Pin LD8_Pin LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin | LD5_Pin | LD7_Pin | LD9_Pin | LD10_Pin | LD8_Pin | LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

/* Overwrite printf*/
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}
/* DMA UART callback*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{

  if (huart->Instance == USART1)
  {
    oldPos = newPos;

    if (oldPos + Size > 128)
    {
      uint16_t datatocopy = 128 - oldPos;
      memcpy((uint8_t *)aRxBuffer + oldPos, RxBuff, datatocopy);

      oldPos = 0;
      memcpy((uint8_t *)aRxBuffer, (uint8_t *)RxBuff + datatocopy, (Size - datatocopy));
      newPos = (Size - datatocopy);
    }

    else
    {
      memcpy((uint8_t *)aRxBuffer + oldPos, RxBuff, Size);
      newPos = Size + oldPos;
    }

    osSemaphoreRelease(BinarySemaphoreUARTHandle);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, RxBuff, 127);
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
  }
}
/* Button reset callback*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == Button_Pin)
  {
    printf("BUTTON ");
    setDeafultValues();
  }
}

/**
 * @brief  Thread to control LED3.
 * @param  argument: Not used
 * @retval None
 */
void StartDefaultTask(void *argument)
{

  setDeafultValues();
  uint32_t frequency = hz_to_msec(ledState.frequency);
  uint32_t brightness = brightness_to_TIM_CCR1(ledState.brightness);
  for (;;)
  {

    if (ledState.state == Off)
    {
      TIM1->CCR1 = 0;
      osDelay(frequency);
    }
    else if (ledState.state == On)
    {
      TIM1->CCR1 = brightness;
      osDelay(frequency);
    }
    else if (ledState.state == Blink)
    {
      TIM1->CCR1 = brightness;
      osDelay(frequency);
      TIM1->CCR1 = 0;
      osDelay(frequency);
    }
    else
    {
      printf("Unexpected error");
    }
    frequency = hz_to_msec(ledState.frequency);
    brightness = brightness_to_TIM_CCR1(ledState.brightness);
  }
}

/**
 * @brief Thread to process for user input.
 * @param argument: Not used
 * @retval None
 */
void StartUARTTask(void *argument)
{

  char *isEndLine = NULL;
  for (;;)
  {
    osSemaphoreAcquire(BinarySemaphoreUARTHandle, osWaitForever);
    isEndLine = strchr((char *)aRxBuffer, '\r');

    if ((aRxBuffer[0] != 0) && (aRxBuffer != 0) && (isEndLine != 0))
    {
      if (strncmp((char *)aRxBuffer, (char *)"reset", 5) == 0)
      {
        setDeafultValues();
      }
      else if (strncmp((char *)aRxBuffer, (char *)"status", 6) == 0)
      {
        retStatus();
      }
      else if (strncmp((char *)aRxBuffer, (char *)"state=", 6) == 0)
      {
        char *value = strchr((char *)aRxBuffer, '=');
        if (value != 0)
        {
          if ((strncmp((char *)value, (char *)"=on", 3) == 0))
            ledState.state = On;
          else if ((strncmp((char *)value, (char *)"=off", 4) == 0))
            ledState.state = Off;
          else if ((strncmp((char *)value, (char *)"=blink", 6) == 0))
            ledState.state = Blink;
          retStatus();
        }
      }
      else if (strncmp((char *)aRxBuffer, (char *)"brightness=", 11) == 0)
      {
        char *value = strchr((char *)aRxBuffer, '=') + 1;
        if (value != 0)
        {
          uint32_t val = 0;
          val = atoi((char *)value);
          if (val <= 100)
            ledState.brightness = val;
          retStatus();
        }
      }
      else if (strncmp((char *)aRxBuffer, (char *)"frequency=", 10) == 0)
      {
        char *value = strchr((char *)aRxBuffer, '=') + 1;

        if (value != 0)
        {
          float val = 0;
          val = atof((char *)value);
          if ((val <= 10.0) && (val >= 0.5))
            ledState.frequency = val;
          retStatus();
        }
      }
      /*reset UART buff*/
      RxIndex = 0;
      memset(aRxBuffer, 0, strlen((char *)aRxBuffer));
      memset(RxBuff, 0, strlen((char *)RxBuff));
      isEndLine = NULL;
      oldPos = 0;
      newPos = 0;
    }
    osDelay(1);
  }
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM2 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    HAL_IncTick();
  }
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
}
#endif /* USE_FULL_ASSERT */
