/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "libjpeg.h"
#include "app_touchgfx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32h750b_discovery_qspi.h"
#include "stm32h750b_discovery_sdram.h"
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

CRC_HandleTypeDef hcrc;

DMA2D_HandleTypeDef hdma2d;

JPEG_HandleTypeDef hjpeg;
MDMA_HandleTypeDef hmdma_jpeg_infifo_th;
MDMA_HandleTypeDef hmdma_jpeg_outfifo_th;

LTDC_HandleTypeDef hltdc;

QSPI_HandleTypeDef hqspi;

RNG_HandleTypeDef hrng;

SDRAM_HandleTypeDef hsdram2;

/* Definitions for gamePhysicsTask */
osThreadId_t gamePhysicsTaskHandle;
const osThreadAttr_t gamePhysicsTask_attributes = {
  .name = "gamePhysicsTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for GUITask */
osThreadId_t GUITaskHandle;
const osThreadAttr_t GUITask_attributes = {
  .name = "GUITask",
  .stack_size = 8192 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for screenInputTask */
osThreadId_t screenInputTaskHandle;
const osThreadAttr_t screenInputTask_attributes = {
  .name = "screenInputTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for blockSelectionT */
osThreadId_t blockSelectionTHandle;
const osThreadAttr_t blockSelectionT_attributes = {
  .name = "blockSelectionT",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal7,
};
/* Definitions for gameSettingsTas */
osThreadId_t gameSettingsTasHandle;
const osThreadAttr_t gameSettingsTas_attributes = {
  .name = "gameSettingsTas",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for pixelInputQueue */
osMessageQueueId_t pixelInputQueueHandle;
const osMessageQueueAttr_t pixelInputQueue_attributes = {
  .name = "pixelInputQueue"
};
/* Definitions for frameUpdateQueue */
osMessageQueueId_t frameUpdateQueueHandle;
const osMessageQueueAttr_t frameUpdateQueue_attributes = {
  .name = "frameUpdateQueue"
};
/* Definitions for blockSelectQueue */
osMessageQueueId_t blockSelectQueueHandle;
const osMessageQueueAttr_t blockSelectQueue_attributes = {
  .name = "blockSelectQueue"
};
/* Definitions for gameSettingsQueue */
osMessageQueueId_t gameSettingsQueueHandle;
const osMessageQueueAttr_t gameSettingsQueue_attributes = {
  .name = "gameSettingsQueue"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_MDMA_Init(void);
static void MX_LTDC_Init(void);
static void MX_DMA2D_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_FMC_Init(void);
static void MX_JPEG_Init(void);
static void MX_CRC_Init(void);
static void MX_RNG_Init(void);
void gamePhysicsTaskFunc(void *argument);
extern void TouchGFX_Task(void *argument);
void screenInputTaskFunc(void *argument);
void blockSelectionTaskFunc(void *argument);
void gameSettingsTaskFunc(void *argument);

/* USER CODE BEGIN PFP */

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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();
/* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* Explicit enabling interrupt to support debugging in CubeIDE when using external flash loader */
  __enable_irq();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_MDMA_Init();
  MX_LTDC_Init();
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_LIBJPEG_Init();
  MX_JPEG_Init();
  MX_CRC_Init();
  MX_RNG_Init();
  MX_TouchGFX_Init();
  /* Call PreOsInit function */
  MX_TouchGFX_PreOSInit();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of pixelInputQueue */
  pixelInputQueueHandle = osMessageQueueNew (16, sizeof(uint32_t), &pixelInputQueue_attributes);

  /* creation of frameUpdateQueue */
  frameUpdateQueueHandle = osMessageQueueNew (1, sizeof(uint8_t), &frameUpdateQueue_attributes);

  /* creation of blockSelectQueue */
  blockSelectQueueHandle = osMessageQueueNew (1, sizeof(uint16_t), &blockSelectQueue_attributes);

  /* creation of gameSettingsQueue */
  gameSettingsQueueHandle = osMessageQueueNew (1, sizeof(uint16_t), &gameSettingsQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of gamePhysicsTask */
  gamePhysicsTaskHandle = osThreadNew(gamePhysicsTaskFunc, NULL, &gamePhysicsTask_attributes);

  /* creation of GUITask */
  GUITaskHandle = osThreadNew(TouchGFX_Task, NULL, &GUITask_attributes);

  /* creation of screenInputTask */
  screenInputTaskHandle = osThreadNew(screenInputTaskFunc, NULL, &screenInputTask_attributes);

  /* creation of blockSelectionT */
  blockSelectionTHandle = osThreadNew(blockSelectionTaskFunc, NULL, &blockSelectionT_attributes);

  /* creation of gameSettingsTas */
  gameSettingsTasHandle = osThreadNew(gameSettingsTaskFunc, NULL, &gameSettingsTas_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_RGB565;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_RGB565;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  hdma2d.LayerCfg[1].AlphaInverted = DMA2D_REGULAR_ALPHA;
  hdma2d.LayerCfg[1].RedBlueSwap = DMA2D_RB_REGULAR;
  hdma2d.LayerCfg[1].ChromaSubSampling = DMA2D_NO_CSS;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/**
  * @brief JPEG Initialization Function
  * @param None
  * @retval None
  */
static void MX_JPEG_Init(void)
{

  /* USER CODE BEGIN JPEG_Init 0 */

  /* USER CODE END JPEG_Init 0 */

  /* USER CODE BEGIN JPEG_Init 1 */

  /* USER CODE END JPEG_Init 1 */
  hjpeg.Instance = JPEG;
  if (HAL_JPEG_Init(&hjpeg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN JPEG_Init 2 */

  /* USER CODE END JPEG_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 39;
  hltdc.Init.VerticalSync = 8;
  hltdc.Init.AccumulatedHBP = 42;
  hltdc.Init.AccumulatedVBP = 11;
  hltdc.Init.AccumulatedActiveW = 522;
  hltdc.Init.AccumulatedActiveH = 283;
  hltdc.Init.TotalWidth = 528;
  hltdc.Init.TotalHeigh = 285;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 480;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 272;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  pLayerCfg.Alpha = 255;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = 0;
  pLayerCfg.ImageWidth = 480;
  pLayerCfg.ImageHeight = 272;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */
  BSP_QSPI_Init_t qspi_initParams ;
  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 3;
  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = 26;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_4_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.DualFlash = QSPI_DUALFLASH_ENABLE;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */
  qspi_initParams.InterfaceMode = MT25TL01G_QPI_MODE;
  qspi_initParams.TransferRate  = MT25TL01G_DTR_TRANSFER ;
  qspi_initParams.DualFlashMode = MT25TL01G_DUALFLASH_ENABLE;
  BSP_QSPI_DeInit(0);
  if (BSP_QSPI_Init(0, &qspi_initParams) != BSP_ERROR_NONE)
  {
    Error_Handler( );
  }
  if(BSP_QSPI_EnableMemoryMappedMode(0) != BSP_ERROR_NONE)
  {
    Error_Handler( );
  }
  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  hrng.Init.ClockErrorDetection = RNG_CED_ENABLE;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * Enable MDMA controller clock
  */
static void MX_MDMA_Init(void)
{

  /* MDMA controller clock enable */
  __HAL_RCC_MDMA_CLK_ENABLE();
  /* Local variables */

  /* MDMA interrupt initialization */
  /* MDMA_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MDMA_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(MDMA_IRQn);

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM2 memory initialization sequence
  */
  hsdram2.Instance = FMC_SDRAM_DEVICE;
  /* hsdram2.Init */
  hsdram2.Init.SDBank = FMC_SDRAM_BANK2;
  hsdram2.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram2.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram2.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram2.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram2.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
  hsdram2.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram2.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
  hsdram2.Init.ReadBurst = FMC_SDRAM_RBURST_ENABLE;
  hsdram2.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 2;
  SdramTiming.ExitSelfRefreshDelay = 7;
  SdramTiming.SelfRefreshTime = 4;
  SdramTiming.RowCycleDelay = 7;
  SdramTiming.WriteRecoveryTime = 5;
  SdramTiming.RPDelay = 2;
  SdramTiming.RCDDelay = 2;

  if (HAL_SDRAM_Init(&hsdram2, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */
  BSP_SDRAM_DeInit(0);
  if(BSP_SDRAM_Init(0) != BSP_ERROR_NONE)
  {
    Error_Handler( );
  }
  /* USER CODE END FMC_Init 2 */
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
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, FRAME_RATE_Pin|RENDER_TIME_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_DE_GPIO_Port, LCD_DE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(VSYNC_FREQ_GPIO_Port, VSYNC_FREQ_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_Port, LCD_BL_CTRL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_RESET_Pin|MCU_ACTIVE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : FRAME_RATE_Pin RENDER_TIME_Pin */
  GPIO_InitStruct.Pin = FRAME_RATE_Pin|RENDER_TIME_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_DE_Pin */
  GPIO_InitStruct.Pin = LCD_DE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_DE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : VSYNC_FREQ_Pin */
  GPIO_InitStruct.Pin = VSYNC_FREQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(VSYNC_FREQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_BL_CTRL_Pin */
  GPIO_InitStruct.Pin = LCD_BL_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_BL_CTRL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_RESET_Pin */
  GPIO_InitStruct.Pin = LCD_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MCU_ACTIVE_Pin */
  GPIO_InitStruct.Pin = MCU_ACTIVE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(MCU_ACTIVE_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

volatile uint8_t* sdramcachestart = (volatile uint8_t*) 0xD0000000;
uint32_t sdramcachesize = (2 * 480 * 272) * 3;

canvas_TypeDef canvas = {0};
volatile gamestate state = GAME_RESET;
volatile uint8_t brushSize = 1;
volatile block_TypeDef brushBlock = BT_POWDER;

block_TypeDef gravSensitive[] = {BT_POWDER, BT_WATER, BT_SEED, BT_ACID};
#define gravSensBlocks 4

void setupPhysicsArrays(){
	canvas.x_speedmap = (char*)(canvas.canvasBitmap + canvas.width * canvas.height);
	canvas.x_speedmap = canvas.x_speedmap + (canvas.width * canvas.height);
	for (uint16_t i = 0; i < (canvas.width * canvas.height); i++){
		canvas.x_speedmap[i] = 0;
		canvas.y_speedmap[i] = 0;
	}
}

void buildWallBorder(){
	for (uint16_t i = 0; i < (canvas.width * canvas.height); i++){
		uint16_t x = i % canvas.width;
		uint16_t y = i / canvas.width;
		if (x < 8 || x > canvas.width - 8 || y < 8 || y > canvas.height - 8){
			canvas.canvasBitmap[i] = BT_WALL;
		}
	}
}


uint16_t mapDensity(block_TypeDef block){
	switch (block){
		case BT_VACCUM:
			return 0;
		case BT_AIR:
			return 1;
		case BT_WALL:
			return 1000;
		case BT_POWDER:
			return 100;
		case BT_WATER:
			return 95;
		case BT_SEED:
			return 99;
		case BT_ACID:
			return 95;
		case BT_TREE:
			return 150;
		case BT_FLOWER:
			return 150;
		case BT_TREE_GROWING:
			return 150;
		case BT_FIRE:
			return 1;
	}
	return 0;
}

uint16_t xy2i(uint16_t x, uint16_t y){
	return canvas.width * y + x;
}

void swapCells(uint16_t x, uint16_t y, uint16_t x2, uint16_t y2){
	uint16_t i = xy2i(x, y);
	uint16_t i2 = xy2i(x2, y2);
	char x2_speed = canvas.x_speedmap[i2];
	char y2_speed = canvas.y_speedmap[i2];

	canvas.x_speedmap[i2] = canvas.x_speedmap[i];
	canvas.y_speedmap[i2] = canvas.y_speedmap[i];
	canvas.x_speedmap[i] = x2_speed;
	canvas.y_speedmap[i] = y2_speed;

	block_TypeDef b2 = canvas.canvasBitmap[i2];
	canvas.canvasBitmap[i2] = canvas.canvasBitmap[i];
	canvas.canvasBitmap[i] = b2;
}

void applyGravity(){
	for (uint16_t i = 0; i < (canvas.width * canvas.height); i++){
		block_TypeDef pb = canvas.canvasBitmap[i];
		for (int ind = 0; ind < gravSensBlocks; ind++){
			if (pb == gravSensitive[ind]){
				canvas.y_speedmap[i] += 1;
				break;
			}
		}

	}
}

char blockCompare(block_TypeDef curblock, block_TypeDef sideblock){
	uint16_t cd = mapDensity(curblock);
	uint16_t sd = mapDensity(sideblock);
	if (cd > sd){
		return 1;
	}
	return 0;
}

void applyPhysics(){
	for (int i = (canvas.width * canvas.height) - 1; i >= 0; i--){
		uint16_t x = i % canvas.width;
		uint16_t y = i / canvas.width;
		char y_speed = canvas.y_speedmap[i];

		uint16_t cur_y = y;
 		if (cur_y + y_speed >= canvas.height){
			if (canvas.canvasBitmap[xy2i(x, 0)] == BT_VACCUM){
				uint16_t dist = canvas.height - cur_y;
				swapCells(x, cur_y, x, y_speed - dist);
			}
			continue;
		}
		char y_dir = y_speed < 0 ? -1 : (y_speed > 0 ? 1 : 0);
		while (cur_y != y + y_speed){
			uint16_t bottom_y = cur_y + y_dir;
			uint16_t cur_i = xy2i(x, cur_y);
			uint16_t bottom_i = xy2i(x, bottom_y);
			if (mapDensity(canvas.canvasBitmap[cur_i]) > mapDensity(canvas.canvasBitmap[bottom_i])){
				swapCells(x, cur_y, x, bottom_y);
				cur_y += y_dir;
			} else {
				canvas.y_speedmap[cur_i] = canvas.y_speedmap[bottom_i];
				break;
			}
		}
	}

	uint32_t rnd = 0;
	for (int i = 0; i < (canvas.width * canvas.height); i++){
		uint16_t x = i % canvas.width;
		uint16_t y = i / canvas.width;


		block_TypeDef curblock = canvas.canvasBitmap[xy2i(x, y)];
		if (curblock != BT_WATER && curblock != BT_ACID) continue;
		if (rnd == 0) HAL_RNG_GenerateRandomNumber(&hrng, &rnd);

		uint16_t ri = xy2i(x+1, y);
		uint16_t li = xy2i(x-1, y);

		if (blockCompare(canvas.canvasBitmap[i], canvas.canvasBitmap[li])){
			if (blockCompare(canvas.canvasBitmap[i], canvas.canvasBitmap[ri])){
				//both sides available
				if ((rnd & 1) == 0){
					swapCells(x, y, x-1, y);
					canvas.y_speedmap[li] = 1;
				} else {
					swapCells(x, y, x+1, y);
					canvas.y_speedmap[ri] = 1;
				}
			} else {
				//left available
				if ((rnd & 1) == 0){
					swapCells(x, y, x-1, y);
					canvas.y_speedmap[li] = 1;
				}
			}
		} else if (blockCompare(canvas.canvasBitmap[i], canvas.canvasBitmap[ri])){
			//right available
			if ((rnd & 1) == 0){
				swapCells(x, y, x+1, y);
				canvas.y_speedmap[ri] = 1;
			}
		}
		rnd = rnd >> 1;
	}
}

void handleBlockInterractions(){
	uint32_t rnd = 0;
	for (int i = 0; i < (canvas.width * canvas.height); i++){
		uint16_t x = i % canvas.width;
		uint16_t y = i / canvas.width;
		block_TypeDef block = canvas.canvasBitmap[i];
		if (rnd == 0) HAL_RNG_GenerateRandomNumber(&hrng, &rnd);


		switch(block){
			case BT_SEED:
				if (y < canvas.height-1 && canvas.canvasBitmap[xy2i(x, y+1)] == BT_POWDER){
					canvas.canvasBitmap[i] = BT_TREE_GROWING;
				}
				break;
			case BT_TREE_GROWING:
				if (rnd % 4 == 0 && (y > 0) && canvas.canvasBitmap[xy2i(x, y-1)] == BT_VACCUM){
					//grow up
					canvas.canvasBitmap[xy2i(x, y-1)] = BT_TREE_GROWING;
				} else if (rnd % 4 == 1 && (x < canvas.width-1) && canvas.canvasBitmap[xy2i(x+1, y)] == BT_VACCUM){
					//grow right
					canvas.canvasBitmap[xy2i(x+1, y)] = BT_TREE_GROWING;
				} else if (rnd % 4 == 2 && (x > 0) && canvas.canvasBitmap[xy2i(x-1, y)] == BT_VACCUM){
					//grow left
					canvas.canvasBitmap[xy2i(x-1, y)] = BT_TREE_GROWING;
				}
				rnd = rnd >> 2;
				if (rnd == 0) HAL_RNG_GenerateRandomNumber(&hrng, &rnd);
				if (rnd % 5 == 0){
					canvas.canvasBitmap[i] = BT_TREE;
				} else if (rnd % 6 == 0){
					canvas.canvasBitmap[i] = BT_FLOWER;
				} else {
					canvas.canvasBitmap[i] = BT_TREE_GROWING;
				}
				rnd = rnd >> 2;
				break;
			case BT_ACID:
				block_TypeDef bottom = y < canvas.height-1 ? canvas.canvasBitmap[xy2i(x, y+1)] : BT_WALL;
				block_TypeDef left = x > 0 ? canvas.canvasBitmap[xy2i(x-1, y)] : BT_WALL;
				block_TypeDef right = x < canvas.width-1 ? canvas.canvasBitmap[xy2i(x+1, y)] : BT_WALL;

				if (bottom != BT_WALL && bottom != BT_ACID && bottom != BT_VACCUM){
					swapCells(x, y, x, y+1);
					canvas.canvasBitmap[i] = BT_VACCUM;
					if (rnd % 2 == 0) canvas.canvasBitmap[xy2i(x, y+1)] = BT_VACCUM;
				} else if (right != BT_WALL && right != BT_ACID && right != BT_VACCUM){
					swapCells(x, y, x+1, y);
					canvas.canvasBitmap[i] = BT_VACCUM;
					if (rnd % 2 == 0) canvas.canvasBitmap[xy2i(x+1, y)] = BT_VACCUM;
				} else if (left != BT_WALL && left != BT_ACID && left != BT_VACCUM){
					swapCells(x, y, x-1, y);
					canvas.canvasBitmap[i] = BT_VACCUM;
					if (rnd % 2 == 0) canvas.canvasBitmap[xy2i(x-1, y)] = BT_VACCUM;
				}
				rnd = rnd >> 1;


				break;
			default:
				break;
		}
	}
}

uint16_t gravFrameCounter = 0;
void gamePhysicsTaskFunc(void *argument){
	uint8_t frameupdate = 1;
	for(;;){
		if (state == GAME_RESET || state == GAME_BUILDING){
			if (state == GAME_BUILDING){
				setupPhysicsArrays();
				buildWallBorder();
				osMessageQueuePut(frameUpdateQueueHandle, &frameupdate, 0U, 0U);
			}
			osDelay(100);
			continue;
		}
		if (state == GAME_PAUSED){
			osDelay(100);
			continue;
		}
		//game running
		if (gravFrameCounter++ % 16 == 0) applyGravity();
		applyPhysics();
		handleBlockInterractions();
		osMessageQueuePut(frameUpdateQueueHandle, &frameupdate, 0U, 0U);
		osDelay(1);
	}
}

void paintDot(uint16_t x, uint16_t y){
    for (uint8_t py = y; py < y + brushSize; py++){
        for (uint8_t px = x; px < x + brushSize; px++){
            if (px >= 0 && px < canvas.width && py >= 0 && py < canvas.height){
            	uint16_t i = xy2i(px, py);
            	if (canvas.canvasBitmap[i] == BT_VACCUM || brushBlock == BT_WALL || brushBlock == BT_VACCUM){
            		canvas.canvasBitmap[i] = brushBlock;
            		for (int ind = 0; ind < gravSensBlocks; ind++){
            			if (brushBlock == gravSensitive[ind]){
            				canvas.y_speedmap[i] = 1;
            				break;
            			}
            		}
            	}
            }
        }
    }
}

void screenInputTaskFunc(void *argument){
    uint32_t inputxy = 0;
    osStatus_t queueStatus;

    for(;;){
        if (state == GAME_RESET || state == GAME_BUILDING){
            osDelay(100);
            continue;
        }

        while ((queueStatus = osMessageQueueGet(pixelInputQueueHandle, &inputxy, 0U, 0U)) == osOK){
            uint16_t x = inputxy >> 16;
            uint16_t y = (uint16_t) inputxy;
            paintDot(x, y);
        }
        osDelay(5);
    }
}


void blockSelectionTaskFunc(void *argument)
{
	osStatus_t queueStatus;
	block_TypeDef block = BT_POWDER;
	for(;;)
		{
		if (state == GAME_RESET || state == GAME_BUILDING){
			osDelay(100);
			continue;
		}
		queueStatus = osMessageQueueGet(blockSelectQueueHandle, &block, 0U, 0U);
		if (queueStatus == osOK){
			brushBlock = block;
		}
		osDelay(30);
	}
}

void gameSettingsTaskFunc(void *argument)
{
	osStatus_t queueStatus;
	uint16_t settings = 0;
	for(;;){
		queueStatus = osMessageQueueGet(gameSettingsQueueHandle, &settings, 0U, 0U);
		if (queueStatus == osOK){
			state = (gamestate)(settings & 0b11);
			brushSize = (uint8_t)(settings >> 2);
		}
		osDelay(20);
	}
}




void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x24000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_512KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x90000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256MB;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  MPU_InitStruct.Size = MPU_REGION_SIZE_128MB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER3;
  MPU_InitStruct.BaseAddress = 0xD0000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256MB;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER4;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32MB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
