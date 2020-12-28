#include "MemoryLogger.h"
#include "BSP_LED.h"
#include "BSP_Audio_Task.h"
#include "BSP_LCD_Task.h"
#include "MIDI_Input_Task.h"
#include "SerialLogger_Task.h"
#include "Monitor_Task.h"
#include "stm32f7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32746g_discovery_audio.h"
#include "stm32746g_discovery_lcd.h"
#define DEBUG_DEFAULT_INTERRUPT_HANDLERS

#define ASSERT(__condition__)                do { if(__condition__) \
                                                   {  assert_failed(__FILE__, __LINE__); \
                                                      while(1);  \
                                                    } \
                                              }while(0)

void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

static void SystemClock_Config(void);
static void MPU_Config(void);

// Declare these somewhere else?
void vApplicationMallocFailedHook( void );
void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName );
void StartIdleMonitor (void);
void EndIdleMonitor (void);

//FreeRTOS heap
uint8_t  ucHeap[configTOTAL_HEAP_SIZE];

char log_buffer[BUFFER_SIZE];
char * log_ptr = log_buffer;
uint32_t  audio_rec_buffer_state;

int main(void)
{
  MPU_Config();
  SCB_InvalidateICache();

  SCB->CCR |= (1 <<18); /* Enable branch prediction */
  __DSB();

  SCB_InvalidateICache();
  SCB_EnableICache();

  SCB_InvalidateDCache();
  SCB_EnableDCache();


  /* STM32F7xx HAL library initialization:
  - Configure the Flash ART accelerator on ITCM interface
  - Configure the Systick to generate an interrupt each 1 msec
  - Set NVIC Group Priority to 4
  - Global MSP (MCU Support Package) initialization
  */

  // DO NOT ADD CODE WITH DELAYS BETWEEN HAL_Init() AND STARTING THE SCHEDULER!!!!!
  // YOU HAVE 1 MILLISECOND TO START THE SCHEDULER OR THE PROGRAM WILL CRASH

  HAL_Init();

  // if (BSP_SDRAM_Init() != 0)
  //     while(1)

  // if (BSP_Fast_UART_Init() != 0)
  //     while(1)

  // if (BSP_UART_Init() != 0)
  //     while(1)

  SystemClock_Config();/* Configure the system clock @ 200 Mhz */

  BSP_LED_Init();

  int myStackSize = 1024;

#if 1
  // LOWER NUMBER PRIORITIES ARE LOWER PRIORITIES

  // This should be the lowest priorty task
  // Create this task first because other tasks send messages
  // Remember not to send messages in constructors because the message queue won't be created yet

  xTaskCreate(SerialLogger_Task,
              "Serial Logger Task",
              myStackSize,
              NULL, //task params
              2, //priority
              NULL ); //task handle

  xTaskCreate(Monitor_Task,
              "Monitor Task",
              myStackSize,
              NULL, //task params
              1, //priority
              NULL ); //task handle

  xTaskCreate(LoopBack_Task,
              "My Audio Task",
              myStackSize,
              NULL, //task params
              8,  //priority
              NULL ); //task handle

  // xTaskCreate(My_LCD_Task,
  //             "My LCD Task",
  //             myStackSize,
  //             NULL, //task params
  //             1,  //priority
  //             NULL ); //task handle

  // xTaskCreate(MIDI_Input_Task,
  //             "MIDI Input Task",
  //             myStackSize,
  //             NULL, //task params
  //             3, //priority
  //             NULL ); //task handle

  // setup_led();
  // blink();
  vTaskStartScheduler();

  /* We should never get here as control is now taken by the scheduler */
  for( ;; );
}
#endif
#if 0 

#include "stm32746g_discovery_lcd.h"
#define SDRAM_DEVICE_ADDR  ((uint32_t)0xC0000000)
#define LCD_FRAME_BUFFER          SDRAM_DEVICE_ADDR
#define LCD_OK                 ((uint8_t)0x00)
#define LCD_ERROR              ((uint8_t)0x01)
#define LCD_TIMEOUT            ((uint8_t)0x02)
#define LTDC_ACTIVE_LAYER	     ((uint32_t)1) /* Layer 1 */


// static void Display_Description(void)
// {
//   uint8_t desc[50];

//   /* Set LCD Foreground Layer  */
//   BSP_LCD_SelectLayer(LTDC_ACTIVE_LAYER);

//   BSP_LCD_SetFont(&LCD_DEFAULT_FONT);

//   /* Clear the LCD */
//   BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
//   BSP_LCD_Clear(LCD_COLOR_WHITE);

//   /* Set the LCD Text Color */
//   BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);

//   /* Display LCD messages */
//   BSP_LCD_DisplayStringAt(0, 10, (uint8_t *)"STM32F746G BSP", CENTER_MODE);
//   BSP_LCD_DisplayStringAt(0, 35, (uint8_t *)"Drivers examples", CENTER_MODE);

//   /* Draw Bitmap */
//   // BSP_LCD_DrawBitmap((BSP_LCD_GetXSize() - 80) / 2, 65, (uint8_t *)stlogo);

//   BSP_LCD_SetFont(&Font12);
//   BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 20, (uint8_t *)"Copyright (c) STMicroelectronics 2015", CENTER_MODE);

//   BSP_LCD_SetFont(&Font16);
//   BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
//   BSP_LCD_FillRect(0, BSP_LCD_GetYSize() / 2 + 15, BSP_LCD_GetXSize(), 60);
//   BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
//   BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
//   BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 + 30, (uint8_t *)"Press User Button to start :", CENTER_MODE);
//   sprintf((char *)desc, "%s AUDIO RECORD");
//   BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 + 45, (uint8_t *)desc, CENTER_MODE);
// }

int main()
{
  

  CPU_CACHE_Enable();
  uint8_t  lcd_status = LCD_OK;

  /* Enable the CPU Cache */
  
  /* STM32F7xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  setup_led();
  blink();
  HAL_Init();
  /* Configure the system clock to 200 Mhz */
  SystemClock_Config();

  // BSP_LED_Init(LED1);

  /* Configure the User Button in GPIO Mode */
  // BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO);

  /*##-1- Initialize the LCD #################################################*/
  /* Initialize the LCD */
  lcd_status = BSP_LCD_Init();
  ASSERT(lcd_status != LCD_OK);

  /* Initialize the LCD Layers */
  // BSP_LCD_LayerDefaultInit(LTDC_ACTIVE_LAYER, LCD_FRAME_BUFFER);


  // Display_Description();


  /* Wait For User inputs */
  while (1)
  {
    HAL_Delay(100);
    // Display_Description();
  }
}
#endif

void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
  (void)xTask;
  (void)pcTaskName;
  while(1);
}


void vApplicationMallocFailedHook( void )
{
  while(1);
}

////FREERTOS IDLE AND TICK HOOKS ARE IN MONITOR_TASK

void StartIdleMonitor (void)
{
}
void EndIdleMonitor (void)
{
}






/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 200000000
  *            HCLK(Hz)                       = 200000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 400
  *            PLL_P                          = 2
  *            PLLSAI_N                       = 384
  *            PLLSAI_P                       = 8
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 6
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  HAL_StatusTypeDef ret = HAL_OK;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }

  /* activate the OverDrive */
  ret = HAL_PWREx_ActivateOverDrive();
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
}


/**
  * @brief  Configure the MPU attributes.
  * @param  None
  * @retval None
  */
static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;

  /* Disable the MPU */
  HAL_MPU_Disable();

  /* Configure the MPU as Normal Non Cacheable for the SRAM1 and SRAM2 */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x20020000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_512KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Configure the MPU attributes as Non Cacheable and Non Bufferable for SDRAM */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0xC0000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256MB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Configure the MPU attributes as WT for SDRAM */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0xC0000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_8MB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER3;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Configure the MPU attributes as strongly ordred for QSPI (unused zone) */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x90000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256MB;
  MPU_InitStruct.AccessPermission = MPU_REGION_PRIV_RO;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER4;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Configure the MPU attributes as WT for QSPI (used 16Mbytes) */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x90000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_16MB;
  MPU_InitStruct.AccessPermission = MPU_REGION_PRIV_RO;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER5;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enable the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}

#ifdef USE_FULL_ASSERT
/**
* @brief  assert_failed
*         Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  File: pointer to the source file name
* @param  Line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line
  number,ex: printf("Wrong parameters value: file %s on line %d\r\n",
  file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif


/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
