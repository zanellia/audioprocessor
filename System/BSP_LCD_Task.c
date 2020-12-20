#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery.h"
#include "BSP_LCD_Task.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "MemoryLogger.h"

#include "SerialLogger.h"

#define LCD_FRAME_BUFFER          SDRAM_DEVICE_ADDR
#define LCD_OK                 ((uint8_t)0x00)
#define LCD_ERROR              ((uint8_t)0x01)
#define LCD_TIMEOUT            ((uint8_t)0x02)


#define ASSERT(__condition__)                do { if(__condition__) \
                                                   {  assert_failed(__FILE__, __LINE__); \
                                                      while(1);  \
                                                    } \
                                              }while(0)

extern char log_buffer[BUFFER_SIZE];
typedef uint32_t BufferStatusMessage_t ;

static QueueHandle_t xQueue_BufferStatus;
static BufferStatusMessage_t bufferStatusMessage;

void Display_Description(void)
{
  // char buffer_alias[BUFFER_SIZE];
  // for(int i = 0; i < BUFFER_SIZE; i++)
  //     buffer_alias[i] = log_buffer[i];

  /* Set LCD Foreground Layer  */
  BSP_LCD_SelectLayer(LTDC_ACTIVE_LAYER);

  BSP_LCD_SetFont(&LCD_DEFAULT_FONT);

  // /* Clear the LCD */
  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
  BSP_LCD_Clear(LCD_COLOR_WHITE);

  // /* Set the LCD Text Color */
  BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);

  // /* Display LCD messages */
  // BSP_LCD_DisplayStringAt(0, 10, (uint8_t *)"STM32F746G BSP", CENTER_MODE);
  // BSP_LCD_DisplayStringAt(0, 35, (uint8_t *)"Drivers examples", CENTER_MODE);

  // /* Draw Bitmap */
  // // BSP_LCD_DrawBitmap((BSP_LCD_GetXSize() - 80) / 2, 65, (uint8_t *)stlogo);

  // BSP_LCD_SetFont(&Font12);
  // BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 20, (uint8_t *)"Copyright (c) STMicroelectronics 2015", CENTER_MODE);

  // BSP_LCD_SetFont(&Font16);
  // BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
  // BSP_LCD_FillRect(0, BSP_LCD_GetYSize() / 2 + 15, BSP_LCD_GetXSize(), 60);
  // BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  // BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
  // BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 + 30, (uint8_t *)"Press User Button to start :", CENTER_MODE);
  // sprintf((char *)desc, "%s");
  
  BSP_LCD_SetFont(&Font8);
  // BSP_LCD_DisplayStringAtWithSize(0, 10, (uint8_t *)buffer_alias, CENTER_MODE, BUFFER_SIZE);
}

void My_LCD_Task(void * argument)
{

  uint8_t  lcd_status = LCD_OK;
  (void)argument;
  RUN_AND_LOG( BSP_LCD_Init(); );

  RUN_AND_LOG( ASSERT(lcd_status != LCD_OK); );

  /* Initialize the LCD Layers */
  RUN_AND_LOG( BSP_LCD_LayerDefaultInit(LTDC_ACTIVE_LAYER, LCD_FRAME_BUFFER); );

  xQueue_BufferStatus = xQueueCreate(32, sizeof(BufferStatusMessage_t));

  while(1)
  {
      Display_Description();
      HAL_Delay(1.0);
  }
}


