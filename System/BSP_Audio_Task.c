#include "BSP_Audio_Task.h"
#include "BSP_LED.h"
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_audio.h"
#include "MemoryLogger.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "MemoryLogger.h"
#include "SerialLogger.h"

enum {
  BUFFER_STATUS_LOWER_HALF_FULL,
  BUFFER_STATUS_UPPER_HALF_FULL
};
typedef uint32_t BufferStatusMessage_t ;

static QueueHandle_t xQueue_BufferStatus;
static BufferStatusMessage_t bufferStatusMessage;


// #define AUDIO_BUFFER_SIZE       2048
#define AUDIO_DEFAULT_VOLUME    70
#define CAMERA_RES_MAX_X          640
#define CAMERA_RES_MAX_Y          480
#define LCD_FRAME_BUFFER          SDRAM_DEVICE_ADDR
#define CAMERA_FRAME_BUFFER       ((uint32_t)(LCD_FRAME_BUFFER + (RK043FN48H_WIDTH * RK043FN48H_HEIGHT * ARBG8888_BYTE_PER_PIXEL)))

#define RGB565_BYTE_PER_PIXEL     2
#define ARBG8888_BYTE_PER_PIXEL   4

#define SDRAM_WRITE_READ_ADDR        ((uint32_t)(CAMERA_FRAME_BUFFER + (CAMERA_RES_MAX_X * CAMERA_RES_MAX_Y * RGB565_BYTE_PER_PIXEL)))

#define SDRAM_WRITE_READ_ADDR_OFFSET ((uint32_t)0x0800)
#define SRAM_WRITE_READ_ADDR_OFFSET  SDRAM_WRITE_READ_ADDR_OFFSET

#define AUDIO_REC_START_ADDR         SDRAM_WRITE_READ_ADDR
#define AUDIO_BLOCK_SIZE   2*((uint32_t)512)
#define AUDIO_BUFFER_IN    AUDIO_REC_START_ADDR     /* In SDRAM */
#define AUDIO_BUFFER_OUT   (AUDIO_REC_START_ADDR + (AUDIO_BLOCK_SIZE * 2)) /* In SDRAM */


static uint16_t  internal_buffer[AUDIO_BLOCK_SIZE] = {0};

uint32_t  audio_rec_buffer_state;

typedef enum
{
  BUFFER_OFFSET_NONE = 0,
  BUFFER_OFFSET_HALF = 1,
  BUFFER_OFFSET_FULL = 2,
}BUFFER_StateTypeDef;


void LoopBack_Task(void * argument)
{
  uint8_t  text[30];

  /* Initialize Audio Recorder */
  int status = BSP_AUDIO_IN_OUT_Init(INPUT_DEVICE_INPUT_LINE_1, OUTPUT_DEVICE_HEADPHONE, DEFAULT_AUDIO_IN_FREQ, DEFAULT_AUDIO_IN_BIT_RESOLUTION, DEFAULT_AUDIO_IN_CHANNEL_NBR);

  /* Initialize SDRAM buffers */
  uint32_t buff_in_address = AUDIO_BUFFER_IN;
  uint32_t buff_out_address = AUDIO_BUFFER_OUT;
  memset((uint16_t*)AUDIO_BUFFER_IN, 0, AUDIO_BLOCK_SIZE*2);
  memset((uint16_t*)AUDIO_BUFFER_OUT, 0, AUDIO_BLOCK_SIZE*2);
  audio_rec_buffer_state = BUFFER_OFFSET_NONE;

  BSP_AUDIO_IN_SetVolume(70);
  BSP_AUDIO_OUT_SetVolume(70);

  /* Start Recording */
  // BSP_AUDIO_IN_Record((uint16_t*)AUDIO_BUFFER_IN, AUDIO_BLOCK_SIZE);
  BSP_AUDIO_IN_Record((uint16_t*)AUDIO_BUFFER_OUT, AUDIO_BLOCK_SIZE);

  /* Start Playback */
  BSP_AUDIO_OUT_SetAudioFrameSlot(CODEC_AUDIOFRAME_SLOT_02);
  BSP_AUDIO_OUT_Play((uint16_t*)AUDIO_BUFFER_OUT, AUDIO_BLOCK_SIZE * 2);

  uint32_t audio_buffer_out = AUDIO_BUFFER_OUT;
  uint32_t audio_buffer_in = AUDIO_BUFFER_IN;

  while (1)
  {
    BSP_LED_Blink();
    // TODO(andrea): figure out why memcpy does not work!
#if 0
    /* Wait end of half block recording */
    while(audio_rec_buffer_state != BUFFER_OFFSET_HALF)
    {
    // HAL_Delay(10);
    }

    audio_rec_buffer_state = BUFFER_OFFSET_NONE;
    /* Copy recorded 1st half block */
    // memcpy((uint16_t *)(AUDIO_BUFFER_OUT),
    //        (uint16_t *)(AUDIO_BUFFER_IN),
    //        AUDIO_BLOCK_SIZE);

    /* Wait end of one block recording */
    while(audio_rec_buffer_state != BUFFER_OFFSET_FULL)
    {
    // HAL_Delay(10);
    }
    audio_rec_buffer_state = BUFFER_OFFSET_NONE;
    /* Copy recorded 2nd half block */
    // memcpy((uint16_t *)(AUDIO_BUFFER_OUT + (AUDIO_BLOCK_SIZE)),
    //        (uint16_t *)(AUDIO_BUFFER_IN + (AUDIO_BLOCK_SIZE)),
    //        AUDIO_BLOCK_SIZE);

#endif
  }
}

void BSP_AUDIO_IN_TransferComplete_CallBack(void)
{
  audio_rec_buffer_state = BUFFER_OFFSET_FULL;
  return;
}

/**
  * @brief  Manages the DMA Half Transfer complete interrupt.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_IN_HalfTransfer_CallBack(void)
{
  audio_rec_buffer_state = BUFFER_OFFSET_HALF;
  return;
}
