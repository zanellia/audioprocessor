#include "BSP_Audio_Task.h"
#include "BSP_LED.h"
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_audio.h"
#include "MemoryLogger.h"

#include "BSP_Audio_Buffer_Interface.h"
// #include "AudioProcessor.h"

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

static int16_t recordBuffer[MY_BUFFER_SIZE_SAMPLES/2];

#if 0
void My_Audio_Task(void * argument)
{


  (void)argument;

  /* Initialize Audio Recorder */
  if (BSP_AUDIO_IN_Init(DEFAULT_AUDIO_IN_FREQ, DEFAULT_AUDIO_IN_BIT_RESOLUTION, DEFAULT_AUDIO_IN_CHANNEL_NBR) == AUDIO_OK)
  {
    BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
    BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 95, (uint8_t *)"  AUDIO RECORD INIT OK  ", CENTER_MODE);

    BSP_AUDIO_IN_SetVolume(70);
    BSP_AUDIO_OUT_SetVolume(70);
  }
  else
  {
    BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
    BSP_LCD_SetTextColor(LCD_COLOR_RED);
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 95, (uint8_t *)"  AUDIO RECORD INIT FAIL", CENTER_MODE);
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 80, (uint8_t *)" Try to reset board ", CENTER_MODE);
  }
  
  // RUN_AND_LOG( BSP_Audio_Init(); );
  BSP_AUDIO_IN_Record(recordBuffer, MY_BUFFER_SIZE_SAMPLES);
  BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_HEADPHONE, 70, DEFAULT_AUDIO_IN_FREQ);
  BSP_AUDIO_OUT_SetAudioFrameSlot(CODEC_AUDIOFRAME_SLOT_02);

  // RUN_AND_LOG( AudioProcessor_Init() );

  xQueue_BufferStatus = xQueueCreate(32, sizeof(BufferStatusMessage_t));

  while (1)
  {
    
    xQueueReceive( xQueue_BufferStatus, &bufferStatusMessage, 1000 );

    //Signal other tasks
    SerialLogger_Signal();
    Monitor_ResetTickCount();  //tracks CPU usage

    switch(bufferStatusMessage)
    {
    case BUFFER_STATUS_LOWER_HALF_FULL:
      {
        // BSP_LED_Blink();
        ExtractSamplesFromDMAReceiveBuffer_LowerHalf(recordBuffer,
                                                     MY_BUFFER_SIZE_SAMPLES / 2);

        // int16_t * outBuf = AudioProcessor_ProcessSampleBuffer(recordBuffer,
        //                                                       MY_BUFFER_SIZE_SAMPLES / 2);

        InsertSamplesIntoDMATransmitBuffer_LowerHalf(recordBuffer,
                                                     MY_BUFFER_SIZE_SAMPLES / 2);
        break;
      }
    case BUFFER_STATUS_UPPER_HALF_FULL:
      {
        // TODO(andrea): this is never reached!
        ExtractSamplesFromDMAReceiveBuffer_UpperHalf(recordBuffer,
                                                     MY_BUFFER_SIZE_SAMPLES / 2);

        // int16_t * outBuf = AudioProcessor_ProcessSampleBuffer(recordBuffer,
        //                                                       MY_BUFFER_SIZE_SAMPLES / 2);

        InsertSamplesIntoDMATransmitBuffer_UpperHalf(recordBuffer,
                                                     MY_BUFFER_SIZE_SAMPLES / 2);
        break;
      }
    }
  }
}
#else

typedef enum
{
  BUFFER_OFFSET_NONE = 0,
  BUFFER_OFFSET_HALF = 1,
  BUFFER_OFFSET_FULL = 2,
}BUFFER_StateTypeDef;

#define AUDIO_BLOCK_SIZE   ((uint32_t)0xFFFE)
// #define AUDIO_BLOCK_SIZE   ((uint32_t)0xFFFE)/4
#define AUDIO_NB_BLOCKS    ((uint32_t)4)

#define RGB565_BYTE_PER_PIXEL     2
#define ARBG8888_BYTE_PER_PIXEL   4

/* Camera have a max resolution of VGA : 640x480 */
#define CAMERA_RES_MAX_X          640
#define CAMERA_RES_MAX_Y          480

/**
  * @brief  LCD FB_StartAddress
  * LCD Frame buffer start address : starts at beginning of SDRAM
  */
#define LCD_FRAME_BUFFER          SDRAM_DEVICE_ADDR

/**
  * @brief  Camera frame buffer start address
  * Assuming LCD frame buffer is of size 480x800 and format ARGB8888 (32 bits per pixel).
  */
#define CAMERA_FRAME_BUFFER       ((uint32_t)(LCD_FRAME_BUFFER + (RK043FN48H_WIDTH * RK043FN48H_HEIGHT * ARBG8888_BYTE_PER_PIXEL)))

/**
  * @brief  SDRAM Write read buffer start address after CAM Frame buffer
  * Assuming Camera frame buffer is of size 640x480 and format RGB565 (16 bits per pixel).
  */
#define SDRAM_WRITE_READ_ADDR        ((uint32_t)(CAMERA_FRAME_BUFFER + (CAMERA_RES_MAX_X * CAMERA_RES_MAX_Y * RGB565_BYTE_PER_PIXEL)))

#define SDRAM_WRITE_READ_ADDR_OFFSET ((uint32_t)0x0800)
#define SRAM_WRITE_READ_ADDR_OFFSET  SDRAM_WRITE_READ_ADDR_OFFSET

#define AUDIO_REC_START_ADDR         SDRAM_WRITE_READ_ADDR

static uint16_t  internal_buffer[AUDIO_BLOCK_SIZE] = {0};

#define AUDIO_BUFFER_SIZE       2048
#define AUDIO_DEFAULT_VOLUME    70

/* Global variables ---------------------------------------------------------*/
uint32_t  audio_rec_buffer_state;

typedef enum {
  AUDIO_ERROR_NONE = 0,
  AUDIO_ERROR_NOTREADY,
  AUDIO_ERROR_IO,
  AUDIO_ERROR_EOF,
}AUDIO_ErrorTypeDef;

typedef enum {
  AUDIO_STATE_IDLE = 0,
  AUDIO_STATE_INIT,
  AUDIO_STATE_PLAYING,
}AUDIO_PLAYBACK_StateTypeDef;

typedef struct {
  uint8_t buff[AUDIO_BUFFER_SIZE];
  uint32_t fptr;  
  BUFFER_StateTypeDef state;
}AUDIO_BufferTypeDef;

static uint32_t  AudioFileSize;
static uint32_t  AudioStartAddress;
static uint32_t GetData(void *pdata, uint32_t offset, uint8_t *pbuf, uint32_t NbrOfData);
AUDIO_ErrorTypeDef AUDIO_Start(uint32_t audio_start_address, uint32_t audio_file_size);

static AUDIO_PLAYBACK_StateTypeDef  audio_state;

ALIGN_32BYTES (static AUDIO_BufferTypeDef  buffer_ctl);

static uint8_t AUDIO_Process(void)
{
  uint32_t bytesread;
  AUDIO_ErrorTypeDef error_state = AUDIO_ERROR_NONE;  
  
  switch(audio_state)
  {
  case AUDIO_STATE_PLAYING:
    
    if(buffer_ctl.fptr >= AudioFileSize)
    {
      /* Play audio sample again ... */
      buffer_ctl.fptr = 0; 
      error_state = AUDIO_ERROR_EOF;
    }

    /* 1st half buffer played; so fill it and continue playing from bottom*/
    if(buffer_ctl.state == BUFFER_OFFSET_HALF)
    {
      bytesread = GetData((void *)AudioStartAddress,
                          buffer_ctl.fptr,
                          &buffer_ctl.buff[0],
                          AUDIO_BUFFER_SIZE /2);
      
      if( bytesread >0)
      { 
        buffer_ctl.state = BUFFER_OFFSET_NONE;
        buffer_ctl.fptr += bytesread; 
        
        /* Clean Data Cache to update the content of the SRAM */
        SCB_CleanDCache_by_Addr((uint32_t*)&buffer_ctl.buff[0], AUDIO_BUFFER_SIZE/2);
      }
    }
    
    /* 2nd half buffer played; so fill it and continue playing from top */    
    if(buffer_ctl.state == BUFFER_OFFSET_FULL)
    {
      bytesread = GetData((void *)AudioStartAddress,
                          buffer_ctl.fptr, 
                          &buffer_ctl.buff[AUDIO_BUFFER_SIZE /2],
                          AUDIO_BUFFER_SIZE /2);
      if( bytesread > 0)
      {
        buffer_ctl.state = BUFFER_OFFSET_NONE;
        buffer_ctl.fptr += bytesread;
        
        /* Clean Data Cache to update the content of the SRAM */
        SCB_CleanDCache_by_Addr((uint32_t*)&buffer_ctl.buff[AUDIO_BUFFER_SIZE/2], AUDIO_BUFFER_SIZE/2);
      }
    }
    break;
    
  default:
    error_state = AUDIO_ERROR_NOTREADY;
    break;
  }
  
  return (uint8_t) error_state;
}

// static uint16_t  total_buffer[AUDIO_BLOCK_SIZE * AUDIO_NB_BLOCKS] = {0};

void My_Audio_Task(void * argument)
{
  uint32_t buff_address = AUDIO_REC_START_ADDR;
  

  // uint32_t buff_address = (uint32_t)total_buffer;
  uint32_t  block_number;
  uint8_t  text[50];

  // AudioRec_SetHint();

  /* Initialize Audio Recorder */
  if (BSP_AUDIO_IN_Init(DEFAULT_AUDIO_IN_FREQ, DEFAULT_AUDIO_IN_BIT_RESOLUTION, DEFAULT_AUDIO_IN_CHANNEL_NBR) == AUDIO_OK)
  {
    BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
    BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 95, (uint8_t *)"  AUDIO RECORD INIT OK  ", CENTER_MODE);

    BSP_AUDIO_IN_SetVolume(70);
    BSP_AUDIO_OUT_SetVolume(70);
  }
  else
  {
    BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
    BSP_LCD_SetTextColor(LCD_COLOR_RED);
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 95, (uint8_t *)"  AUDIO RECORD INIT FAIL", CENTER_MODE);
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 80, (uint8_t *)" Try to reset board ", CENTER_MODE);
  }

  audio_rec_buffer_state = BUFFER_OFFSET_NONE;

  /* Display the state on the screen */
  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
  BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 80, (uint8_t *)"       RECORDING...     ", CENTER_MODE);

  /* Start Recording */
  BSP_AUDIO_IN_Record(internal_buffer, AUDIO_BLOCK_SIZE);

  for (block_number = 0; block_number < AUDIO_NB_BLOCKS; block_number++)
  {
    /* Wait end of half block recording */
    while(audio_rec_buffer_state != BUFFER_OFFSET_HALF)
    {
        //
      HAL_Delay(10);
      // if (CheckForUserInput() > 0)
      // {
      //   /* Stop Player before close Test */
      //   BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW);
      //   return;
      // }
    }
    audio_rec_buffer_state = BUFFER_OFFSET_NONE;
        /* Copy recorded 1st half block in SDRAM */
    // for (int i = 0; i < (int)AUDIO_BLOCK_SIZE; i++)
    //     *((uint32_t *)(buff_address +(block_number * AUDIO_BLOCK_SIZE * 2))+i) = internal_buffer[i];
    memcpy((uint32_t *)(buff_address + (block_number * AUDIO_BLOCK_SIZE * 2)),
           internal_buffer,
           AUDIO_BLOCK_SIZE);

    /* Wait end of one block recording */
    while(audio_rec_buffer_state != BUFFER_OFFSET_FULL)
    {
      HAL_Delay(10);
      // if (CheckForUserInput() > 0)
      // {
      //   /* Stop Player before close Test */
      //   BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW);
      //   return;
      // }
    }
    audio_rec_buffer_state = BUFFER_OFFSET_NONE;
    /* Copy recorded 2nd half block in SDRAM */
    memcpy((uint32_t *)(buff_address + (block_number * AUDIO_BLOCK_SIZE * 2) + (AUDIO_BLOCK_SIZE)),
           (uint16_t *)(&internal_buffer[AUDIO_BLOCK_SIZE/2]),
           AUDIO_BLOCK_SIZE);
  }

  /* Stop recorder */
  BSP_AUDIO_IN_Stop(CODEC_PDWN_SW);

  uint16_t max_val = 0;
  uint16_t sat_count = 0;
  float avg = 0.0;
  for(int i = 0; i < AUDIO_BLOCK_SIZE; i++) {
	  avg += (float) internal_buffer[i];
  	  if (max_val < internal_buffer[i]) {
  		  max_val = internal_buffer[i];
  	  }
  	if (internal_buffer[i] > (uint16_t) 65533) {
  	  		  max_val = internal_buffer[i];
  	  	  }

  }
  avg = avg / ((float) AUDIO_BLOCK_SIZE);

  sprintf((char*)text, "max = %u, avg = %u, sat_cout = %u", (uint16_t) max_val, (uint16_t) avg, sat_count);
  BSP_LCD_DisplayStringAt(15, BSP_LCD_GetYSize() - 25, (uint8_t *)&text, LEFT_MODE);


  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
  BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 65, (uint8_t *)"RECORDING DONE, START PLAYBACK...", CENTER_MODE);

  /* -----------Start Playback -------------- */
  /* Initialize audio IN at REC_FREQ*/ 
  BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_HEADPHONE, 70, DEFAULT_AUDIO_IN_FREQ);
  BSP_AUDIO_OUT_SetAudioFrameSlot(CODEC_AUDIOFRAME_SLOT_02);

  /* Play the recorded buffer*/
  AUDIO_Start(buff_address, AUDIO_BLOCK_SIZE * AUDIO_NB_BLOCKS * 2);  /* Use Audio play demo to playback sound */
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 40, (uint8_t *)"PLAYBACK DONE", CENTER_MODE);

  while (1)
  {
    AUDIO_Process();

    HAL_Delay(10);
    
    // if (CheckForUserInput() > 0)
    // {
    //   /* Stop Player before close Test */
    //   BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW);
    //   return;
    // }
  }
}

static uint32_t GetData(void *pdata, uint32_t offset, uint8_t *pbuf, uint32_t NbrOfData)
{
  uint8_t *lptr = pdata;
  uint32_t ReadDataNbr;
  
  ReadDataNbr = 0;
  while(((offset + ReadDataNbr) < AudioFileSize) && (ReadDataNbr < NbrOfData))
  {
    pbuf[ReadDataNbr]= lptr [offset + ReadDataNbr];
    ReadDataNbr++;
  }
  return ReadDataNbr;
}

AUDIO_ErrorTypeDef AUDIO_Start(uint32_t audio_start_address, uint32_t audio_file_size)
{
  uint32_t bytesread;
  
  buffer_ctl.state = BUFFER_OFFSET_NONE;
  AudioStartAddress = audio_start_address;
  AudioFileSize = audio_file_size;
  bytesread = GetData( (void *)AudioStartAddress,
                      0,
                      &buffer_ctl.buff[0],
                      AUDIO_BUFFER_SIZE);
  if(bytesread > 0)
  {
    /* Clean Data Cache to update the content of the SRAM */
    SCB_CleanDCache_by_Addr((uint32_t*)&buffer_ctl.buff[0], AUDIO_BUFFER_SIZE/2);
        
    BSP_AUDIO_OUT_Play((uint16_t*)&buffer_ctl.buff[0], AUDIO_BUFFER_SIZE);
    audio_state = AUDIO_STATE_PLAYING;      
    buffer_ctl.fptr = bytesread;
    return AUDIO_ERROR_NONE;
  }
  return AUDIO_ERROR_IO;
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

void BSP_AUDIO_OUT_TransferComplete_CallBack(void)
{
  if(audio_state == AUDIO_STATE_PLAYING)
  {
    /* allows AUDIO_Process() to refill 2nd part of the buffer  */
    buffer_ctl.state = BUFFER_OFFSET_FULL;
  }
}

/**
  * @brief  Manages the DMA Half Transfer complete event.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_OUT_HalfTransfer_CallBack(void)
{
  if(audio_state == AUDIO_STATE_PLAYING)
  {
    /* allows AUDIO_Process() to refill 1st part of the buffer  */
    buffer_ctl.state = BUFFER_OFFSET_HALF;
  }
}
#endif

// DMA receive complete ISRs

void My_AUDIO_IN_TransferComplete_CallBack(void)
{
  LOG_ONESHOT("AUDIO IN COMPLETE");
  static BufferStatusMessage_t msg = BUFFER_STATUS_UPPER_HALF_FULL;
  static BaseType_t higherPriorityTaskWoken = 0;
  xQueueSendFromISR( xQueue_BufferStatus, &msg, &higherPriorityTaskWoken );
}

void My_AUDIO_IN_HalfTransfer_CallBack(void)
{
  LOG_ONESHOT("AUDIO IN HALF");
  static BufferStatusMessage_t msg = BUFFER_STATUS_LOWER_HALF_FULL;
  static BaseType_t higherPriorityTaskWoken = 0;
  xQueueSendFromISR( xQueue_BufferStatus, &msg, &higherPriorityTaskWoken );
}

void My_AUDIO_OUT_Error_CallBack(void){
}

