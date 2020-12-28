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

#define AUDIO_BLOCK_SIZE   2*((uint32_t)512)

#define SDRAM 0

#if SDRAM
#define AUDIO_BUFFER_SIZE       2048
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
#define AUDIO_BUFFER_IN    AUDIO_REC_START_ADDR     /* In SDRAM */
#define AUDIO_BUFFER_OUT   (AUDIO_REC_START_ADDR + (AUDIO_BLOCK_SIZE * 2)) /* In SDRAM */

#else

int16_t audio_buffer_in[AUDIO_BLOCK_SIZE/2] = {1};
int16_t audio_buffer_out[AUDIO_BLOCK_SIZE/2] = {1};

#define MY_BUFFER_SIZE_SAMPLES 1024

#define MY_DMA_BYTES_PER_FRAME 8
#define MY_DMA_BYTES_PER_MSIZE 2
#define MY_DMA_BUFFER_SIZE_BYTES MY_BUFFER_SIZE_SAMPLES * MY_DMA_BYTES_PER_FRAME
#define MY_DMA_BUFFER_SIZE_MSIZES MY_DMA_BUFFER_SIZE_BYTES / MY_DMA_BYTES_PER_MSIZE

static uint8_t saiDMATransmitBuffer[MY_DMA_BUFFER_SIZE_BYTES];
static uint8_t saiDMAReceiveBuffer[MY_DMA_BUFFER_SIZE_BYTES];

static void ExtractSamplesFromDMAReceiveBuffer_LowerHalf(int16_t * sampleBuffer, uint32_t num_samples)
{
  for(uint32_t i = 0; i<num_samples; i++){
    int16_t * samplePointer = (int16_t *) & saiDMAReceiveBuffer[i*8];
    sampleBuffer[i] = *samplePointer;
  }
}

static void ExtractSamplesFromDMAReceiveBuffer_UpperHalf(int16_t * sampleBuffer, uint32_t num_samples)
{
  for(uint32_t i = 0; i<num_samples; i++){
    int16_t * samplePointer = (int16_t *) & saiDMAReceiveBuffer[(MY_DMA_BUFFER_SIZE_BYTES / 2) + i*8];
    sampleBuffer[i] = *samplePointer;
  }
}

static void InsertSamplesIntoDMATransmitBuffer_LowerHalf(int16_t * sampleBuffer, uint32_t num_samples)
{
  for(uint32_t i=0; i<num_samples; i++){
    int16_t * p = (int16_t *) &saiDMATransmitBuffer[i*8];
    *p = sampleBuffer[i];
    *(p+2) = sampleBuffer[i];
  }
}

static void InsertSamplesIntoDMATransmitBuffer_UpperHalf(int16_t * sampleBuffer, uint32_t num_samples)
{
  for(uint32_t i=0; i<num_samples; i++){
    int16_t * p = (int16_t *) &saiDMATransmitBuffer[(MY_DMA_BUFFER_SIZE_BYTES / 2) + i*8];
    *p = sampleBuffer[i];
    *(p+2) = sampleBuffer[i];
  }
}

#endif

uint32_t  audio_rec_buffer_state;

// typedef enum
// {
//     BUFFER_OFFSET_NONE = 0,
//     BUFFER_OFFSET_HALF = 1,
//     BUFFER_OFFSET_FULL = 2,
// }BUFFER_StateTypeDef;

typedef uint32_t BufferStatusMessage_t ;

static QueueHandle_t xQueue_BufferStatus;
static BufferStatusMessage_t bufferStatusMessage;

void LoopBack_Task(void * argument)
{
    uint8_t  text[30];

    /* Initialize Audio Recorder */
    int status = BSP_AUDIO_IN_OUT_Init(INPUT_DEVICE_INPUT_LINE_1, OUTPUT_DEVICE_HEADPHONE, DEFAULT_AUDIO_IN_FREQ, DEFAULT_AUDIO_IN_BIT_RESOLUTION, DEFAULT_AUDIO_IN_CHANNEL_NBR);

#if SDRAM
    /* Initialize SDRAM buffers */
    memset((uint16_t*)AUDIO_BUFFER_IN, 0, AUDIO_BLOCK_SIZE*2);
    memset((uint16_t*)AUDIO_BUFFER_OUT, 0, AUDIO_BLOCK_SIZE*2);
#endif
    // audio_rec_buffer_state = BUFFER_OFFSET_NONE;

    BSP_AUDIO_IN_SetVolume(70);
    BSP_AUDIO_OUT_SetVolume(70);

    /* Start Recording */
#if SDRAM
    BSP_AUDIO_IN_Record((uint16_t*)AUDIO_BUFFER_IN, AUDIO_BLOCK_SIZE);
    // BSP_AUDIO_IN_Record((uint16_t*)AUDIO_BUFFER_OUT, AUDIO_BLOCK_SIZE);

    // /* Start Playback */
    // BSP_AUDIO_OUT_SetAudioFrameSlot(CODEC_AUDIOFRAME_SLOT_02);
    BSP_AUDIO_OUT_Play((uint16_t*)AUDIO_BUFFER_OUT, AUDIO_BLOCK_SIZE);
#else
    BSP_AUDIO_HAL_SAI_Receive_DMA((uint8_t*)saiDMAReceiveBuffer, MY_DMA_BUFFER_SIZE_MSIZES);

    // /* Start Playback */
    BSP_AUDIO_HAL_SAI_Transmit_DMA((uint8_t*)saiDMATransmitBuffer, MY_DMA_BUFFER_SIZE_MSIZES);
#endif

    xQueue_BufferStatus = xQueueCreate(32, sizeof(BufferStatusMessage_t));

    while (1)
    {
        xQueueReceive( xQueue_BufferStatus, &bufferStatusMessage, 1000 );

        //Signal other tasks
        SerialLogger_Signal();
        Monitor_ResetTickCount();  //tracks CPU usage

        BSP_LED_Blink();
        // TODO(andrea): figure out why memcpy does not work!
        switch(bufferStatusMessage)
        {
        case BUFFER_STATUS_LOWER_HALF_FULL:

        for(int i =0; i < MY_DMA_BUFFER_SIZE_BYTES/2; i++)
            saiDMATransmitBuffer[i] = saiDMAReceiveBuffer[i]; 

        break;

        case BUFFER_STATUS_UPPER_HALF_FULL:

        for(int i =0; i < MY_DMA_BUFFER_SIZE_BYTES/2; i++)
            saiDMATransmitBuffer[MY_DMA_BUFFER_SIZE_BYTES/2+i] = saiDMAReceiveBuffer[MY_DMA_BUFFER_SIZE_BYTES/2+i]; 

        break;
        }
    }
}

void BSP_AUDIO_IN_TransferComplete_CallBack(void)
{
  LOG_ONESHOT("AUDIO IN COMPLETE");
  static BufferStatusMessage_t msg = BUFFER_STATUS_UPPER_HALF_FULL;
  static BaseType_t higherPriorityTaskWoken = 0;
  xQueueSendFromISR( xQueue_BufferStatus, &msg, &higherPriorityTaskWoken );
}

void BSP_AUDIO_IN_HalfTransfer_CallBack(void)
{
  LOG_ONESHOT("AUDIO IN HALF");
  static BufferStatusMessage_t msg = BUFFER_STATUS_LOWER_HALF_FULL;
  static BaseType_t higherPriorityTaskWoken = 0;
  xQueueSendFromISR( xQueue_BufferStatus, &msg, &higherPriorityTaskWoken );
}
