#include "BSP_LED.h"
#include "stm32f7xx_hal.h"
#include "stm32746g_discovery.h"

void BSP_LED_Blink() 
{
  int i;
  for(i=0; i<4; i++)
  {
    if (i % 2)
      BSP_LED_Off();
    else
      BSP_LED_On();
    for (int j = 0; j < 100000; j++)
      __asm volatile("nop");
  }
}

