#include "BSP_LED.h"
#include "stm32f7xx_hal.h"
#include "stm32746g_discovery.h"


// #define LED1_GPIO_PORT                   ((GPIO_TypeDef*)GPIOI)
// #define LED2_GPIO_PORT                   ((GPIO_TypeDef*)GPIOI)

// #define LED1_PIN                         ((uint32_t)GPIO_PIN_1)
// #define LED2_PIN                         ((uint32_t)GPIO_PIN_5)

// uint32_t GPIO_PIN[] = {LED1_PIN,
//                            LED2_PIN};

// GPIO_TypeDef* GPIO_PORT[] = {LED1_GPIO_PORT,
//                                  LED2_GPIO_PORT};

#define LEDx_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOJ_CLK_ENABLE()

void BSP_LED_Init()
{

  GPIO_InitTypeDef gpioInitStructure;

  __HAL_RCC_GPIOI_CLK_ENABLE();
  gpioInitStructure.Pin = GPIO_PIN_1;
  gpioInitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  gpioInitStructure.Pull = GPIO_PULLUP;
  gpioInitStructure.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOI, &gpioInitStructure);

}

void BSP_LED_On()
{
  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_1, GPIO_PIN_SET);
}

/**
 * @brief  Turns selected LED Off.
 * @param  Led: LED to be set off
 *          This parameter can be one of the following values:
 *            @arg  LED1
 *            @arg  LED2
 * @retval None
 */
void BSP_LED_Off()
{
  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_1, GPIO_PIN_RESET);
}

/**
 * @brief  Toggles the selected LED.
 * @param  Led: LED to be toggled
 *          This parameter can be one of the following values:
 *            @arg  LED1
 *            @arg  LED2
 * @retval None
 */
void BSP_LED_Toggle()
{
  HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_1);
}

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

