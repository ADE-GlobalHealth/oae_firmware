#include <stdint.h>
#include <stm32l4xx_hal.h>

#include "app_lib.h"

void delay_non_blocking(uint32_t delay)
{
  uint32_t tickstart = HAL_GetTick();
  uint32_t wait = delay;

  // Add a period to guarantee minimum wait
  if (wait < HAL_MAX_DELAY) {
    wait += (uint32_t)uwTickFreq;
  }

  while ((HAL_GetTick() - tickstart) < wait){}
}
