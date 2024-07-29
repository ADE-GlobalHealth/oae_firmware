#include "stm32l4xx_hal.h"

#define DAC_CHANNEL_12D ((uint32_t) 0x00000011)

HAL_StatusTypeDef HAL_DAC_Start_DualDMA(DAC_HandleTypeDef *, uint32_t, uint32_t *, uint32_t, uint32_t);

