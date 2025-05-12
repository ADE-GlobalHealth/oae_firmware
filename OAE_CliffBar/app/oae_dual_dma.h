/*
 * dual_dma.h
 *
 *  Created on: Sep 21, 2023
 *      Author: veswaranandam
 */

#ifndef OAE_DUAL_DMA_H_
#define OAE_DUAL_DMA_H_

#include "stm32l4xx_hal.h"

#define DAC_CHANNEL_12D ((uint32_t) 0x00000011)

HAL_StatusTypeDef HAL_DAC_Start_DualDMA(DAC_HandleTypeDef *, uint32_t, uint32_t *, uint32_t, uint32_t);


#endif /* OAE_DUAL_DMA_H_ */
