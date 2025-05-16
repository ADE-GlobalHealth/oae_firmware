/**
 * ADC functions for device use.
 */

#pragma once

#include <stm32l4xx_hal.h>
#include <stm32l4xx_hal_sai.h>

#include "tlv320adc3120.h"

extern SAI_HandleTypeDef hsai_BlockA2;

#define BUFFER_SIZE ((uint16_t) 4096)

extern volatile uint32_t data_i2s_0[BUFFER_SIZE];
extern volatile uint32_t data_i2s_1[BUFFER_SIZE];

/**
 * Initialize the ADC using a configuration.
 */
void init_adc(void);

/**
 * Connect ADC input to the DMA and collect ADC samples over the audio serial
 * interface.
 *
 * @param data_buffer (int32_t *) A pointer to the buffer to store ADC samples.
 */
void dma_adc_input(int32_t *data_buffer);
