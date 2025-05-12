/**
 * ADC functions for device use.
 */

#pragma once

#include <stm32l4xx_hal.h>
#include <stm32l4xx_hal_sai.h>

#include "tlv320adc3120.h"

extern SAI_HandleTypeDef hsai_BlockA2;

#define BUFFER_SIZE 4096

extern volatile uint32_t data_i2s_0[BUFFER_SIZE];
extern volatile uint32_t data_i2s_1[BUFFER_SIZE];

/**
 * Initialize the ADC using a configuration.
 */
void init_adc(void);

/**
 * Connect ADC input to the DMA and begin serial audio input.
 */
void start_dma_adc_input(void);
