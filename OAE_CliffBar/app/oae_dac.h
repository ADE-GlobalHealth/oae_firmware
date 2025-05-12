/**
 * DAC functions for device use.
 */

#pragma once

#include <oae_dual_dma.h>
#include <stm32l4xx_hal.h>
#include <stm32l4xx_hal_dac.h>

//extern DMA_HandleTypeDef hdma_dac_ch1; TODO: remove if not used
extern TIM_HandleTypeDef htim6;
extern DAC_HandleTypeDef hdac1;

// TODO: differentiate between these
#define DAC_LUT_SIZE 128
#define DAC_LUT_SAMPLES 4096

// DAC output look up table. Generating using tools/lut_gen.py.
extern uint32_t dac_wave_lut[DAC_LUT_SAMPLES];

/**
 * Initialize timers used for DAC output.
 */
void init_dac(void);

/**
 * Start a dual DMA output with two DAC outputs.
 * TODO: finish docstring
 */
void start_dual_dma(void);
