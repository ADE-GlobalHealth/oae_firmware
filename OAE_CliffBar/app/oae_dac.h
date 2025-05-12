/**
 * DAC functions for device use.
 */

#pragma once

#include <oae_dual_dma.h>
#include <stm32l4xx_hal.h>
#include <stm32l4xx_hal_dac.h>

extern TIM_HandleTypeDef htim6;
extern DAC_HandleTypeDef hdac1;

#define DAC_LUT_SAMPLES 128

// DAC output look up table. Generating using tools/lut_gen.py.
extern uint32_t dac_wave_lut[DAC_LUT_SAMPLES];

/**
 * Initialize timers used for DAC output.
 */
void init_dac(void);

/**
 * Start dual DMA output with the two DAC outputs.
 */
void start_dac_output(void);

/**
 * Stop dual DMA output of the two DAC outputs.
 */
void stop_dac_output(void);
