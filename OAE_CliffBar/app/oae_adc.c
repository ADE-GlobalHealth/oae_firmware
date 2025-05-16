#include "oae_adc.h"

void init_adc(void) {
	tlv_config_t adc_config = {
		.areg_select = INTERNAL,
		.asi_protocol = I2S,
		.asi_word_length = LENGTH_24b,
		.gpio1_function = IRQ,
		.in_ch_en = {1,1,0,0}, // enable channels 1 & 2
		.out_ch_en = {1,1,0,0}, // enable channels 1 & 2
		.micbias_en = ENABLED,
		.adc_en = ENABLED,
		.pll_en = ENABLED,
	};

	tlv_init(adc_config);

	// wake up from sleep
	tlv_sleep(DISABLED);
}


void dma_adc_input(int32_t* data_buffer) {
	HAL_SAI_Receive_DMA(&hsai_BlockA2, (uint8_t*) data_buffer, sizeof(*data_buffer));
}


/**
 * DMA full callback - called when DMA is full. This ensures the SAI stops when
 * the DMA is full.
 *
 * TODO: If ping-pong buffers are truly needed, these functions should change or
 * additional functions should be added to handle data flow from the ADC to the
 * DMA with two distinct buffers.
 */
void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai) {
    HAL_SAI_DMAStop(hsai);
}
