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

	// wake up from sleep
	tlv_sleep(DISABLED);

	HAL_Delay(10);
	tlv_init(adc_config);
	start_dma_adc_input();

	
}

// TODO: these ideally should not be global variables
volatile uint32_t data_i2s_0[BUFFER_SIZE];
volatile uint32_t data_i2s_1[BUFFER_SIZE];

void start_dma_adc_input(void) {
	HAL_SAI_Receive_DMA(&hsai_BlockA2, (uint8_t*) data_i2s_0, BUFFER_SIZE);
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai) {
    if (hsai->Instance == hsai_BlockA2.Instance) {
        __NOP();
    }
}






