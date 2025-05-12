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
	};

	tlv_init(adc_config);
}


