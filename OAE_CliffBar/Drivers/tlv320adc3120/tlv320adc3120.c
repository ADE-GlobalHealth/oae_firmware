/**
 * tlv320adc3120 driver using memory page headers (tlv320adcx120_pagex.h) from
 * Texas Instruments.
 *
 * Authors: Drew Pang
 */

#include <stdint.h>

#include "tlv320adc3120.h"

void _tlv_read_register(uint8_t address, uint8_t *data) {
	HAL_I2C_Mem_Read(&hi2c3, TLV_DEVICE_ID, address, 1, data, 1,
			HAL_MAX_DELAY);
}


void _tlv_write_register(uint8_t address, uint8_t data) {
	HAL_I2C_Mem_Write(&hi2c3, TLV_DEVICE_ID, address, 1, data, 1,
			HAL_MAX_DELAY);
}


void _tlv_write_register_mask(uint8_t address, uint8_t mask, uint8_t data) {
	// read data in register
	uint8_t read_data;
	_tlv_read_register(address, read_data);

	// clear mask bits and or with write data
	// NOTE: UB possible if mask does not cover the write data
	read_data &= ~mask;
	read_data |= data;
	_tlv_write_register(address, read_data);
}


void _tlv_switch_register_page(uint8_t page_number) {
	_tlv_write_register(PAGE_CFG_ADDRESS, page_number);
}


void tlv_init(tlv_config_t tlv_config) {
	// perform a software reset to ensure TLV is at default
	tlv_software_reset();

	// AREG select
	switch (tlv_config.areg_select) {
	case INTERNAL:
		_tlv_write_register_mask(SLEEP_CFG_ADDRESS, SLEEP_CFG_AREG_SELECT_MASK,
		SLEEP_CFG_AREG_SELECT_INTERNAL);
	case EXTERNAL:
		_tlv_write_register_mask(SLEEP_CFG_ADDRESS, SLEEP_CFG_AREG_SELECT_MASK,
		SLEEP_CFG_AREG_SELECT_EXTERNAL);
	}

	// ASI protocol
	switch (tlv_config.asi_protocol) {
	case TDM:
		_tlv_write_register_mask(ASI_CFG0_ADDRESS, ASI_CFG0_FORMAT_MASK,
		ASI_CFG0_FORMAT_TDM);
	case I2S:
		_tlv_write_register_mask(ASI_CFG0_ADDRESS, ASI_CFG0_FORMAT_MASK,
		ASI_CFG0_FORMAT_I2S);
	case LJ:
		_tlv_write_register_mask(ASI_CFG0_ADDRESS, ASI_CFG0_FORMAT_MASK,
		ASI_CFG0_FORMAT_LJ);
	}

	// ASI word length
	switch (tlv_config.asi_word_length) {
	case LENGTH_16b:
		_tlv_write_register_mask(ASI_CFG0_ADDRESS, ASI_CFG0_WLEN_MASK,
		ASI_CFG0_WLEN_16_BITS);
	case LENGTH_20b:
		_tlv_write_register_mask(ASI_CFG0_ADDRESS, ASI_CFG0_WLEN_MASK,
		ASI_CFG0_WLEN_20_BITS);
	case LENGTH_24b:
		_tlv_write_register_mask(ASI_CFG0_ADDRESS, ASI_CFG0_WLEN_MASK,
		ASI_CFG0_WLEN_24_BITS);
	case LENGTH_32b:
		_tlv_write_register_mask(ASI_CFG0_ADDRESS, ASI_CFG0_WLEN_MASK,
		ASI_CFG0_WLEN_32_BITS);
	}

	// GPIO1 function
	switch (tlv_config.gpio1_function) {
	case DISABLED:
		_tlv_write_register_mask(GPIO_CFG0_ADDRESS, GPIO_CFG0_GPIO1_CFG_MASK,
				GPIO_CFG0_GPIO1_CFG_DISABLED);
	case GPI:
		_tlv_write_register_mask(GPIO_CFG0_ADDRESS, GPIO_CFG0_GPIO1_CFG_MASK,
		GPIO_CFG0_GPIO1_CFG_GPI);
	case GPO:
		_tlv_write_register_mask(GPIO_CFG0_ADDRESS, GPIO_CFG0_GPIO1_CFG_MASK,
		GPIO_CFG0_GPIO1_CFG_GPO);
	case IRQ:
		_tlv_write_register_mask(GPIO_CFG0_ADDRESS, GPIO_CFG0_GPIO1_CFG_MASK,
		GPIO_CFG0_GPIO1_CFG_IRQ);
	}

	// analog input channel enable
	uint8_t analog_in_data = IN_CH_EN_DEFAULT;
	for (uint8_t i = 0; i < 4; i++) {
		analog_in_data &= ~(0b1 << (8 - i));
		analog_in_data |= tlv_config.in_ch_en[i] << (8 - i);
	}
	// mask not required, register is only used for analog channel
	// enable/disable
	_tlv_write_register(IN_CH_EN_ADDRESS, analog_in_data);

	// digital output channel enable
	uint8_t digital_out_data = ASI_OUT_CH_EN_DEFAULT;
	for (uint8_t i = 0; i < 4; i++) {
		digital_out_data &= ~(0b1 << (8 - i));
		digital_out_data |= tlv_config.out_ch_en[i] << (8 - i);
	}
	// mask not required, register is only used for digital channel
	// enable/disable
	_tlv_write_register(ASI_OUT_CH_EN_ADDRESS, digital_out_data);

	// micbias enable
	switch (tlv_config.micbias_en) {
	case ENABLED:
		_tlv_write_register_mask(PWR_CFG_ADDRESS, PWR_CFG_MICBIAS_PDZ_MASK,
		PWR_CFG_MICBIAS_PDZ_ON);
	case DISABLED:
		_tlv_write_register_mask(PWR_CFG_ADDRESS, PWR_CFG_MICBIAS_PDZ_MASK,
		PWR_CFG_MICBIAS_PDZ_OFF);
	}

	// ADC enable
	switch (tlv_config.adc_en) {
	case ENABLED:
		_tlv_write_register_mask(PWR_CFG_ADDRESS, PWR_CFG_ADC_PDZ_MASK,
		PWR_CFG_ADC_PDZ_ON);
	case DISABLED:
		_tlv_write_register_mask(PWR_CFG_ADDRESS, PWR_CFG_ADC_PDZ_MASK,
		PWR_CFG_ADC_PDZ_OFF);
	}

}


void tlv_start_adc_conversions(void) {
	;
}


void tlv_software_reset(void) {
	// rest of register is 0 by default, no masking required
	_tlv_write_register(SW_RESET_ADDRESS, SW_RESET_RESET);
}


void tlv_sleep(enable_e enabled) {
	if (enabled == ENABLED) {
		_tlv_write_register_mask(SLEEP_CFG_ADDRESS, SLEEP_CFG_SLEEP_ENZ_MASK,
		SLEEP_CFG_SLEEP_ENZ_SLEEP);
	} else {
		_tlv_write_register_mask(SLEEP_CFG_ADDRESS, SLEEP_CFG_SLEEP_ENZ_MASK,
		SLEEP_CFG_SLEEP_ENZ_ACTIVE);
	}
}
