/**
 * tlv320adc3120 driver using memory page headers (tlv320adcx120_pagex.h) from
 * Texas Instruments.
 *
 * Authors: Drew Pang
 */

#include <stdint.h>

#include "tlv320adc3120.h"

void _tlv_read_register(uint8_t address, uint8_t data) {
	HAL_I2C_Mem_Read(&hi2c3, TLV_DEVICE_ID, address, 1, &data, 1,
			HAL_MAX_DELAY);
}


void _tlv_write_register(uint8_t address, uint8_t data) {
	HAL_I2C_Mem_Write(&hi2c3, TLV_DEVICE_ID, address, 1, &data, 1,
			HAL_MAX_DELAY);
}


void _tlv_switch_register_page(uint8_t page_number) {
	uint8_t switch_register_page_data = 0;
	switch_register_page_data &= PAGE_CFG_PAGE_MASK;
	switch_register_page_data |= page_number;
	_tlv_write_register(PAGE_CFG_ADDRESS, switch_register_page_data);
}


void tlv_init(void) {
	;
}


void tlv_start_adc_conversions(void) {
	// perform a software reset to ensure tlv is at default
	tlv_software_reset();
}


void tlv_software_reset(void) {
	uint8_t sw_reset_data = 0;
	sw_reset_data &= ~SW_RESET_MASK;
	sw_reset_data |= SW_RESET_RESET;
	_tlv_write_register(SW_RESET_ADDRESS, sw_reset_data);
}


void tlv_sleep(uint8_t enabled) {
	uint8_t sleep_data = 0;
	sleep_data &= ~SLEEP_CFG_SLEEP_ENZ_MASK;
	sleep_data |= enabled;
	_tlv_write_register(sleep_data);
}
