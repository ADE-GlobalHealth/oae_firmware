/**
 * tlv320adc3120 driver using memory page headers (tlv320adcx120_pagex.h) from
 * Texas Instruments.
 *
 * This library does not cover all tlv320adc3120 functions, and default are used
 * for most registers. For instructions on adding additional functionality, see:
 * TODO
 *
 * Authors: Drew Pang
 */

extern I2C_HandleTypeDef hi2c3;

#include "tlv320adcx120_page0.h"
// page 1 contains voice detection registers which are not needed
//#include "tlv320adcx120_page1.h"
#include "tlv320adcx120_page2.h"
#include "tlv320adcx120_page3.h"
#include "tlv320adcx120_page4.h"
#include "tlv320adcx120_page5.h"
#include "tlv320adcx120_page6.h"

// TLV I2C device ID
# define TLV_DEVICE_ID  ((uint8_t) 0b1001110)

/**
 * TLV ADC configuration struct.
 *
 * in_ch_en and out_ch_en refer to the analog (INxP/M) and digital (ASI) inputs
 * and outputs respectively. These are arrays where the channel increments
 * each element of the array
 * ex: .in_ch_en = { ENABLED, DISABLED, ENABLED, DISABLED } enables channels 1 & 3.
 */
typedef struct {
	areg_select_e areg_select;
	asi_protocol_e asi_protocol;
	asi_word_length_e asi_word_length;
	gpio1_function_e gpio1_function;
	enable_e in_ch_en[4];
	enable_e out_ch_en[4];
	enable_e micbias_en;
	enable_e adc_en;
} tlv_config_t;


/**
 * Internal/external AREG enum.
 */
typedef enum {
	INTERNAL, EXTERNAL
} areg_select_e;


/**
 * Audio serial interface protocol format enum.
 */
typedef enum {
	TDM, I2S, LJ
} asi_protocol_e;


/**
 * Audio serial interface word length enum.
 */
typedef enum {
	LENGTH_16b, LENGTH_20b, LENGTH_24b, LENGTH_32b
} asi_word_length_e;


/**
 * GPIO1 configuration enum. For full list of functionality, see table 8-70 in
 * the datasheet.
 */
typedef enum {
	DISABLED, GPIO, IRQ
} gpio1_function_e;


/**
 * General purpose enable/disable enum.
 */
typedef enum {
	DISABLED, ENABLED
} enable_e;


/**
 * Audio serial interface output enum.
 */
typedef enum {
	LENGTH_20b, LENGTH_24b, LENGTH_32b
} asi_output_ch_en_e;


/**
 * Read a register in the ADC.
 *
 * @param address (uint8_t) The register address to read.
 * @param data (uint8_t *) A pointer to store the data.
 */
void _tlv_read_register(uint8_t address, uint8_t *data);


/**
 * Write a register in the ADC.
 *
 * @param address (uint8_t) The register address to write.
 * @param data (uint8_t) The data to write to the register.
 */
void _tlv_write_register(uint8_t address, uint8_t data);

/**
 * Write a register in the ADC using a mask.
 *
 * Useful for only writing bits in the mask. Bits not contained in the mask will
 * not be changed.
 *
 * @param address (uint8_t) The register address to write.
 * @param mask (uint8_t) The mask to use for clearing and setting bits.
 * @param data (uint8_t ) The data to write to the register.
 */
void _tlv_write_register_mask(uint8_t address, uint8_t mask, uint8_t data);


/**
 * Switch register page in the ADC.
 *
 * @param page_number (uint8_t) The register page to switch to.
 */
void _tlv_switch_register_page(uint8_t page_number);


/**
 * Initialize the ADC with a device config.
 *
 * @param tlv_config (tlv_config_t) The config to initialize the TLV with.
 */
void tlv_init(tlv_config_t tlv_config);


/**
 * Start ADC conversions and buffer data in a given memory address.
 *
 * @param
 */
void tlv_start_adc_conversions(void);


/**
 * Software reset the ADC.
 */
void tlv_software_reset(void);


/**
 * Put the TLV ADC to sleep.
 *
 * @ param enabled (enabled_e) If enabled, the TLV is put to sleep.
 */
void tlv_sleep(enable_e enabled);
