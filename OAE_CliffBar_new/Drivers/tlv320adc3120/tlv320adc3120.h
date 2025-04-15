/**
 * tlv320adc3120 driver using memory page headers (tlv320adcx120_pagex.h) from
 * Texas Instruments.
 *
 * Authors: Drew Pang
 */

#include "tlv320adcx120_page0.h"
// page 1 contains voice detection registers which are not needed
//#include "tlv320adcx120_page1.h"
#include "tlv320adcx120_page2.h"
#include "tlv320adcx120_page3.h"
#include "tlv320adcx120_page4.h"
#include "tlv320adcx120_page5.h"
#include "tlv320adcx120_page6.h"

/**
 * Read a register in the ADC.
 *
 * @param
 */
void tlv_read_register(void);

/**
 * Write a register in the ADC.
 *
 * @param
 */
void tlv_write_register(void);

/**
 * Switch register page in the ADC.
 *
 * @param
 */
void tlv_switch_register_page(void);


/**
 * Initialize the ADC with a device config.
 *
 * @param
 */
void tlv_init(void);


/**
 * Start ADC conversions and buffer data in a given memory address.
 *
 * @param
 */
void tlv_start_adc_conversions(void);
