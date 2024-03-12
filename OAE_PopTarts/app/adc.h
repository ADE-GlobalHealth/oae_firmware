#ifndef ADC_H_
#define ADC_H_

#include <stm32l4xx_hal.h>
#include "tlv320adcx120_page0.h"

void w(uint16_t devaddr, uint16_t memaddr, uint8_t data2);
void init_adc();
void init_adc_nointerupt();
void end_adc();

#endif /* ADC_H_ */