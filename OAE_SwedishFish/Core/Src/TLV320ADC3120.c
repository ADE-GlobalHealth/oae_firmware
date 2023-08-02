/*
 * TLV320ADC3120.c
 *
 *  Created on: Aug 1, 2023
 *      Author: ADE Global HEalth
 */
#include "TLV320ADC3120.h"

uint8_t TLV320ADC3120_Initialize(TLV320ADC3120 *dev, I2C_HandleTypeDef *i2cHandle){
	dev->i2cHandle = i2cHandle;
	dev->output_signal = 0.0f;

	uint8_t errNum = 0;
	HAL_StatusTypeDef status;
	uint8_t regData;

	status = TLV320ADC3120_ReadRegister(dev, TLV320ADC3120_INPUT, &regData);

	errNum += (status != HAL_OK);
};
