/*
 * TLV320ADC3120.h
 *
 *  Created on: Jul 26, 2023
 *      Author: ADE Global Health
 *  Sets up the registers to be used on the TLV320ADC3120 for the Microphone Signal Chain input to be
 *  read by the ADC
 *
 *  Datasheet can be found at https://www.ti.com/lit/ds/symlink/tlv320adc3120.pdf?ts=1690939260986
 */

#ifndef INC_TLV320ADC3120_H
#define INC_TLV320ADC3120_H

#include "stm32l4xx_hal.h"

/* Initialize I2C Address and frame shift 1 bit to the left */
#define TLV320ADC3120_I2C_ADDR     (0x4C << 1)

/* Define essential registers to be used */
#define TLV320ADC3120_ASI_CONFIG    0x07
#define TLV320ADC3120_MCLK_CONFIG    0x13
#define TLV320ADC3120_SAMPLE_CONFIG    0x14
#define TLV320ADC3120_INPUT    0x73
#define TLV320ADC3120_SAI      0x74
#define TLV320ADC3120_PLL      0x75

typedef struct{
	I2C_HandleTypeDef *i2cHandle;

	float output_signal;
} TLV320ADC3120;

/* Initialize I2C */
uint8_t TLV320ADC3120_Initialize(TLV320ADC3120 *dev, I2C_HandleTypeDef *i2cHandle);

/* Interpret microphone data */
HAL_StatusTypeDef TLV320ADC3120_ReadMicrophone(TLV320ADC3120 *dev);

/* Read selected data from register */
HAL_StatusTypeDef TLV320ADC3120_ReadRegister(TLV320ADC3120 *dev, uint8_t reg, uint8_t *data);

HAL_StatusTypeDef TLV320ADC3120_ReadRegisters(TLV320ADC3120 *dev, uint8_t reg, uint8_t *data, uint8_t length);

/* Write data into a selected register*/
HAL_StatusTypeDef TLV320ADC3120_WriteRegister(TLV320ADC3120 *dev, uint8_t reg, uint8_t *data);


#endif /* INC_TLV320ADC3120_H_ */
