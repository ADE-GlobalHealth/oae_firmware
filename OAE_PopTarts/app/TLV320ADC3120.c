/*
 * TLV320ADC3120.c
 *
 *  Created on: Aug 1, 2023
 *      Author: ADE Global Health
 *
 *      Initializes the ADC Registers to output voltage readings
 */
#include "TLV320ADC3120.h"

HAL_StatusTypeDef TLV320ADC3120_Initialize(TLV320ADC3120 *dev, I2C_HandleTypeDef *i2cHandle){
	dev->i2cHandle = i2cHandle;
	dev->output_signal = 0.0f;

	uint8_t errNum = 0;
	HAL_StatusTypeDef status;
	HAL_StatusTypeDef status11;
	uint8_t reg_data_1;
	uint8_t reg_data_2;
	HAL_StatusTypeDef status12;
	uint8_t regData;

	uint8_t writeBuf;
	writeBuf = 0x01;
	status = TLV320ADC3120_WriteRegister(dev, 0x01, writeBuf);
	errNum += (status != HAL_OK);
	HAL_Delay(20);

	status11 = TLV320ADC3120_ReadRegister(dev, 0x02, &reg_data_1);
	writeBuf = 0x05;
	status = TLV320ADC3120_WriteRegister(dev, 0x02, writeBuf);
	errNum += (status != HAL_OK);

	status12 = TLV320ADC3120_ReadRegister(dev, 0x02, &reg_data_2);
    uint8_t temp_reg = 0x02;
	status11 = TLV320ADC3120_ReadRegister(dev, TLV320ADC3120_INPUT, &temp_reg);
	writeBuf = 0xF0;
	status = TLV320ADC3120_WriteRegister(dev, TLV320ADC3120_INPUT, writeBuf);
	errNum += (status != HAL_OK);
	writeBuf = 0xF0;
	status = TLV320ADC3120_WriteRegister(dev, TLV320ADC3120_SAI, writeBuf);
	errNum += (status != HAL_OK);
	writeBuf = 0xE0;
	status = TLV320ADC3120_WriteRegister(dev, TLV320ADC3120_PLL, writeBuf);
	errNum += (status != HAL_OK);
	HAL_Delay(10);
	
	writeBuf = 0x60;
	status = TLV320ADC3120_WriteRegister(dev, TLV320ADC3120_ASI_CONFIG, writeBuf);
	errNum += (status != HAL_OK);
	writeBuf = 0x80;
	status = TLV320ADC3120_WriteRegister(dev, TLV320ADC3120_MCLK_CONFIG, writeBuf);
	errNum += (status != HAL_OK);
	writeBuf = 0x48;
	status = TLV320ADC3120_WriteRegister(dev, TLV320ADC3120_SAMPLE_CONFIG, writeBuf);
	errNum += (status != HAL_OK);

	return status;
};

HAL_StatusTypeDef TLV320ADC3120_ReadRegister(TLV320ADC3120 *dev, uint8_t reg, uint8_t *data){
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(dev->i2cHandle, TLV320ADC3120_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
	return status;
};
HAL_StatusTypeDef TLV320ADC3120_ReadRegisters(TLV320ADC3120 *dev, uint8_t reg, uint8_t *data, uint8_t length){
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(dev->i2cHandle, TLV320ADC3120_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
	return status;
};

HAL_StatusTypeDef TLV320ADC3120_WriteRegister(TLV320ADC3120 *dev, uint8_t reg, uint8_t *data){
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(dev->i2cHandle, TLV320ADC3120_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
	return status;
};
