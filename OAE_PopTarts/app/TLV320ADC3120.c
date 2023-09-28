/*
 * TLV320ADC3120.c
 *
 *  Created on: Aug 1, 2023
 *      Author: ADE Global Health
 *
 *      Initializes the ADC Registers to output voltage readings
 */
//#include "TLV320ADC3120.h"
//
//HAL_StatusTypeDef TLV320ADC3120_Initialize(TLV320ADC3120 *dev, I2C_HandleTypeDef *i2cHandle){
//	dev->i2cHandle = i2cHandle;
//	dev->output_signal = 0.0f;
//
//	uint8_t errNum = 0;
//	HAL_StatusTypeDef status;
//	HAL_StatusTypeDef status11;
//	uint8_t reg_data_1;
//	uint8_t reg_data_2;
//	HAL_StatusTypeDef status12;
//	uint8_t regData;
//
//	status = TLV320ADC3120_WriteRegister(dev, 0x01, 0x01);
//	errNum += (status != HAL_OK);
//	HAL_Delay(20);
//
//	status11 = TLV320ADC3120_ReadRegister(dev, 0x02, reg_data_1);
//
//	status = TLV320ADC3120_WriteRegister(dev, 0x02, 0x05);
//	errNum += (status != HAL_OK);
//
//	status12 = TLV320ADC3120_ReadRegister(dev, 0x02, reg_data_2);
//
//	status11 = TLV320ADC3120_ReadRegister(dev, TLV320ADC3120_INPUT, 0x02);
//
//	status = TLV320ADC3120_WriteRegister(dev, TLV320ADC3120_INPUT, 0xF0);
//	errNum += (status != HAL_OK);
//
//	status = TLV320ADC3120_WriteRegister(dev, TLV320ADC3120_SAI, 0xF0);
//	errNum += (status != HAL_OK);
//
//	status = TLV320ADC3120_WriteRegister(dev, TLV320ADC3120_PLL, 0xE0);
//	errNum += (status != HAL_OK);
//	HAL_Delay(10);
//
//	status = TLV320ADC3120_WriteRegister(dev, TLV320ADC3120_ASI_CONFIG, 0x60);
//	errNum += (status != HAL_OK);
//
//	status = TLV320ADC3120_WriteRegister(dev, TLV320ADC3120_MCLK_CONFIG, 0x80);
//	errNum += (status != HAL_OK);
//
//	status = TLV320ADC3120_WriteRegister(dev, TLV320ADC3120_SAMPLE_CONFIG, 0x48);
//	errNum += (status != HAL_OK);
//
//	return status;
//};
//
//HAL_StatusTypeDef TLV320ADC3120_ReadRegister(TLV320ADC3120 *dev, uint8_t reg, uint8_t *data){
//	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(dev->i2cHandle, TLV320ADC3120_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
//	return status;
//};
//HAL_StatusTypeDef TLV320ADC3120_ReadRegisters(TLV320ADC3120 *dev, uint8_t reg, uint8_t *data, uint8_t length){
//	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(dev->i2cHandle, TLV320ADC3120_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
//	return status;
//};
//
//HAL_StatusTypeDef TLV320ADC3120_WriteRegister(TLV320ADC3120 *dev, uint8_t reg, uint8_t *data){
//	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(dev->i2cHandle, TLV320ADC3120_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
//	return status;
//};
