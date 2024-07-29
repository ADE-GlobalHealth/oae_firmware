/*
 * app_main.c
 *
 *  Created on: Sep 19, 2023
 *      Author: veswaranandam
 */

#include <app_core.h>
#include <main.h>
#include <arm_math.h>
#include "dual_dma.h"
#include "tlv320adcx120_page0.h"
#include <stm32l4xx_hal_sai.h>
#include <stm32l4xx_hal_dac.h>
#include <stdbool.h>

#define NS  4096
#define n_data 1000
#define M_PI 3.14159265358979323846
#define LUT_SIZE 128
#define BUFFER_SIZE 4096
#define ADC_ADDRESS ((uint8_t) 0x9C)

extern DAC_HandleTypeDef hdac1;
extern DMA_HandleTypeDef hdma_dac_ch1;
extern TIM_HandleTypeDef htim6;
extern SAI_HandleTypeDef hsai_BlockA2;
extern I2C_HandleTypeDef hi2c3;

uint32_t Wave_LUT[NS] = {
	0x07d007d0, 0x09b60a15, 0x0b7f0c27, 0x0d0f0dda, 0x0e4f0f08, 0x0f2b0f96, 0x0f960f7a, 0x0f8a0eb4, 0x0f080d56, 0x0e160b7f, 0x0cc50956, 0x0b27070c, 0x095604d3, 0x076e02db, 0x058b0151,
	0x03cc0056, 0x024a0000, 0x011d0056, 0x00560151, 0x000202db, 0x002604d3, 0x00c0070c, 0x01c60956, 0x03290b7f, 0x04d30d56, 0x06ab0eb4, 0x08940f7a, 0x0a720f96, 0x0c270f08, 0x0d9a0dda,
	0x0eb40c27, 0x0f640a15, 0x0fa007d0, 0x0f64058b, 0x0eb40379, 0x0d9a01c6, 0x0c270098, 0x0a72000a, 0x08940026, 0x06ab00ec, 0x04d3024a, 0x03290421, 0x01c6064a, 0x00c00894, 0x00260acd,
	0x00020cc5, 0x00560e4f, 0x011d0f4a, 0x024a0fa0, 0x03cc0f4a, 0x058b0e4f, 0x076e0cc5, 0x09560acd, 0x0b270894, 0x0cc5064a, 0x0e160421, 0x0f08024a, 0x0f8a00ec, 0x0f960026, 0x0f2b000a,
	0x0e4f0098, 0x0d0f01c6, 0x0b7f0379, 0x09b6058b, 0x07d007d0, 0x05ea0a15, 0x04210c27, 0x02910dda, 0x01510f08, 0x00750f96, 0x000a0f7a, 0x00160eb4, 0x00980d56, 0x018a0b7f, 0x02db0956,
	0x0479070c, 0x064a04d3, 0x083202db, 0x0a150151, 0x0bd40056, 0x0d560000, 0x0e830056, 0x0f4a0151, 0x0f9e02db, 0x0f7a04d3, 0x0ee0070c, 0x0dda0956, 0x0c770b7f, 0x0acd0d56, 0x08f50eb4,
	0x070c0f7a, 0x052e0f96, 0x03790f08, 0x02060dda, 0x00ec0c27, 0x003c0a15, 0x000007d0, 0x003c058b, 0x00ec0379, 0x020601c6, 0x03790098, 0x052e000a, 0x070c0026, 0x08f500ec, 0x0acd024a,
	0x0c770421, 0x0dda064a, 0x0ee00894, 0x0f7a0acd, 0x0f9e0cc5, 0x0f4a0e4f, 0x0e830f4a, 0x0d560fa0, 0x0bd40f4a, 0x0a150e4f, 0x08320cc5, 0x064a0acd, 0x04790894, 0x02db064a, 0x018a0421,
	0x0098024a, 0x001600ec, 0x000a0026, 0x0075000a, 0x01510098, 0x029101c6, 0x04210379, 0x05ea058b,
};

volatile uint16_t data_i2s[BUFFER_SIZE];
volatile uint16_t data_i2s_2[BUFFER_SIZE];

uint8_t data1 = 0x01;
uint8_t* pData = &data1;

/**
 * Write to a register in a peripheral over I2C.
 * 
 * @param devaddr The I2C device address to write to.
 * @param memaddr The register address to write to.
 * @param data2 The data to write to the register.
*/
void w(uint16_t devaddr, uint16_t memaddr, uint8_t data2){
	data1 = data2;
	pData = &data1;
	HAL_I2C_Mem_Write(&hi2c3, devaddr, memaddr, 1, pData, 1, HAL_MAX_DELAY);
}

/**
 * Read a register from a peripheral over I2C.
 * 
 * @param devaddr The I2C device address to read from.
 * @param memaddr The register address to read from.
 * @param data A data buffer to store the read data.
*/
void r(uint16_t devaddr, uint16_t memaddr, uint8_t* data){
	HAL_I2C_Mem_Read(&hi2c3, devaddr, memaddr, 1, data, 1, HAL_MAX_DELAY);
}

/**
 * Startup ADC by writing to specific registers on the device. 
 * More info can be found about the specific registers in the ADC datasheet: 
 * https://www.ti.com/lit/ds/symlink/tlv320adc3120.pdf
 * Example config script is on pg 105, data on registers is on pg 60 
 * 
 * Can sniff I2C messages using Analog Discovery 2.
*/
void init_adc() {
	// Wait 1 ms
	HAL_Delay(1000);

	uint8_t read_data = UINT8_MAX;
	r(ADC_ADDRESS, 0x02, &read_data);
	r(ADC_ADDRESS, 0x13, &read_data);
  	
	// Wake-up the device with an I2C write into P0_R2 using an internal AREG
    w(ADC_ADDRESS, 0x02, 0x81);
  	r(ADC_ADDRESS, 0x02, &read_data);

	HAL_Delay(10);

	// Enable input Ch-1 and Ch-2 by an I2C write into P0_R115
	w(ADC_ADDRESS, 0x73, 0xC0);
	r(ADC_ADDRESS, 0x73, &read_data);

	// Configure output as I2S
	w(ADC_ADDRESS, 0x07, 0x40);
	r(ADC_ADDRESS, 0x07, &read_data);

	// Enable ASI output Ch-1 and Ch-2 slots by an I2C write into P0_R116
	w(ADC_ADDRESS, 0x74, 0xC0);
	r(ADC_ADDRESS, 0x74, &read_data);

	// Power-up the ADC, MICBIAS, and PLL by an I2C write into P0_R117
	w(ADC_ADDRESS, 0x75, 0xE0);
	r(ADC_ADDRESS, 0x75, &read_data);
}

/**
 * The function below is currently not being used; it potentially might come back
 *  if we find we need to stop the ADC at some point in the runtime
*/ 
void end_adc(){
	// Enter sleep mode by writing to P0_R2
	w(ADC_ADDRESS, 0x02, SLEEP_CFG_AREG_SELECT_INTERNAL | SLEEP_CFG_SLEEP_ENZ_SLEEP);
	// Wait at least 6ms
	HAL_Delay(6);
	// Read P0_R119 to check device shutdown and sleep mode status
	uint8_t status = 0;
	HAL_I2C_Mem_Read(&hi2c3, ADC_ADDRESS, 0x77, 1, &status, 1, HAL_MAX_DELAY);
	if (status != DEV_STS1_MODE_STS_SLEEP) HAL_Delay(1000);
}


/**
 * Runs setup and configuration functions once at the beginning
 * of the runtime.
*/
void app_setup() {
	// Start DMA for the DACs
	HAL_DAC_Start_DualDMA(&hdac1, DAC_CHANNEL_12D, (uint32_t*)Wave_LUT, LUT_SIZE, DAC_ALIGN_12B_R);
	// Start Timer for the DACs
	HAL_TIM_Base_Start(&htim6);

	// Initialize ADC through I2C
	init_adc();

	// Start DMA channel for receiving data from mic
	HAL_SAI_Receive_DMA(&hsai_BlockA2,(uint8_t*) data_i2s, BUFFER_SIZE);
}

uint32_t time = 0;
// bool CheckButtonState(GPIO_TypeDef* port,GPIO_TypeDef* pin, unsigned long time);
uint8_t counter = 0;
bool endflag = false;

void app_loop(){
	// DO not use HAL_Delay -> generates an interrupt that halts DMA channels

	// The code below isn't currently being used but might be useful to reference
	// in the future

    // time = HAL_GetTick();
	// HAL_GPIO_TogglePin(LD1_GPIO_Port,LD1_Pin);
	// HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
	// blink_time = time;
	// counter++;
	//w(0x12,0x02,SLEEP_CFG_AREG_SELECT_INTERNAL & SLEEP_CFG_SLEEP_ENZ_ACTIVE);

	// if (counter == 200 && endflag == false){
	// 	// end_adc();
	// 	endflag = true;
	// }
    // // if (CheckButtonState(SW1_GPIO_Port,SW1_Pin,time)){
    // // 	HAL_GPIO_TogglePin(LD3_GPIO_Port,LD3_Pin);
    // // }
	// HAL_Delay(1000);
	// HAL_GPIO_TogglePin(LD3_GPIO_Port,LD3_Pin);
}


// #define BUFFER_SIZE 128
// int16_t adcData[BUFFER_SIZE];
// int16_t dacData[BUFFER_SIZE];
// static volatile int16_t *inputBufferPtr;
// static volatile int16_t *outputBufferPtr = &dacData;

// uint8_t dataReadyFlag;

// void HAL_I2S_DR_Half_Callback(SAI_HandleTypeDef *hi2s){
// 	inputBufferPtr = &adcData[0];
// 	outputBufferPtr = &dacData[0];

// 	dataReadyFlag = 1;
// }

// void HAL_I2S_DR_Full_Callback(SAI_HandleTypeDef *hi2s){
// 	inputBufferPtr = &adcData[BUFFER_SIZE/2];
// 	outputBufferPtr = &dacData[BUFFER_SIZE/2];
// 	dataReadyFlag = 1;
// }

// #define INT16_TO_FLOAT 1.0f/32768.0f
// #define FLOAT_TO_INT16 32768.0f

// void processData(){
// 	static float leftIn, leftOut;
// 	static float rightIn, rightOut;
// 	for (uint8_t n = 0; n < (BUFFER_SIZE/2) - 1 ; n+= 2){
// 		leftIn = INT16_TO_FLOAT * inputBufferPtr[n];
// 		if (leftIn > 1.0f){
// 			leftIn -= 2.0f;
// 		}

// 		leftOut = leftIn;
// 		outputBufferPtr[n] = (int16_t) (FLOAT_TO_INT16 * leftOut);
// 		rightIn = INT16_TO_FLOAT * inputBufferPtr[n + 1];
// 		if (rightIn > 1.0f){
// 			rightIn -= 2.0f;
// 		}
// 		outputBufferPtr[n + 1] = (int16_t) (FLOAT_TO_INT16 * rightOut);
// 	}
// 	dataReadyFlag = 0;
// }
