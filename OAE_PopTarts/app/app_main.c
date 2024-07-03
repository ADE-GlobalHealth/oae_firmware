/*
 * app_main.c
 *
 *  Created on: Sep 19, 2023
 *      Author: veswaranandam
 */

#include <app_core.h>
#include <main.h>
// #include "TLV320ADC3120.h"
#include <arm_math.h>
#include "dual_dma.h"

#define NS  4096
#define n_data 1000
#define M_PI 3.14159265358979323846
#define LUT_SIZE 128
#define BUFFER_SIZE 4096
#include <stdbool.h>

#include <stm32l4xx_hal_sai.h>
#include <stm32l4xx_hal_dac.h>

extern DAC_HandleTypeDef hdac1;
extern DMA_HandleTypeDef hdma_dac_ch1;
extern TIM_HandleTypeDef htim6;
extern SAI_HandleTypeDef hsai_BlockA2;
extern I2C_HandleTypeDef hi2c3;

// uint32_t Wave_LUT[NS] = {
// 	                2048, 2149, 2250, 2350, 2450, 2549, 2646, 2742, 2837, 2929, 3020, 3108, 3193, 3275, 3355,
// 	                3431, 3504, 3574, 3639, 3701, 3759, 3812, 3861, 3906, 3946, 3982, 4013, 4039, 4060, 4076,
// 	                4087, 4094, 4095, 4091, 4082, 4069, 4050, 4026, 3998, 3965, 3927, 3884, 3837, 3786, 3730,
// 	                3671, 3607, 3539, 3468, 3394, 3316, 3235, 3151, 3064, 2975, 2883, 2790, 2695, 2598, 2500,
// 	                2400, 2300, 2199, 2098, 1997, 1896, 1795, 1695, 1595, 1497, 1400, 1305, 1212, 1120, 1031,
// 	                944, 860, 779, 701, 627, 556, 488, 424, 365, 309, 258, 211, 168, 130, 97,
// 	                69, 45, 26, 13, 4, 0, 1, 8, 19, 35, 56, 82, 113, 149, 189,
// 	                234, 283, 336, 394, 456, 521, 591, 664, 740, 820, 902, 987, 1075, 1166, 1258,
// 	                1353, 1449, 1546, 1645, 1745, 1845, 1946, 2047
// 	        };

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
uint8_t first_time = 1;
// TLV320ADC3120 dev;


uint8_t data1 = 0x01;
uint8_t* pData = &data1;

/**
 * Write to a register in a peripheral over i2c.
 * 
 * @param devaddr The i2c device address to write to.
 * @param memaddr The register address to write to.
 * @param data2 The data to write to the register.
*/
void w(uint16_t devaddr, uint16_t memaddr, uint8_t data2){
	data1 = data2;
	// Device address is 1001110, left shifted by 1 is 9C
	pData = &data1;
	HAL_I2C_Mem_Write(&hi2c3, devaddr, memaddr, 1,pData, 1, HAL_MAX_DELAY);
}

/**
 * Read a register from a peripheral over i2c.
 * 
 * @param devaddr The i2c device address to read from.
 * @param memaddr The register address to read from.
 * @param data A data buffer to store the read data.
*/
void r(uint16_t devaddr, uint16_t memaddr, uint8_t* data){
  HAL_I2C_Mem_Read(&hi2c3, devaddr, memaddr, 1, data, 1, HAL_MAX_DELAY);
}

#include "tlv320adcx120_page0.h"

void init_adc(){
	// Step 1: apply power to the device

	// // Wait for 1 ms.
	// HAL_Delay(10000);

	// // Step 2a: wake up the device

	// // Wake-up the device with an I2C write into P0_R2 using an internal AREG
	// w(0x9C,0x02,SLEEP_CFG_AREG_SELECT_INTERNAL & SLEEP_CFG_SLEEP_ENZ_ACTIVE);

	// w(0x9C,0x01,SW_RESET_RESET);
	// // Step 2b: wait for the device to wake up
	// HAL_Delay(100);
	// Wait for 1 ms.
	HAL_Delay(1000);

	// Step 2a: wake up the device

	// Wake-up the device with an I2C write into P0_R2 using an internal AREG
	w(0x9C,0x02,SLEEP_CFG_AREG_SELECT_INTERNAL | SLEEP_CFG_SLEEP_ENZ_ACTIVE);

	HAL_Delay(10);

	// // Enable input Ch-1 and Ch-2 by an I2C write into P0_R115
	w(0x9C,0x73,0xC0);

	//!Set micbias to be GP2
	 w(0x9C, 0x3B, BIAS_CFG_MBIAS_VAL_GPI2);
	// !Set GPI2 to be MCLK input
	 w(0x9C, 0x2B, GPI_CFG0_GPI2_CFG_MCLK);

	// Step 2c. Overwrite default configuration registers or programmable coefficient values as required

	//!Set Master Clock for PLL and make this slave
	//w(0x9C,0x13,MST_CFG0_MST_SLV_CFG_SLAVE | MST_CFG0_AUTO_CLK_CFG_ENABLED); - only defaults

	//Set I2S sample rate to 8kHz
	// w(0x9C,0x14,MST_CFG1_FS_RATE_7P35_8_KHZ & MST_CFG1_FS_BCLK_RATIO_32); - master mode only

	//! Write prefered format as I2S into P0_R7
	w(0x9C,0x07,ASI_CFG0_FORMAT_I2S | ASI_CFG0_WLEN_32_BITS);

	//! Set clock source to BLK
	w(0x9C,0x16,CLK_SRC_DIS_PLL_SLV_CLK_SRC_BCLK);

	//! Set single ended input for channel 1
	w(0x9C,0x3C,CH1_CFG0_INSRC_SINGLE);

	//! Set single ended input for channel 2
	//w(0x9C,0x41,CH2_CFG0_INSRC_SINGLE);

	//! Set channel summation mode
	//w(0x9C,0x6B,DSP_CFG0_CH_SUM_2CH);

	//! Set GPIO1 to be an interupt output
	w(0x9C,0x21,GPIO_CFG0_GPIO1_CFG_IRQ);

	//! Set interupt to active high
	w(0x9C,0x32,INT_CFG_INT_POL_HIGH);

	//! Set interupt masks to allow clock errors
	w(0x9C,0x33,INT_MASK0_DEFAULT | INT_MASK0_ASI_CLK_ERR_UNMASKED);

	//Step 2d. Enable all desired input channels
	//Enable channel 1 and 2 - enabled by default
	w(0x9C,0x73,IN_CH_EN_CH1_ENABLED | IN_CH_EN_CH2_DISABLED);
	
	//Step 2e. Enable all desired serial audio output channels

	// Enable ASI output Ch-1 by an I2C write into P0_R116
	w(0x9C,0x74,ASI_OUT_CH_EN_CH1_ENABLED);

	//Step 2f. Power up the ADC, MICBIAS and PLL
	// Power-up the ADC, MICBIAS, and PLL by an I2C write into P0_R117
	w(0x9C,0x75,PWR_CFG_ADC_PDZ_ON | PWR_CFG_PLL_PDZ_ON | PWR_CFG_MICBIAS_PDZ_OFF);


}

void init_adc_2(){
	HAL_Delay(1000);
	uint8_t read_data = UINT8_MAX;
	r(0x9C, 0x02, &read_data);
	r(0x9C, 0x13, &read_data);
  // Wake-up the device with an I2C write into P0_R2 using an internal AREG
    w(0x9C, 0x02, 0x81);
  	r(0x9C, 0x02, &read_data);

	 HAL_Delay(10);
	 //Enable input Ch-1 and Ch-2 by an I2C write into P0_R115
	 w(0x9C, 0x73, 0xC0);
	 r(0x9C, 0x73, &read_data);

	 // ASI_CFG0_FORMAT_I2S = (uint8_t) 0x40
	 // ASI_CFG0_WLEN_16_BITS = (uint8_t) 0x00
	 //w(0x9C, 0x07, ASI_CFG0_FORMAT_I2S | ASI_CFG0_WLEN_16_BITS);
	 // Configure output as i2s
	 w(0x9C, 0x07, 0x40);
	 r(0x9C, 0x07, &read_data);

	 //Enable ASI output Ch-1 and Ch-2 slots by an I2C write into P0_R116
	 w(0x9C, 0x74, 0xC0);
	 r(0x9C, 0x74, &read_data);

	 // Power-up the ADC, MICBIAS, and PLL by an I2C write into P0_R117
	 w(0x9C, 0x75, 0xE0);
	 r(0x9C, 0x75, &read_data);

	 r(0x9C, 0x15, &read_data);
	 HAL_Delay(1000);
	 r(0x9C, 0x07, &read_data);
	 r(0x9C, 0x08, &read_data);
	 r(0x9C, 0x09, &read_data);
	 r(0x9C, 0x15, &read_data);
	 r(0x9C, 0x76, &read_data);
	 r(0x9C, 0x77, &read_data);
	 r(0x9C, 0x13, &read_data);

}

void end_adc(){
	// Enter sleep mode by writing to P0_R2
	w(0x9C,0x02,SLEEP_CFG_AREG_SELECT_INTERNAL | SLEEP_CFG_SLEEP_ENZ_SLEEP);
	// Wait at least 6ms
	HAL_Delay(6);
	// Read P0_R119 to check device shutdown and sleep mode status
	uint8_t status = 0;
	HAL_I2C_Mem_Read(&hi2c3, 0x9C, 0x77, 1, &status, 1, HAL_MAX_DELAY);
	if (status != DEV_STS1_MODE_STS_SLEEP) HAL_Delay(1000);
}

// arm_rfft_fast_instance_f32 S_;
// arm_rfft_fast_instance_f32* S = &S_;

int fft_cycles = 10; //must be even to start with

float32_t fft_output[10+1]; //would be length of fft_cycles +1 for inital 

void fft(){
	int* current_buffer = fft_cycles % 2 == 0 ? data_i2s : data_i2s_2;
	int* next_buffer = fft_cycles % 2 == 0 ? data_i2s_2 : data_i2s;

	for (int i = 0; i < n_data; i++){
		current_buffer[i] = (float) current_buffer[i];
	}
	// arm_rfft_fast_init_f32(S, 4096);
	// arm_rfft_fast_f32(S,current_buffer, fft_output, 0);
	// further analysis here & output
	//median of noise bins
	// if good
		// mean of noise
		// mean of dpoae
	//sum noise and dpoae into total 
	//counter of good vs bad -> if test bad
	// keep dpoae total and log it
	// divide noise and dpoae by total
	// subtract noise from dpoae
	// log to find snr


	if (fft_cycles > 0)
	{
		HAL_SAI_Receive_DMA(&hsai_BlockA2,(uint8_t*) next_buffer, BUFFER_SIZE);
		fft_cycles--;
	}
}


void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai){
	uint16_t a = 1;
	// if(first_time) {
	// 	HAL_SAI_Receive_DMA(&hsai_BlockA2,(uint8_t*) data_i2s_2, sizeof(data_i2s_2));
	// 	first_time = 0;
	// }
	// else {
	// 	HAL_SAI_Receive_DMA(&hsai_BlockA2,(uint8_t*) data_i2s, sizeof(data_i2s));
	// 	first_time = 1;
	// }
}


uint32_t blink_time = 0;

void app_setup(){
	// for (int i = 0; i < NS; i++) {
	// 		  //for dual right alignment (page 624 of reference manual)
	// 		  Wave_LUT[i] = (Wave_LUT[i] << 16) | (Wave_LUT[i] >> 2);
	// }
	HAL_DAC_Start_DualDMA(&hdac1, DAC_CHANNEL_12D, (uint32_t*)Wave_LUT, LUT_SIZE, DAC_ALIGN_12B_R);
	HAL_TIM_Base_Start(&htim6);
	init_adc_2();
	// TLV320ADC3120_Initialize(&dev, &hi2c3);
	// uint8_t status = HAL_GPIO_ReadPin(ADC_Interupt_GPIO_Port,ADC_Interupt_Pin);
	HAL_SAI_Receive_DMA(&hsai_BlockA2,(uint8_t*) data_i2s, BUFFER_SIZE);
	
	// status = HAL_GPIO_ReadPin(ADC_Interupt_GPIO_Port,ADC_Interupt_Pin);
	blink_time = HAL_GetTick();
}


uint32_t time = 0;
// bool CheckButtonState(GPIO_TypeDef* port,GPIO_TypeDef* pin, unsigned long time);
uint8_t counter = 0;
bool endflag = false;
void app_loop(){
//	w(0x12,0x02,0x00);
	// // DO not use HAL_Delay -> generates an interrupt that halts DMA channels
    time = HAL_GetTick();
    if ((time - blink_time) > 10000) {
		uint16_t b = 1;
        // HAL_GPIO_TogglePin(LD1_GPIO_Port,LD1_Pin);
        // HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
        // blink_time = time;
		// counter++;
		//w(0x12,0x02,SLEEP_CFG_AREG_SELECT_INTERNAL & SLEEP_CFG_SLEEP_ENZ_ACTIVE);
    }
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
