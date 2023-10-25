/*
 * app_main.c
 *
 *  Created on: Sep 19, 2023
 *      Author: veswaranandam
 */

#include <app_core.h>
#include <main.h>
// #include "TLV320ADC3120.h"

#include "dual_dma.h"

#define NS  128
#define n_data 1000
#define M_PI 3.14159265358979323846
#include <stdbool.h>

extern DAC_HandleTypeDef hdac1;
extern DMA_HandleTypeDef hdma_dac_ch1;
extern TIM_HandleTypeDef htim6;
extern SAI_HandleTypeDef hsai_BlockA2;
extern I2C_HandleTypeDef hi2c3;

uint32_t Wave_LUT[NS] = {
	                2048, 2149, 2250, 2350, 2450, 2549, 2646, 2742, 2837, 2929, 3020, 3108, 3193, 3275, 3355,
	                3431, 3504, 3574, 3639, 3701, 3759, 3812, 3861, 3906, 3946, 3982, 4013, 4039, 4060, 4076,
	                4087, 4094, 4095, 4091, 4082, 4069, 4050, 4026, 3998, 3965, 3927, 3884, 3837, 3786, 3730,
	                3671, 3607, 3539, 3468, 3394, 3316, 3235, 3151, 3064, 2975, 2883, 2790, 2695, 2598, 2500,
	                2400, 2300, 2199, 2098, 1997, 1896, 1795, 1695, 1595, 1497, 1400, 1305, 1212, 1120, 1031,
	                944, 860, 779, 701, 627, 556, 488, 424, 365, 309, 258, 211, 168, 130, 97,
	                69, 45, 26, 13, 4, 0, 1, 8, 19, 35, 56, 82, 113, 149, 189,
	                234, 283, 336, 394, 456, 521, 591, 664, 740, 820, 902, 987, 1075, 1166, 1258,
	                1353, 1449, 1546, 1645, 1745, 1845, 1946, 2047
	        };


volatile uint8_t data_i2s[128];
// TLV320ADC3120 dev;


uint8_t data1 = 0x01;
uint8_t* pData = &data1;

void w(uint16_t devaddr, uint16_t memaddr, uint8_t data2){
	data1 = data2;
	// Device address is 1001110, left shifted by 1 is 9C
	pData = &data1;
	HAL_I2C_Mem_Write(&hi2c3, devaddr, memaddr, 1,pData, 1, HAL_MAX_DELAY);
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
	HAL_Delay(10000);

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
	w(0x9C,0x13,MST_CFG0_MST_SLV_CFG_SLAVE | MST_CFG0_AUTO_CLK_CFG_ENABLED);

	//Set I2S sample rate to 8kHz
	// w(0x9C,0x14,MST_CFG1_FS_RATE_7P35_8_KHZ & MST_CFG1_FS_BCLK_RATIO_32); - master mode only

	//! Write prefered format as I2S into P0_R7
	w(0x9C,0x07,ASI_CFG0_FORMAT_I2S | ASI_CFG0_WLEN_16_BITS);

	//! Set clock source to BLK
	// w(0x9C,0x16,CLK_SRC_DIS_PLL_SLV_CLK_SRC_BCLK);

	//! Set single ended input for channel 1
	w(0x9C,0x3C,CH1_CFG0_INSRC_SINGLE);

	//! Set single ended input for channel 2
	w(0x9C,0x41,CH2_CFG0_INSRC_SINGLE);

	//! Set channel summation mode
	// w(0x9C,0x6B,DSP_CFG0_CH_SUM_2CH);

	//! Set GPIO1 to be an interupt output
	// w(0x9C,0x21,GPIO_CFG0_GPIO1_CFG_IRQ);

	//! Set interupt to active high
	// w(0x9C,0x32,INT_CFG_INT_POL_HIGH);

	//! Set interupt masks to allow clock errors
	// w(0x9C,0x33,INT_MASK0_DEFAULT | INT_MASK0_ASI_CLK_ERR_UNMASKED);

	//Step 2d. Enable all desired input channels
	//Enable channel 1 and 2 - enabled by default
	w(0x9C,0x73,IN_CH_EN_CH1_ENABLED | IN_CH_EN_CH2_ENABLED);
	
	//Step 2e. Enable all desired serial audio output channels

	// Enable ASI output Ch-1 and Ch-2 slots by an I2C write into P0_R116
	w(0x9C,0x74,ASI_OUT_CH_EN_CH1_ENABLED | ASI_OUT_CH_EN_CH2_ENABLED);

	//Step 2f. Power up the ADC, MICBIAS and PLL
	// Power-up the ADC, MICBIAS, and PLL by an I2C write into P0_R117
	w(0x9C,0x75,PWR_CFG_ADC_PDZ_ON | PWR_CFG_PLL_PDZ_ON | PWR_CFG_MICBIAS_PDZ_OFF);


}

void init_adc_2(){
	HAL_Delay(1000);
	w(0x9C,0x02, 0x81);
	HAL_Delay(10);
	//Enable input Ch-1 and Ch-2 by an I2C write into P0_R115
	w(0x9C,0x73,0xC0);

	w(0x9C,0x07,ASI_CFG0_FORMAT_I2S & ASI_CFG0_WLEN_16_BITS);
	//Enable ASI output Ch-1 and Ch-2 slots by an I2C write into P0_R116
	w(0x9C,0x74,0xC0);

	// Power-up the ADC, MICBIAS, and PLL by an I2C write into P0_R117
	w(0x9C,0x75,0xE0);
}

void end_adc(){
	// Enter sleep mode by writing to P0_R2
	w(0x9C,0x02,SLEEP_CFG_AREG_SELECT_INTERNAL & SLEEP_CFG_SLEEP_ENZ_SLEEP);
	// Wait at least 6ms
	HAL_Delay(6);
	// Read P0_R119 to check device shutdown and sleep mode status
	uint8_t status = 0;
	HAL_I2C_Mem_Read(&hi2c3, 0x9C, 0x77, 1, &status, 1, HAL_MAX_DELAY);
	if (status != DEV_STS1_MODE_STS_SLEEP) HAL_Delay(1000);
}




void app_setup(){
	for (int i = 0; i < NS; i++) {
			  //for dual right alignment (page 624 of reference manual)
			  Wave_LUT[i] = (Wave_LUT[i] << 16) | (Wave_LUT[i] >> 2);
	}
	HAL_DAC_Start_DualDMA(&hdac1, DAC_CHANNEL_12D, (uint32_t*)Wave_LUT, 128, DAC_ALIGN_12B_R);
	HAL_TIM_Base_Start(&htim6);
	init_adc();
	// TLV320ADC3120_Initialize(&dev, &hi2c3);
	uint8_t status = HAL_GPIO_ReadPin(ADC_Interupt_GPIO_Port,ADC_Interupt_Pin);
	HAL_SAI_Receive_DMA(&hsai_BlockA2,(uint8_t*) data_i2s, sizeof(data_i2s));
	status = HAL_GPIO_ReadPin(ADC_Interupt_GPIO_Port,ADC_Interupt_Pin);
}


uint32_t time = 0;
uint32_t blink_time = 0;
// bool CheckButtonState(GPIO_TypeDef* port,GPIO_TypeDef* pin, unsigned long time);
uint8_t counter = 0;
bool endflag = false;
void app_loop(){
//	w(0x12,0x02,0x00);
	// DO not use HAL_Delay -> generates an interrupt that halts DMA channels
    time = HAL_GetTick();
    if ((time - blink_time) > 500) {
        HAL_GPIO_TogglePin(LD1_GPIO_Port,LD1_Pin);
        HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
        blink_time = time;
		counter++;
		//w(0x12,0x02,SLEEP_CFG_AREG_SELECT_INTERNAL & SLEEP_CFG_SLEEP_ENZ_ACTIVE);
    }
	if (counter == 200 && endflag == false){
		// end_adc();
		endflag = true;
	}
    // if (CheckButtonState(SW1_GPIO_Port,SW1_Pin,time)){
    // 	HAL_GPIO_TogglePin(LD3_GPIO_Port,LD3_Pin);
    // }
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
