/*
 * app_main.c
 *
 *  Created on: Sep 19, 2023
 *      Author: veswaranandam
 */

#include <app_core.h>
#include <main.h>
#include "TLV320ADC3120.h"

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
volatile int16_t i2s_sample;
TLV320ADC3120 dev;

void app_setup(){
	for (int i = 0; i < NS; i++) {
			  //for dual right alignment (page 624 of reference manual)
			  Wave_LUT[i] = (Wave_LUT[i] << 16) | (Wave_LUT[i] >> 2);
	}
	HAL_DAC_Start_DualDMA(&hdac1, DAC_CHANNEL_12D, (uint32_t*)Wave_LUT, 128, DAC_ALIGN_12B_R);
	HAL_TIM_Base_Start(&htim6);
	TLV320ADC3120_Initialize(&dev, &hi2c3);
	HAL_SAI_Receive_DMA(&hsai_BlockA2,(uint8_t*) data_i2s, sizeof(data_i2s));
}


uint32_t time = 0;
uint32_t blink_time = 0;
// bool CheckButtonState(GPIO_TypeDef* port,GPIO_TypeDef* pin, unsigned long time);
void app_loop(){
	// DO not use HAL_Delay -> generates an interrupt that halts DMA channels
    time = HAL_GetTick();
    if ((time - blink_time) > 500) {
        HAL_GPIO_TogglePin(LD1_GPIO_Port,LD1_Pin);
        HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
        blink_time = time;
    }
    // if (CheckButtonState(SW1_GPIO_Port,SW1_Pin,time)){
    // 	HAL_GPIO_TogglePin(LD3_GPIO_Port,LD3_Pin);
    // }
}


#define BUFFER_SIZE 128
int16_t adcData[BUFFER_SIZE];
int16_t dacData[BUFFER_SIZE];
static volatile int16_t *inputBufferPtr;
static volatile int16_t *outputBufferPtr = &dacData[0];

uint8_t dataReadyFlag;

void HAL_I2S_DR_Half_Callback(SAI_HandleTypeDef *hi2s){
	inputBufferPtr = &adcData[0];
	outputBufferPtr = &dacData[0];

	dataReadyFlag = 1;
}

void HAL_I2S_DR_Full_Callback(SAI_HandleTypeDef *hi2s){
	inputBufferPtr = &adcData[BUFFER_SIZE/2];
	outputBufferPtr = &dacData[BUFFER_SIZE/2];

	dataReadyFlag = 1;
}

#define INT16_TO_FLOAT 1.0f/32768.0f
#define FLOAT_TO_INT16 32768.0f

void processData(){
	static float leftIn, leftOut;
	static float rightIn, rightOut;
	for (uint8_t n = 0; n < (BUFFER_SIZE/2) - 1 ; n+= 2){
		leftIn = INT16_TO_FLOAT * inputBufferPtr[n];
		if (leftIn > 1.0f){
			leftIn -= 2.0f;
		}

		leftOut = leftIn;
		outputBufferPtr[n] = (int16_t) (FLOAT_TO_INT16 * leftOut);
		rightIn = INT16_TO_FLOAT * inputBufferPtr[n + 1];
		if (rightIn > 1.0f){
			rightIn -= 2.0f;
		}
		outputBufferPtr[n + 1] = (int16_t) (FLOAT_TO_INT16 * rightOut);
	}
	dataReadyFlag = 0;
}