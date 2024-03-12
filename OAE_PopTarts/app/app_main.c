/*
 * app_main.c
 *
 *  Created on: Sep 19, 2023
 *      Author: veswaranandam
 */

#include <stdio.h>
#include <app_core.h>
#include <main.h>
// #include "TLV320ADC3120.h"
#include <arm_math.h>
#include "dual_dma.h"
#include "adc.h"
#include "lut.c"
#define NS 128
#define n_data 1000
#define M_PI 3.14159265358979323846
#include <stdbool.h>
#include <dsp/fast_math_functions.h>

#include <stm32l4xx_hal_sai.h>
#include <stm32l4xx_hal_dac.h>

extern DAC_HandleTypeDef hdac1;
extern DMA_HandleTypeDef hdma_dac_ch1;
extern TIM_HandleTypeDef htim6;
extern SAI_HandleTypeDef hsai_BlockA2;
extern I2C_HandleTypeDef hi2c3;

// // frequency calculation - set to 
// pts = 128; // number of points
// targetFS = 2; // KHz - target frequency in kHz
// TriggerFS = targetFS * pts; // KHz - intermediate calc
// Fclk = 80000000; // MHz - timer clock
// PSC = 0; // prescaler value for timer
// ARR = Fclk / (TriggerFS * (PSC+1.0)) - 1; // ARR value for timer
// ARR = 312.49 // ARR value for 2KHz
// 6 cycles max of a singular frequency

#define FFT_LEN (4096)

volatile uint32_t data_i2s[FFT_LEN];
volatile uint32_t data_i2s_2[FFT_LEN];



arm_rfft_fast_instance_f32 S_;
arm_rfft_fast_instance_f32 *S = &S_;

int fft_cycles = 10; // must be even to start with

float32_t fft_output[FFT_LEN]; // would be length of fft_cycles +1 for inital

void fft(float *current_buffer)
{
//	uint32_t *current_buffer = fft_cycles % 2 == 0 ? data_i2s : data_i2s_2;
//	uint32_t *next_buffer = fft_cycles % 2 == 0 ? data_i2s_2 : data_i2s;
//	for (int i = 0; i < FFT_LEN; i++)
//	{
//		current_buffer[i] = (float)current_buffer[i];
//	}
	arm_rfft_fast_init_f32(S, FFT_LEN);
	arm_rfft_fast_f32(S, current_buffer, fft_output, 0);
	// further analysis here & output
	// median of noise bins
	// if good
	// mean of noise
	// mean of dpoae
	// sum noise and dpoae into total
	// counter of good vs bad -> if test bad
	//  keep dpoae total and log it
	//  divide noise and dpoae by total
	//  subtract noise from dpoae
	//  log to find snr

//	if (fft_cycles > 0)
//	{
//		HAL_SAI_Receive_DMA(&hsai_BlockA2, (uint8_t *)next_buffer, sizeof(next_buffer));
//		fft_cycles--;
//	}
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
	// fft();
}
extern Wave_LUT;
uint32_t* LUT_ptr = &Wave_LUT;
void app_setup(){
	// print(ARR);
	// int i = 0;
	// for (float t = 0; t < (2*PI); t+= ((2*PI/(NS-1))) /*linspace points from 0 to 2pi*/) {
	// 		uint32_t f1 = (2048 * (arm_sin_f32(6* t)+1));
	// 		uint32_t f2 = (2048 * (arm_sin_f32(5* t)+1));
	// 		  //for dual right alignment (page 624 of reference manual)
	// 		  Wave_LUT[i] = (f1 << 16) | (f2 >> 2);
	// 		  i++;
	// }
	
	HAL_DAC_Start_DualDMA(&hdac1, DAC_CHANNEL_12D, LUT_ptr, 128, DAC_ALIGN_12B_R);
	HAL_TIM_Base_Start(&htim6);
	init_adc_nointerupt();
	// TLV320ADC3120_Initialize(&dev, &hi2c3);
	// uint8_t status = HAL_GPIO_ReadPin(ADC_Interupt_GPIO_Port,ADC_Interupt_Pin);
	HAL_SAI_Receive_DMA(&hsai_BlockA2,(uint8_t*) data_i2s, sizeof(data_i2s));
	
	// status = HAL_GPIO_ReadPin(ADC_Interupt_GPIO_Port,ADC_Interupt_Pin);
}

uint32_t fft_start = 0;
uint32_t fft_end = 0;
uint32_t time = 0;
uint32_t blink_time = 0;
// bool CheckButtonState(GPIO_TypeDef* port,GPIO_TypeDef* pin, unsigned long time);
uint8_t counter = 0;
bool endflag = false;
void app_loop()
{
	float current_buffer[FFT_LEN];
	uint8_t f = 1;
	for (int i = 0; i < FFT_LEN; i++)
	{
		current_buffer[i] = (float)sin(f*((float)i/FFT_LEN)*2*PI);
	}

	fft_start = HAL_GetTick();
	fft(current_buffer);
	fft_end = HAL_GetTick();
	uint32_t fft_time_ms = fft_end - fft_start;

	//	w(0x12,0x02,0x00);
	// // DO not use HAL_Delay -> generates an interrupt that halts DMA channels
	// time = HAL_GetTick();
	// if ((time - blink_time) > 500) {
	//     HAL_GPIO_TogglePin(LD1_GPIO_Port,LD1_Pin);
	//     HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
	//     blink_time = time;
	// 	counter++;
	// 	//w(0x12,0x02,SLEEP_CFG_AREG_SELECT_INTERNAL & SLEEP_CFG_SLEEP_ENZ_ACTIVE);
	// }
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
