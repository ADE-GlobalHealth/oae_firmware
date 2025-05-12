#include <app_main.h>
#include <main.h>
#include <stdbool.h>

#include "oae_serial.h"
#include "usbd_cdc_if.h"
#include "oae_adc.h"
#include "oae_dac.h"

/**
 * Runs setup and configuration functions once at the beginning
 * of the runtime.
 */
void app_setup() {
	// Initialize oae serial protocol
	oae_serial_init();

	// Initialize ADC through I2C
	init_adc();

	// Initialize DAC timers
	init_dac();
}

uint32_t time = 0;
uint32_t blink_time = 0;
// bool CheckButtonState(GPIO_TypeDef* port,GPIO_TypeDef* pin, unsigned long time);
uint8_t counter = 0;
bool endflag = false;

void app_loop() {
	static uint8_t RxBuffer[APP_RX_DATA_SIZE];
	static uint32_t RxBufferLen;

	// DO not use HAL_Delay -> generates an interrupt that halts DMA channels

	// The code below isn't currently being used but might be useful to reference
	// in the future

	time = HAL_GetTick();
	if (time - blink_time > 1000) {
		//HAL_GPIO_TogglePin(LD1_GPIO_Port,LD1_Pin);
		//HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
		blink_time = time;
	}

	// Check for incoming USB serial packets:
	while (RX_USB_CDC_Data(RxBuffer, &RxBufferLen) == 1) {
		for (int i = 0; i < RxBufferLen; i++) {
			if (oae_serial_receive(RxBuffer[i])) {
				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
				oae_process_rx_packet();
				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
			}
		}
	}

//	 counter++;
//	 w(0x12,0x02,SLEEP_CFG_AREG_SELECT_INTERNAL & SLEEP_CFG_SLEEP_ENZ_ACTIVE);
//
//	 if (counter == 200 && endflag == false){
//	 	// end_adc();
//	 	endflag = true;
//	 }
//     // if (CheckButtonState(SW1_GPIO_Port,SW1_Pin,time)){
//     // 	HAL_GPIO_TogglePin(LD3_GPIO_Port,LD3_Pin);
//     // }
//	 HAL_Delay(1000);
//	 HAL_GPIO_TogglePin(LD3_GPIO_Port,LD3_Pin);
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
