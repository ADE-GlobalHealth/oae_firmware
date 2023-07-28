#include "main.h"
#include "arm_math.h"

//Runs once after all peripherals are initialized
void app_setup(){
#define BUFFER_SIZE 128
int16_t adcData[BUFFER_SIZE];
int16_t dacData[BUFFER_SIZE];
static volatile int16_t *inputBufferPtr;
static volatile int16_t *outputBufferPtr = &dacData[0];

uint_8 dataReadyFlag

void HAL_I2S_DR_Half_Callback(I2S_HandleTypeDef *hi2s){
	inputBufferPtr = &adcData[0];
	outputBufferPtr = &dacData[0];

	dataReadyFlag = 1;
}

void HAL_I2S_DR_Full_Callback(I2S_HandleTypeDef *hi2s){
	inputBufferPtr = &adcData[BUFFER_SIZE/2];
	outputBufferPtr = &dacData[BUFFER_SIZE/2];

	dataReadyFlag = 1;
}

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
}

//Runs repeatedly after app_setup() has finished
void app_loop() {
	if (dataReadyFlag){
		processData()
	}

}

