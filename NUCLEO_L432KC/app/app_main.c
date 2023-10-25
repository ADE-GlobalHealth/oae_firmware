/*
 * app.c
 *
 *  Created on: Oct 24, 2023
 *      Author: veswaranandam
 *
 */

#include <app_core.h>
#include <main.h>

extern I2C_HandleTypeDef hi2c3;

uint8_t data1 = 0x12;
uint8_t* pData = &data1;

void w(uint16_t devaddr, uint16_t memaddr, uint8_t data2){
	data1 = data2;
	// Device address is 1001110, left shifted by 1 is 9C
	pData = &data1;
	HAL_I2C_Mem_Write(&hi2c3, devaddr, memaddr, 1,pData, 1, HAL_MAX_DELAY);
}

void app_setup(){
}

void app_loop(){
	HAL_Delay(100);
	HAL_GPIO_TogglePin(LD3_GPIO_Port,LD3_Pin);
	w(0x9C,0x74,0x00 & 0x01);
}

