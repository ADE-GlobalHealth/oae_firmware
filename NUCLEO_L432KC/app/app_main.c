/*
 * app.c
 *
 *  Created on: Oct 24, 2023
 *      Author: veswaranandam
 *
 */

#include <app_core.h>
#include <main.h>
#include <stm32l4xx_hal_sai.h>
#include <stm32l4xx_hal_dac.h>


#define NS  4096


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

extern I2C_HandleTypeDef hi2c3;
extern DAC_HandleTypeDef hdac1;
extern DMA_HandleTypeDef hdma_dac_ch1;
extern TIM_HandleTypeDef htim2;
extern SAI_HandleTypeDef hsai_BlockA1;

volatile uint32_t data_i2s[4096];
volatile uint32_t data_i2s_2[4096];

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


uint32_t time = 0;
uint32_t max_time = 1000;
uint32_t counter = 0;
uint32_t blink_time = 0;

void app_setup(){
	for (int i = 0; i < NS; i++) {
				  //for dual right alignment (page 624 of reference manual)
		Wave_LUT[i] = (Wave_LUT[i] << 16) | (Wave_LUT[i] >> 2);
	}
	HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)Wave_LUT, 128, DAC_ALIGN_12B_R);
	HAL_TIM_Base_Start(&htim2);
	init_adc_2();
		// TLV320ADC3120_Initialize(&dev, &hi2c3);
		// uint8_t status = HAL_GPIO_ReadPin(ADC_Interupt_GPIO_Port,ADC_Interupt_Pin);
	HAL_SAI_Receive_DMA(&hsai_BlockA1,(uint8_t*) data_i2s, sizeof(data_i2s));
	
	time = HAL_GetTick();
	while(time < max_time) {
		time = HAL_GetTick();
	}
	
	int a = 1;
		// status = HAL_GPIO_ReadPin(ADC_Interupt_GPIO_Port,ADC_Interupt_Pin);
}

void app_loop(){
	// HAL_Delay(100);
	// HAL_GPIO_TogglePin(LD3_GPIO_Port,LD3_Pin);

	// time = HAL_GetTick();
    // if ((time - blink_time) > 500) {
    //     HAL_GPIO_TogglePin(LD3_GPIO_Port,LD3_Pin);
    //     blink_time = time;
	// 	counter++;
	// 	//w(0x12,0x02,SLEEP_CFG_AREG_SELECT_INTERNAL & SLEEP_CFG_SLEEP_ENZ_ACTIVE);
    // }

	HAL_Delay(100);
    HAL_GPIO_TogglePin(LD3_GPIO_Port,LD3_Pin);
    w(0x9C,0x74,0x00 & 0x01);

//	w(0x9C,0x74,0x00 & 0x01);
}

