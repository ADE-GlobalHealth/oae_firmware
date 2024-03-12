/****
 * adc.c
 * 
 * 
*/
#include "adc.h"
// TLV320ADC3120 dev;
uint8_t data1 = 0x01;
uint8_t *pData = &data1;

void w(uint16_t devaddr, uint16_t memaddr, uint8_t data2)
{
	data1 = data2;
	// Device address is 1001110, left shifted by 1 is 9C
	pData = &data1;
	HAL_I2C_Mem_Write(&hi2c3, devaddr, memaddr, 1, pData, 1, HAL_MAX_DELAY);
}

void init_adc()
{
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
	w(0x9C, 0x02, SLEEP_CFG_AREG_SELECT_INTERNAL | SLEEP_CFG_SLEEP_ENZ_ACTIVE);

	HAL_Delay(10);

	// // Enable input Ch-1 and Ch-2 by an I2C write into P0_R115
	w(0x9C, 0x73, 0xC0);

	//! Set micbias to be GP2
	w(0x9C, 0x3B, BIAS_CFG_MBIAS_VAL_GPI2);
	// !Set GPI2 to be MCLK input
	w(0x9C, 0x2B, GPI_CFG0_GPI2_CFG_MCLK);

	// Step 2c. Overwrite default configuration registers or programmable coefficient values as required

	//! Set Master Clock for PLL and make this slave
	// w(0x9C,0x13,MST_CFG0_MST_SLV_CFG_SLAVE | MST_CFG0_AUTO_CLK_CFG_ENABLED); - only defaults

	// Set I2S sample rate to 8kHz
	//  w(0x9C,0x14,MST_CFG1_FS_RATE_7P35_8_KHZ & MST_CFG1_FS_BCLK_RATIO_32); - master mode only

	//! Write prefered format as I2S into P0_R7
	w(0x9C, 0x07, ASI_CFG0_FORMAT_I2S | ASI_CFG0_WLEN_32_BITS);

	//! Set clock source to BLK
	w(0x9C, 0x16, CLK_SRC_DIS_PLL_SLV_CLK_SRC_BCLK);

	//! Set single ended input for channel 1
	w(0x9C, 0x3C, CH1_CFG0_INSRC_SINGLE);

	//! Set single ended input for channel 2
	// w(0x9C,0x41,CH2_CFG0_INSRC_SINGLE);

	//! Set channel summation mode
	// w(0x9C,0x6B,DSP_CFG0_CH_SUM_2CH);

	//! Set GPIO1 to be an interupt output
	w(0x9C, 0x21, GPIO_CFG0_GPIO1_CFG_IRQ);

	//! Set interupt to active high
	w(0x9C, 0x32, INT_CFG_INT_POL_HIGH);

	//! Set interupt masks to allow clock errors
	w(0x9C, 0x33, INT_MASK0_DEFAULT | INT_MASK0_ASI_CLK_ERR_UNMASKED);

	// Step 2d. Enable all desired input channels
	// Enable channel 1 and 2 - enabled by default
	w(0x9C, 0x73, IN_CH_EN_CH1_ENABLED | IN_CH_EN_CH2_DISABLED);

	// Step 2e. Enable all desired serial audio output channels

	// Enable ASI output Ch-1 by an I2C write into P0_R116
	w(0x9C, 0x74, ASI_OUT_CH_EN_CH1_ENABLED);

	// Step 2f. Power up the ADC, MICBIAS and PLL
	//  Power-up the ADC, MICBIAS, and PLL by an I2C write into P0_R117
	w(0x9C, 0x75, PWR_CFG_ADC_PDZ_ON | PWR_CFG_PLL_PDZ_ON | PWR_CFG_MICBIAS_PDZ_OFF);
}

void init_adc_nointerupt()
{
	HAL_Delay(1000);
	// Wake-up the device with an I2C write into P0_R2 using an internal AREG
	w(0x9C, 0x02, SLEEP_CFG_AREG_SELECT_INTERNAL | SLEEP_CFG_SLEEP_ENZ_ACTIVE);
	HAL_Delay(10);
	// Enable input Ch-1 and Ch-2 by an I2C write into P0_R115
	w(0x9C, 0x73, 0xC0);

	w(0x9C, 0x07, ASI_CFG0_FORMAT_I2S | ASI_CFG0_WLEN_16_BITS);
	// Enable ASI output Ch-1 and Ch-2 slots by an I2C write into P0_R116
	w(0x9C, 0x74, 0xC0);

	// Power-up the ADC, MICBIAS, and PLL by an I2C write into P0_R117
	w(0x9C, 0x75, 0xE0);
}

void end_adc()
{
	// Enter sleep mode by writing to P0_R2
	w(0x9C, 0x02, SLEEP_CFG_AREG_SELECT_INTERNAL | SLEEP_CFG_SLEEP_ENZ_SLEEP);
	// Wait at least 6ms
	HAL_Delay(6);
	// Read P0_R119 to check device shutdown and sleep mode status
	uint8_t status = 0;
	HAL_I2C_Mem_Read(&hi2c3, 0x9C, 0x77, 1, &status, 1, HAL_MAX_DELAY);
	if (status != DEV_STS1_MODE_STS_SLEEP)
		HAL_Delay(1000);
}