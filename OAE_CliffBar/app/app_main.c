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
}
