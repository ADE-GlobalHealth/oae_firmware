#include <app_main.h>
#include <main.h>
#include <stdbool.h>

#include "oae_serial.h"
#include "usbd_cdc_if.h"
#include "oae_adc.h"
#include "oae_dac.h"
#include "oae_led.h"

void app_setup() {
	// Initialize oae serial protocol
	oae_serial_init();

	// Initialize ADC through I2C
	init_adc();

	// Initialize DAC timers
	init_dac();
}

void app_loop() {
	heartbeat_led(LD1_GPIO_Port, LD1_Pin);

	static uint8_t RxBuffer[APP_RX_DATA_SIZE];
	static uint32_t RxBufferLen;

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
