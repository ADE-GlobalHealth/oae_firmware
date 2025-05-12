#include "oae_led.h"

void heartbeat_led(GPIO_TypeDef *gpio_port, uint16_t pin) {
	static uint32_t time = 0;
	static uint32_t blink_time = 0;

	time = HAL_GetTick();
	if (time - blink_time > 1000) {
		HAL_GPIO_TogglePin(gpio_port, pin);
		blink_time = time;
	}
}
