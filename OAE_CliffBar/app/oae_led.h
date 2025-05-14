/**
 * LED functions for OAE device usage.
 */

#pragma once

#include <main.h>
#include <stm32l4xx_hal.h>

/**
 * Toggle a heartbeat led based on HAL ticks.
 *
 * @param gpio_port (GPIO_TypeDef *) The gpio port of the heartbeat led
 * @param pin (uint16_t) The pin connected to the heartbeat led
 */
void heartbeat_led(GPIO_TypeDef *gpio_port, uint16_t pin);
