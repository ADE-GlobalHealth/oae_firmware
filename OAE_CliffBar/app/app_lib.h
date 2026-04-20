#pragma once

/**
 * Delay locally, but do not delay hardware peripherals, such as use of the DMA.
 */
void delay_non_blocking(uint32_t time);
