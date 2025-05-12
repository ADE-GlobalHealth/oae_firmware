/*
 * User-defined application main functions for OAE device operation.
 *
 * Authors: Venkadesh Eswaranandam, Luke Nonas-Hunter, Drew Pang
 */

#pragma once

/**
 * OAE device application setup to run after all CubeMX auto-generated
 * initialization code but before the application loop.
 */
void app_setup(void);

/*
 * OAE device application main loop to run continuously after all system and
 * OAE device specific initialization.
 *
 * Do not use HAL_Delay in app_loop, because it generates an interrupt that halts
 * DMA channels
 */
void app_loop(void);
