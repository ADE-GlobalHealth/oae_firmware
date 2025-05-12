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
 */
void app_loop(void);
