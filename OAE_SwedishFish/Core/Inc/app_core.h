/*
 * app_core.h
 *
 * Contains all functions that must be defined by an application.
 *
 *  Created on: Jul 19, 2023
 *      Author: Luke Nonas-Hunter
 */

#ifndef INC_APP_CORE_H_
#define INC_APP_CORE_H_

/* The setup function is run immediately before the main loop but after all of
 * the CubeMX auto-generated initialization code.
 */
void setup(void);

/* The loop function is the first function called in the main loop.
 */
void loop(void);


#endif /* INC_APP_CORE_H_ */
