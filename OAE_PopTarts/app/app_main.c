/*
 * app_main.c
 *
 *  Created on: Sep 19, 2023
 *      Author: veswaranandam
 */

#include <app_core.h>
#include <main.h>

void app_setup(){
}
uint32_t time = 0;
uint32_t blink_time = 0;
void app_loop(){
    time = HAL_GetTick();
    if ((time - blink_time) > 500) {
        HAL_GPIO_TogglePin(LD1_GPIO_Port,LD1_Pin);
        HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
        blink_time = time;
    }
    
    // HAL_Delay(500);
    // HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
    // HAL_Delay(500);
    // HAL_GPIO_TogglePin(LD1_GPIO_Port,LD1_Pin);
    // HAL_Delay(500);
    // HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
    // HAL_Delay(500);
}
