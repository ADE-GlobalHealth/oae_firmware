#include "main.h"
#include "TLV320ADC3120.h"

void driver_setup(){
	TLV320ADC3120 dev;
  	TLV320ADC3120_Initialize(&dev, &hi2c3);
}

void driver_loop(){
     HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
     HAL_Delay(2000);
     HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
     HAL_Delay(1000);
     HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
     HAL_Delay(2000);
     HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
     HAL_Delay(1000);
     HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
     HAL_Delay(2000);
     HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
     HAL_Delay(1000);
}
