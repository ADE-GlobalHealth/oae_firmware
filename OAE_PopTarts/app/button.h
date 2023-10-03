#ifndef BUTTON_H_
#define BUTTON_H_

#include <stm32l4xx_hal.h>
#include <stdbool.h>
#include <main.h>

bool CheckButtonState(GPIO_TypeDef* port,GPIO_TypeDef* pin, unsigned long time);
bool debounce(GPIO_TypeDef* port,GPIO_TypeDef* pin);


#endif /* BUTTON_H_ */