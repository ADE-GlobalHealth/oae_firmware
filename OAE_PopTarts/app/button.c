// #include "button.h"

// #define CHANGE_INTERVAL 50
// bool CheckButtonState(GPIO_TypeDef* port,GPIO_TypeDef* pin, unsigned long time)
// {
//   static unsigned long lastDebounce = 0;
//   // Check if its appropriate to test the button state
//   if ((time - lastDebounce) >= CHANGE_INTERVAL)
//   {
//     // Do debounce
//     if (debounce(port,pin))
//     {
//       return true;
//     }
//     // Update the last debounce check time
//     lastDebounce = HAL_GetTick();
//   }
//   return false;
// }

// //Stores history of button states
// uint16_t btnStates = 0;

// bool debounce(GPIO_TypeDef* port,GPIO_TypeDef* pin)
// {
//   /* based off of http://www.ganssle.com/debouncing-pt2.htm
//   - stores the button press history as a sequence of bits
//   - if the sequence matches a rising edge (0000 0000 1111 1111), a valid button press is detected
//   */
//   btnStates = (btnStates << 1) | (!HAL_GPIO_ReadPin(SW1_GPIO_Port,SW1_Pin));
//   return (btnStates == 0x00FF);
// }
