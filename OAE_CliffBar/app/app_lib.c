// #include "app_lib.h"

// //copy paste hal_delay to remove weak typing
// void HAL_Delay(uint32_t Delay)
// {
//   uint32_t tickstart = HAL_GetTick();
//   uint32_t wait = Delay;

//   /* Add a period to guaranty minimum wait */
//   if (wait < HAL_MAX_DELAY)
//   {
//     wait += (uint32_t)uwTickFreq;
//   }

//   while ((HAL_GetTick() - tickstart) < wait)
//   {
//   }
// }