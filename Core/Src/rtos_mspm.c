#include "rtos_mspm.h"
#include "ti/devices/msp/m0p/mspm0g350x.h"
#ifdef __cplusplus
extern "C" {
#endif


extern void xPortSysTickHandler(void);

// void SysTick_Handler(void) {
//   SysTick->CTRL;
//
//   if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
//     xPortSysTickHandler();
//   }
// }


#ifdef __cplusplus
}
#endif
