#include "bsp_can.h"
#include "bsp_mspm0g_tim_base.h"
#include "freertos_manager.h"
#include "mcu_config.h"
#include "mcu_device.h"
#include "ti/driverlib/dl_gpio.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

// Test
#include "comm_key.h"

uint64_t delta;
uint64_t time_line;
EF_CommKetOutput_e out;
uint8_t touch;

int main(void) {
  SYSCFG_DL_init();
  EasyFrameDevice_Init();
  FreeRTOS_InitOS();
  FreeRTOS_InitTask();
  FreeRTOS_Run();

  while (1) {
      delta = EasyFrameSysTime_GetTimeline_us()- time_line; 
      time_line= EasyFrameSysTime_GetTimeline_us();
  }
}
