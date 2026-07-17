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
#include "at24cxx.h"
#include "comm_key.h"

int main(void) {
  SYSCFG_DL_init();
  EasyFrameDevice_Init();
  FreeRTOS_InitOS();
  FreeRTOS_InitTask();
  FreeRTOS_Run();

  while (1) {
  }
}
