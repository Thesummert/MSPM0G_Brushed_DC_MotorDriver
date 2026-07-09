#include "ti/driverlib/dl_gpio.h"
#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdlib.h>
#include "mcu_device.h"

// Test
#include "bsp_time.h"


EF_BSP_TimerPWM_t *pwm;

int main(void) {
  SYSCFG_DL_init();
  EasyFrameDevice_Init();

  while (1) {
  }
}

