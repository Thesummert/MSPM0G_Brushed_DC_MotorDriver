#include "bsp_can.h"
#include "bsp_mspm0g_tim_base.h"
#include "ti/driverlib/dl_gpio.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "mcu_device.h"

// Test
#include "bsp_time.h"


EF_BSP_TimerPWM_t *pwm;
EF_BSP_CAN_t *can;
int32_t a;
uint8_t data[4] = {0xaa, 0xbb, 0xcc, 0xdd};

int main(void) {
  SYSCFG_DL_init();
  EasyFrameDevice_Init();

  can = EFDevice_Get_CAN();
  while (1) {
      can->TransmitFIFO(can, 0x200, data, 4, false);
      EasyFrameSysTime_Delay(0.02);
  }
}

