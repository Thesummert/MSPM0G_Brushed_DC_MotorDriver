#include "ti/driverlib/dl_gpio.h"
#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdlib.h>
#include "mcu_device.h"

// Test
#include "bsp_time.h"


EF_BSP_TimerPWM_t *pwm;
int32_t a;

int main(void) {
  SYSCFG_DL_init();
  EasyFrameDevice_Init();
  pwm = EFDevice_Get_PWM();
  pwm->StartPWM(pwm);

  while (1) {
      pwm->SetPWM(pwm, 0, 500);
      pwm->SetPWM(pwm, 1, 1);
  }
}

