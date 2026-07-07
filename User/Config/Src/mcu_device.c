#include "mcu_device.h"
#include "bsp_mspm0g_tim_base.h"
#include "ti/devices/msp/m0p/mspm0g350x.h"
#include "ti_msp_dl_config.h"
#include <stdlib.h>

#include "bsp_time.h"

static EF_BSP_TimerBase_t motor_control_tim;
static EF_BSP_TimerPWM_t motor_control_tim_pwm;

_Bool EasyFrameDevice_Init() {
  EasyFrameSysTime_Init(4); // 初始化系统定时
  EF_BSP_TimerBase_Init(&motor_control_tim, PWM_0_INST, 80000000, 0xFF, 0xFFFF);
  EF_BSP_TimerPWM_Init(&motor_control_tim_pwm, &motor_control_tim, 2);

  return true;
}
