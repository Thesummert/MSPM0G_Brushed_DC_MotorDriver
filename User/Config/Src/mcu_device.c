#include "mcu_device.h"
#include "bsp_mspm0g_tim_base.h"
#include "ti/devices/msp/m0p/mspm0g350x.h"
#include "ti_msp_dl_config.h"
#include <stdlib.h>


static EF_BSP_TimerBase_t motor_control_tim;
static EF_BSP_TimerBase_t motor_encoder_tim;
static EF_BSP_TimerPWM_t motor_control_tim_pwm;
static EF_BSP_TimerQEI_t motor_encoder_tim_qei;

_Bool EasyFrameDevice_Init() {
  EasyFrameSysTime_Init(4); // 初始化系统定时
  EF_BSP_TimerBase_Init(&motor_control_tim, PWM_0_INST, 80000000, 0xFF, 0xFFFF);
  EF_BSP_TimerPWM_Init(&motor_control_tim_pwm, &motor_control_tim, 2);
  EF_BSP_TimerBase_Init(&motor_encoder_tim, QEI_0_INST, 40000000, 0xFF, 0xFFFF);
  EF_BSP_TimerQEI_Init(&motor_encoder_tim_qei, &motor_encoder_tim);

  return true;
}

EF_BSP_TimerPWM_t * EFDevice_Get_PWM()
{
    return &motor_control_tim_pwm;
}

EF_BSP_TimerQEI_t *  EFDevice_Get_QEI()
{
    return &motor_encoder_tim_qei;
}
