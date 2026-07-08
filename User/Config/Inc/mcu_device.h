#ifndef __MCU_DEVICE_H__
#define __MCU_DEVICE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_time.h"

_Bool EasyFrameDevice_Init();

EF_BSP_TimerPWM_t * EFDevice_Get_PWM();
EF_BSP_TimerQEI_t *  EFDevice_Get_QEI();

#ifdef __cplusplus
}
#endif

#endif /* __MCU_DEVICE_H__ */
