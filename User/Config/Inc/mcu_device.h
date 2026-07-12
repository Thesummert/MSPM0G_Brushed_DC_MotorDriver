#ifndef __MCU_DEVICE_H__
#define __MCU_DEVICE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_time.h"
#include "bsp_can.h"
#include "brushed_motor.h"
#include "at24cxx.h"
#include "comm_led.h"
#include "comm_key.h"
#include "bsp_mspm0g_usart.h"

_Bool EasyFrameDevice_Init();

EF_BSP_TimerPWM_t *EFDevice_Get_PWM();
EF_BSP_TimerQEI_t *EFDevice_Get_QEI();
EF_BSP_CAN_t *EFDevice_Get_CAN();
EF_Usart_Typedef *EFDevice_Get_Uart();
EF_BrushedMotor_t *EFDevice_Get_Motor();
EF_Device_AT24CXX_t *EFDevice_Get_EEPROM();
EF_Device_Comm_LED_t *EFDevice_Get_LED();
EF_Device_CommKey_t *EFDevice_Get_Key();

#ifdef __cplusplus
}
#endif

#endif /* __MCU_DEVICE_H__ */
