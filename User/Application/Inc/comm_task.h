#ifndef __COMM_TASK_H__
#define __COMM_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "comm_key.h"
#include "comm_led.h"
#include "freertos_manager.h"
#include "motor_can_control.h"
#include "motor_module_manager.h"

typedef struct {
  MotorCan_Slave2Master_t can_transmit;
  EF_Device_Comm_LED_t *status_led;
  EF_Device_CommKey_t *cmd_key;
  MotorManager_t manager;

} CommTask_t;



#ifdef __cplusplus
}
#endif

#endif /* __COMM_TASK_H__ */
