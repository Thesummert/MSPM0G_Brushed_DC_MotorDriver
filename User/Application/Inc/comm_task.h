#ifndef __COMM_TASK_H__
#define __COMM_TASK_H__

#include "bsp_can.h"
#include "bsp_mspm0g_usart.h"
#include "motor_runner.h"
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#include "abstract_queue.h"
#include "comm_key.h"
#include "comm_led.h"
#include "freertos_manager.h"
#include "motor_can_control.h"
#include "motor_module_manager.h"
#include "motor_uart_control.h"

typedef struct {
  EF_BSP_CAN_t *ecan;
  EF_Algorithm_Queue_t uart_queue;
  EF_Usart_Typedef *euart;
  MotorCan_Slave2Master_t can_message;
  EF_Device_Comm_LED_t *status_led;
  EF_Device_CommKey_t *cmd_key;
  MotorManager_t manager;
  MotorUart_Slave2Master_t uart_message;
  MotorModuleFreq_e send_freq;
  BrushedMotorRunner_t *motor;
  uint8_t can_counter;     // can接收计数器
  _Bool comm_mode;         // 通信模式 0为uart 1为can
  uint16_t counter;        // 计数器 用来设定发送频率
  uint16_t reload_counter; // 重置频率计数器Load

} CommTask_t;

void CommTask_Init();
void CommTask_CAN_Decode(const uint8_t *data, uint8_t data_len);
uint16_t CommTask_GetID();

#ifdef __cplusplus
}
#endif

#endif /* __COMM_TASK_H__ */
