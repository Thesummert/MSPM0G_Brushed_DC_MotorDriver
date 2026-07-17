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
#include "key_manager.h"
#include "motor_can_control.h"
#include "motor_module_manager.h"
#include "motor_uart_control.h"

typedef struct {
  EF_BSP_CAN_t *ecan;                    // CAN实例
  EF_Algorithm_Queue_t uart_queue;       // 串口数据队列
  EF_Usart_Typedef *euart;               // UART实例
  MotorCan_Slave2Master_t can_message;   // CAN消息编码器
  MotorUart_Slave2Master_t uart_message; // 串口消息编码器
  KeyManager_t key_manager;              // 按键管理器
  EF_Device_Comm_LED_t *status_led;      // 状态指示灯
  MotorManager_t manager;                // 管理器
  MotorModuleFreq_e send_freq;           // 数据发送频率
  BrushedMotorRunner_t *motor;           // 电机runner指针
  uint8_t can_counter;                   // can接收计数器
  _Bool comm_mode;                       // 通信模式 0为uart 1为can
  uint16_t counter;                      // 计数器 用来设定发送频率
  uint16_t reload_counter;               // 重置频率计数器Load
  _Bool set_value_mode; // 设定数值模式 将会强制使电机进入到无力状态

  float time_line;
  uint32_t cnt_last;
  _Bool is_online;
  float dt;

} CommTask_t;

void CommTask_Init();
void CommTask_CAN_Decode(const uint8_t *data, uint8_t data_len);
uint16_t CommTask_GetID();
void CommTask_UartRXCallback(void *param);

#ifdef __cplusplus
}
#endif

#endif /* __COMM_TASK_H__ */
