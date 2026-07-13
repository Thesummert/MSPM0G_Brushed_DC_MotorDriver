#ifndef __MOTOR_UART_CONTROL_H__
#define __MOTOR_UART_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "mcu_config.h"
#include "stdint.h"
#include <stdint.h>

/**
 * @note 电机速度采用16位整数传输 前12位为整数 后4位代表小数
 * @note 对于串口控制 采取只要接收到CAN有效信息三次后切换为CAN控制
 * 默认为串口控制
 * @note 串口不仅可以控制电机 还能修改电机的参数
 * */

/*电机UART状态控制命令*/
typedef enum {
  MOTOR_UART_IDLE = 0b00,
  MOTOR_UART_RUN = 0b01,
  MOTOR_UART_BREAK = 0b11,
} MotorUartStatusCMD_e;

/*电机UART方向控制命令*/
typedef enum {
  MOTOR_UART_FORWARD = 0b0,
  MOTOR_UART_BACKWARD = 0b1,
} MotorUartDirCMD_e;

/*电机串口通信命令码*/
typedef enum {
  MOTOR_CMD_SET = 0x01,   // 设定电机模块参数
  MOTOR_CMD_DRIVE = 0x02, // 电机驱动命令
  MOTOR_CMD_TRANSMIT,     // 传输数据命令
  MOTOR_CMD_FEEDBACK,     // 回报数据命令
  MOTOR_CMD_ACK = 0x10,   // 电机回复报文
} MotorUartCtrlCMD_e;

/*电机控制联合*/
typedef union {
  struct {
    uint8_t status; // 驱动器状态 [7:7]被设计用来表示速度方向
    uint16_t value; // 速度值
  } cmd;
  uint8_t datas[3];
} MotorUartCtrl_u;

typedef struct MotorUart_Slave2Master_t {
  uint8_t buffer[UART_TX_DMA_BUFFER_SIZE]; // 串口DMA传输使用到的缓冲区
  uint16_t send_len;                       // 数据发送长度

  _Bool (*Encode)(struct MotorUart_Slave2Master_t *self, uint16_t master_id,
                  uint16_t slave_id, uint8_t cmd, uint8_t *data,
                  uint16_t data_len);
} MotorUart_Slave2Master_t;

_Bool MotorUartSlave_Init(MotorUart_Slave2Master_t *self);

_Bool MotorUartSlave_Decode(uint8_t *data_input, uint16_t *master_id,
                            uint16_t *slave_id, uint8_t *cmd,
                            uint16_t *data_len, uint8_t *data_out,
                            uint16_t message_len);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_UART_CONTROL_H__ */
