#include "motor_uart_control.h"
#include "crc_ref.h"
#include "mcu_config.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/reent.h>

static _Bool Encode(MotorUart_Slave2Master_t *self, uint16_t master_id,
                    uint16_t slave_id, uint8_t cmd, uint8_t *data,
                    uint16_t data_len);

/**
 * @brief   初始化电机UART从机对象
 * @param   self  电机UART从机对象指针
 * @retval  true  初始化成功
 * @retval  false 初始化失败（参数为空指针）
 */
_Bool MotorUartSlave_Init(MotorUart_Slave2Master_t *self) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error in motor uart init \r\n");
    return false;
  }
  memset(self, 0, sizeof(MotorUart_Slave2Master_t));
  self->Encode = Encode;

  return true;
}

/**
 * @brief   编码电机UART发送报文（添加CRC校验）
 * @param   self      电机UART从机对象指针
 * @param   master_id 主机地址
 * @param   slave_id  从机地址
 * @param   cmd       命令码
 * @param   data      数据负载指针（可为NULL）
 * @param   data_len  数据长度
 * @retval  true  编码成功
 * @retval  false 编码失败（参数为空或超出缓冲区）
 */
static _Bool Encode(MotorUart_Slave2Master_t *self, uint16_t master_id,
                    uint16_t slave_id, uint8_t cmd, uint8_t *data,
                    uint16_t data_len) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error in motor uart encode \r\n");
    return false;
  }
  // uint16_t send_len = data_len + 5 + 2 + 2;
  self->send_len = data_len + 9;

  if (self->send_len > UART_TX_DMA_BUFFER_SIZE) {
    RTT_Print(0, "Out of buffer size \r\n");
    return false;
  }
  memset(self->buffer, 0, UART_TX_DMA_BUFFER_SIZE);
  // 设定主机地址
  self->buffer[0] = (master_id >> 8) & 0xFF;
  self->buffer[1] = master_id & 0xFF;
  self->buffer[2] = cmd;
  // 设定从机地址
  self->buffer[3] = (slave_id >> 8) & 0xFF;
  self->buffer[4] = slave_id & 0xFF;
  self->buffer[5] = (data_len >> 8) & 0xFF;
  self->buffer[6] = data_len & 0xFF;
  if (data != NULL) {
    memcpy(self->buffer + 7, data, data_len);
  }
  Append_CRC16_Check_Sum(self->buffer, self->send_len);
  return true;
}

/**
 * @brief   解码电机UART接收报文（含CRC校验）
 * @param   data_input  原始数据输入指针
 * @param   master_id   解析后的主机地址输出指针
 * @param   slave_id    解析后的从机地址输出指针
 * @param   cmd         解析后的命令码输出指针
 * @param   data_len    解析后的数据长度输出指针
 * @param   data_out    数据负载输出缓冲区指针（可为NULL）
 * @param   message_len 报文总长度
 * @retval  true  解码成功（CRC校验通过）
 * @retval  false 解码失败（参数错误、报文非法或CRC校验失败）
 */
_Bool MotorUartSlave_Decode(uint8_t *data_input, uint16_t *master_id,
                            uint16_t *slave_id, uint8_t *cmd,
                            uint16_t *data_len, uint8_t *data_out,
                            uint16_t message_len) {
  if (data_input == NULL || master_id == NULL || slave_id == NULL ||
      cmd == NULL || data_len == NULL) {
    RTT_Print(0, "Null pointer error in motor uart decode \r\n");
    return false;
  }
  if (message_len < 9) {
    RTT_Print(0, "Illega message \r\n");
    return false;
  }
  if (!Verify_CRC16_Check_Sum(data_input, message_len)) {
    return false;
  }
  *master_id = ((uint16_t)data_input[0] << 8) | data_input[1];
  *cmd = data_input[2];
  *slave_id = ((uint16_t)data_input[3] << 8) | data_input[4];
  *data_len = ((uint16_t)data_input[5] << 8) | data_input[6];
  if (data_out != NULL) {
    memcpy(data_out, data_input + 7, *data_len);
  }
  return true;
}
