#include "motor_can_control.h"
#include "stdbool.h"
#include <stdint.h>
#include <string.h>

#if BRUSHED_MOTOR_IS_MASTER == 1

#else
static void S2MEncoder(MotorCan_Slave2Master_t *self, uint8_t status,
                       float speed);

_Bool MotorCanSlave_Init(MotorCan_Slave2Master_t *self, uint16_t id) {
  if (self == NULL) {
    RTT_Print(0, "NULL pointer error in motor slave init \r\n");
    return false;
  }
  memset(self, 0, sizeof(MotorCan_Slave2Master_t));
  self->slave_id = id;
  self->Encoder = S2MEncoder;

  return true;
}

static void S2MEncoder(MotorCan_Slave2Master_t *self, uint8_t status,
                       float speed) {
  // 设计上调用此函数一定不是空指针 且不会改成普通函数式调用的需求
  // 所以不判断空指针
  uint8_t direction = (speed < 0.0f) ? true : false;
  self->feedback.cmd.status = status | (direction << 7);
  self->feedback.cmd.value = (uint16_t)(speed * 100.0f);
}

#endif
