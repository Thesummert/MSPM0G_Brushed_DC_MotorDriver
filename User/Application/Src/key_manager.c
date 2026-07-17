#include "key_manager.h"
#include "comm_key.h"
#include "mcu_config.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/reent.h>

static _Bool Scan(KeyManager_t *self, uint64_t delta_time);
static _Bool GetResult(KeyManager_t *self, KeyManagerStatus_e *status,
                       uint8_t *times);

_Bool EF_App_KeyManager_Init(KeyManager_t *self, EF_Device_CommKey_t *key) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error in key manager init \r\n");
    return false;
  }
  memset(self, 0, sizeof(KeyManager_t));
  self->key = key;
  self->GetResult = GetResult;
  self->Scan = Scan;

  self->is_inited = true;

  return true;
}

_Bool Scan(KeyManager_t *self, uint64_t delta_time) {
  if (self == NULL || self->is_inited == false) {
    RTT_Print(0, "Null pointer error or not init in key manager scan \r\n");
    return false;
  }
  EF_Device_CommKey_t *key = self->key;
  key->Touch(key, delta_time);
  // 在设定模式下 除了出错以外的退出都是在此函数外部处理的
  if (self->has_answer == false) {
    if (key->output_lock == true) {
      EF_CommKetOutput_e output;
      uint8_t touch_time;
      key->GetState(key, &output, &touch_time);
      switch (self->status) {
      case KEY_IDLE:
        // 长按进入设定从机ID 短按三次进入设定主机ID
        if (output == EF_COMM_TOUCH_HOLD) {
          self->status = KEY_SETTING_SLAVE_ID;
          self->has_answer = true;
        } else if (output == EF_COMM_TOUCH_TIMES || touch_time == 3) {
          self->status = KEY_SETTING_MASTER_ID;
          self->has_answer = true;
        }
      case KEY_SETTING_SLAVE_ID:
        if (output == EF_COMM_TOUCH_TIMES) {
          if (fabsf(touch_time) < 20) {
            self->output_times = touch_time;
            self->has_answer = true;
          }
        } else {
          self->status = KEY_IDLE;
        }
        break;
      case KEY_SETTING_MASTER_ID:
        if (output == EF_COMM_TOUCH_TIMES) {
          if (fabsf(touch_time) < 20) {
            self->output_times = touch_time;
            self->has_answer = true;
          }
        } else {
          self->status = KEY_IDLE;
        }
        break;
      default:
        break;
      }
    }
  }
  return true;
}

_Bool GetResult(KeyManager_t *self, KeyManagerStatus_e *status,
                uint8_t *times) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error or not init in key manager get \r\n");
    return false;
  }
  if (self->has_answer == true) {
    *status = self->status;
    *times = self->output_times;
    self->has_answer = false;
    self->status = KEY_IDLE;
  }

  return true;
}
