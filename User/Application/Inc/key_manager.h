#ifndef __KEY_MANAGER_H__
#define __KEY_MANAGER_H__

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#include "comm_key.h"

typedef enum {
  KEY_IDLE,
  KEY_SETTING_SLAVE_ID,
  KEY_SETTING_MASTER_ID,
} KeyManagerStatus_e;

typedef struct KeyManager_t {
  EF_Device_CommKey_t *key;
  KeyManagerStatus_e status;
  _Bool has_answer; // 是否有需要读取的设定值
                    // 需要读取的参数
  uint8_t output_times;
  KeyManagerStatus_e output_status;
  _Bool is_inited;

  _Bool (*Scan)(struct KeyManager_t *self, uint64_t delta_time);
  _Bool (*GetResult)(struct KeyManager_t *self, KeyManagerStatus_e *status,
                     uint8_t *times);
} KeyManager_t;

_Bool EF_App_KeyManager_Init(KeyManager_t *self, EF_Device_CommKey_t *key);

#ifdef __cplusplus
}
#endif

#endif /* __KEY_MANAGER_H__ */
