#ifndef __DETECT_TASK_H__
#define __DETECT_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "freertos_manager.h"
#include "mcu_config.h"
#include <stdint.h>

#define SOFT_WDT_ID_MAX_LEN 16

typedef struct EF_App_SoftWDT_t {
  uint8_t wdt_name[SOFT_WDT_ID_MAX_LEN]; // 存储字符串 保存WDT实例名
  uint32_t load;                         // 装载值
  uint32_t cnt;                          // 看门狗计时器
  _Bool is_online;                       // 是否在线
  void (*Callback)(void *item);          // 回调函数
  void *item;                            // 万能空指针 传递参数使用
  _Bool (*Feed)(struct EF_App_SoftWDT_t *self);
} EF_App_SoftWDT_t;

typedef struct EF_App_SoftWDT_Group_t {
  EF_App_SoftWDT_t *wdts[SOFT_WDT_MAX_INST_NUM];
  uint16_t num;
  uint16_t online_num;

  _Bool (*Check)(struct EF_App_SoftWDT_Group_t *self);
} EF_App_SoftWDT_Group_t;

_Bool EF_SoftWDT_Init(EF_App_SoftWDT_t *self, uint8_t *id, uint8_t id_len,
                      uint32_t load, void *Callback, void *item);
_Bool EF_App_SoftWDT_Group_Init(EF_App_SoftWDT_Group_t *self);
_Bool EF_App_SoftWDT_Group_Add(EF_App_SoftWDT_Group_t *self,
                               EF_App_SoftWDT_t *wdt);

EF_App_SoftWDT_Group_t *EF_App_SoftWDT_Group_Get(uint16_t id);

#ifdef __cplusplus
}
#endif

#endif /* __DETECT_TASK_H__ */
