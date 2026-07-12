#ifndef __COMM_KEY_H__
#define __COMM_KEY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_mspm0g_gpio.h"
#include <stdint.h>

typedef enum {
  EF_COMM_KEY_IDLE,      // 按键未按下
  EF_COMM_KEY_TOUCH,     // 按键按下
  EF_COMM_KEY_IDLE_TEMP, // 按键临时松开
} EF_CommKeyStatus_e;

typedef enum {
  EF_COMM_TOUCH_NONE,  // 无点击
  EF_COMM_TOUCH_TIMES, // 点击
  EF_COMM_TOUCH_HOLD,  // 长按

} EF_CommKetOutput_e;

typedef struct EF_CommKey_t {
  EasyFrame_GPIO_Typedef_t *gpio;
  EF_CommKeyStatus_e status;
  uint64_t hold_time;      // 按住时长 用于判断长按
  uint64_t set_hold_time;  // 设定长按时长
  uint64_t wait_time;      // 消抖计时/多次点击间隔计时
  uint64_t set_wait_time;  // 设定按键消抖时长
  uint64_t set_multi_time; // 设定多次点击检测超时
  _Bool level;             // 按下对应的电平
  _Bool is_inited;         // 是否初始化
  _Bool output_lock;  // 输出锁 出现符合条件的点击必须读取后才能进行下一次读取
  uint8_t touch_time; // 多次点击次数 假定一次点击不可能超过0xFF次

  _Bool (*Touch)(struct EF_CommKey_t *self, uint64_t delta_time);
  _Bool (*GetState)(struct EF_CommKey_t *self, EF_CommKetOutput_e *output,
                    uint8_t *touch_time);
} EF_Device_CommKey_t;

_Bool EF_CommKey_Init(EF_Device_CommKey_t *self, EasyFrame_GPIO_Typedef_t *key,
                      _Bool level);

#ifdef __cplusplus
}
#endif

#endif /* __COMM_KEY_H__ */
