#include "comm_key.h"
#include "bsp_mspm0g_gpio.h"
#include "mcu_config.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define DEFAULT_WAIT_TIME 100     // 默认消抖时长 (us)
#define DEFAULT_HOLD_TIME 500000  // 默认长按时长 (us)
#define DEFAULT_MULTI_TIME 300000 // 默认多次点击检测超时 (us)

static _Bool Touch(EF_Device_CommKey_t *self, uint64_t delta_time);
static _Bool GetState(EF_Device_CommKey_t *self, EF_CommKetOutput_e *output,
                      uint8_t *touch_time);

/**
 * @brief   初始化按键对象
 * @param   self   按键对象指针
 * @param   key    GPIO引脚配置指针
 * @param   level  按下时对应的电平
 * @retval  true  初始化成功
 * @retval  false 初始化失败（参数为空指针）
 */
_Bool EF_CommKey_Init(EF_Device_CommKey_t *self, EasyFrame_GPIO_Typedef_t *key,
                      _Bool level) {
  if (self == NULL || key == NULL) {
    RTT_Print(0, "Null pointer error in comm key init \r\n");
    return false;
  }
  memset(self, 0, sizeof(EF_Device_CommKey_t));
  self->gpio = key;
  self->set_wait_time = DEFAULT_WAIT_TIME;
  self->set_hold_time = DEFAULT_HOLD_TIME;
  self->set_multi_time = DEFAULT_MULTI_TIME;
  self->level = level;
  self->Touch = Touch;
  self->GetState = GetState;
  self->is_inited = true;

  return true;
}

/**
 * @brief   按键触摸检测（消抖、长按、多击状态机处理）
 * @param   self       按键对象指针
 * @param   delta_time 距离上次调用的时间增量 (us)
 * @retval  true  处理正常
 * @retval  false 处理失败（参数为空或未初始化）
 */
static _Bool Touch(EF_Device_CommKey_t *self, uint64_t delta_time) {
  if (self == NULL || !self->is_inited) {
    RTT_Print(0, "Null pointer error in comm key touch \r\n");
    return false;
  }

  _Bool level;
  self->gpio->Read(self->gpio, &level);
  _Bool pressed = (level == self->level);

  if (self->output_lock == true) {
    return false;
  }

  switch (self->status) {
  case EF_COMM_KEY_IDLE:
    if (pressed) {
      self->wait_time = 0;
      self->hold_time = 0;
      self->touch_time = 0;
      self->status = EF_COMM_KEY_TOUCH;
    }
    break;

  case EF_COMM_KEY_TOUCH:
    if (pressed) {
      self->wait_time += delta_time;
      self->hold_time += delta_time;
      if (self->hold_time > self->set_hold_time && self->touch_time != 0xFF) {
        self->touch_time = 0xFF; // 长按标记
      }
    } else {
      if (self->wait_time > self->set_wait_time) {
        if (self->touch_time == 0xFF) {
          self->status = EF_COMM_KEY_IDLE; // 长按释放
          self->output_lock = true;
        } else {
          self->touch_time += 1;
          self->wait_time = 0;
          self->hold_time = 0;
          self->status = EF_COMM_KEY_IDLE_TEMP; // 等待多击
        }
      } else {
        // 消抖时间内释放，视为噪声
        self->touch_time = 0;
        self->wait_time = 0;
        self->hold_time = 0;
        self->status = EF_COMM_KEY_IDLE;
      }
    }
    break;

  case EF_COMM_KEY_IDLE_TEMP:
    if (pressed) {
      self->wait_time = 0;
      self->hold_time = 0;
      self->status = EF_COMM_KEY_TOUCH; // 下一击
    } else {
      self->wait_time += delta_time;
      if (self->wait_time > self->set_multi_time) {
        self->status = EF_COMM_KEY_IDLE; // 多击窗口超时，输出结果
        self->output_lock = true;
      }
    }
    break;
  default:
    break;
  }
  return true;
}

/**
 * @brief   获取按键状态（点击次数 / 长按 / 无操作）
 * @param   self       按键对象指针
 * @param   output     输出状态枚举指针
 * @param   touch_time 点击次数输出指针（可为NULL）
 * @retval  true  成功获取状态
 * @retval  false 无有效输出或参数错误
 */
static _Bool GetState(EF_Device_CommKey_t *self, EF_CommKetOutput_e *output,
                      uint8_t *touch_time) {
  if (self == NULL || !self->is_inited || output == NULL) {
    RTT_Print(0, "Null pointer error in comm key get state \r\n");
    return false;
  }
  if (self->output_lock == false) {
    return false;
  } else {
    if (self->hold_time > self->set_hold_time) {
      *output = EF_COMM_TOUCH_HOLD;
    } else if (self->touch_time > 0) {
      *output = EF_COMM_TOUCH_TIMES;
      if (touch_time != NULL) {
        *touch_time = self->touch_time;
      }
    } else {
      *output = EF_COMM_TOUCH_NONE;
    }
    self->output_lock = false;
  }
  return true;
}
