#include "bsp_time.h"
#include "mcu_config.h"
#include "ti/driverlib/dl_timer.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/reent.h>

static _Bool StartCounter(EF_BSP_TimerBase_t *self);
static _Bool StopCounter(EF_BSP_TimerBase_t *self);
static _Bool setCounterMode(EF_BSP_TimerBase_t *self,
                            EF_TimerCounterMode_e mode);
static _Bool SetPWM_Float(EF_BSP_TimerPWM_t *self, uint8_t channel_id,
                          float radio);
static _Bool SetPWM(EF_BSP_TimerPWM_t *self, uint8_t channel_id,
                    uint32_t compare);
static _Bool SetPWM_Mode(EF_BSP_TimerPWM_t *self, EF_TimerPWMMode_e mode);
static _Bool StartPWM(EF_BSP_TimerPWM_t *self);
static _Bool StopPWM(EF_BSP_TimerPWM_t *self);

_Bool EF_BSP_TimerBase_Init(EF_BSP_TimerBase_t *self, GPTIMER_Regs *tim,
                            uint32_t clock_source_freq, uint32_t max_prescaler,
                            uint32_t max_auto_reload_num) {
  if (self == NULL || tim == NULL) {
    RTT_Print(0, "Null pointer error in tim init \r\n");
    return false;
  }
  memset(self, 0, sizeof(EF_BSP_TimerBase_t));
  self->clock_source_freq = clock_source_freq;
  self->mspm0g.tim = tim;
  self->max_auto_reload_num = max_auto_reload_num;
  self->max_prescaler = max_prescaler;

  self->StartCounter = StartCounter;
  self->StopCounter = StopCounter;
  self->setCounterMode = setCounterMode;

  self->is_inited = true;

  return true;
}

static _Bool StartCounter(EF_BSP_TimerBase_t *self) {
  /*开始计数*/
  if (self == NULL) {
    RTT_Print(0, "Null pointer error in tim start \r\n");
    return false;
  }
  if (self->is_inited == false) {
    return false;
  }
  DL_Timer_startCounter(self->mspm0g.tim);

  return true;
}

static _Bool StopCounter(EF_BSP_TimerBase_t *self) {
  /*关闭计数*/
  if (self == NULL) {
    RTT_Print(0, "Null pointer error in tim stop \r\n");
    return false;
  }
  if (self->is_inited == false) {
    return false;
  }
  DL_Timer_stopCounter(self->mspm0g.tim);

  return true;
}

static _Bool setCounterMode(EF_BSP_TimerBase_t *self,
                            EF_TimerCounterMode_e mode) {
  /*设定计数器模式*/
  if (self == NULL) {
    RTT_Print(0, "Null pointer error in tim start \r\n");
    return false;
  }
  if (self->is_inited == false) {
    return false;
  }
  self->counter_mode = mode;
  // 根据架构设定参数设定实际参数
  switch (self->counter_mode) {
  case TIMER_COUNT_MODE_ONCE_UP:
    self->mspm0g.mode = DL_TIMER_TIMER_MODE_ONE_SHOT_UP;
    break;
  case TIMER_COUNT_MODE_ONCE_DOWN:
    self->mspm0g.mode = DL_TIMER_TIMER_MODE_ONE_SHOT;
    break;
  case TIMER_COUNT_MODE_ONCE_UP_DOWN:
    self->mspm0g.mode = DL_TIMER_TIMER_MODE_ONE_SHOT_UP_DOWN;
    break;
  case TIMER_COUNT_MODE_PERIODIC_UP:
    self->mspm0g.mode = DL_TIMER_TIMER_MODE_PERIODIC_UP;
    break;
  case TIMER_COUNT_MODE_PERIODIC_DOWN:
    self->mspm0g.mode = DL_TIMER_TIMER_MODE_PERIODIC;
    break;
  case TIMER_COUNT_MODE_PERIODIC_UP_DOWN:
    self->mspm0g.mode = DL_TIMER_TIMER_MODE_PERIODIC_UP_DOWN;
    break;
  default:
    self->mspm0g.mode = DL_TIMER_TIMER_MODE_PERIODIC_UP;
    break;
  }

  DL_TimerG_TimerConfig config = {
      .period = 0,
      .timerMode = self->mspm0g.mode,
      .startTimer = DL_TIMER_STOP,
  };
  DL_Timer_initTimerMode(self->mspm0g.tim, &config);

  return true;
}

/*===========PWM===========*/
_Bool EF_BSP_TimerPWM_Init(EF_BSP_TimerPWM_t *self, EF_BSP_TimerBase_t *etim,
                           uint8_t max_channel) {
  if (self == NULL || etim == NULL) {
    RTT_Print(0, "Null pointer error in timer pwm init \r\n");
    return false;
  }
  if (etim->is_inited == false) {
    RTT_Print(0, "Need to init tim first then init pwm \r\n");
    return false;
  }
  memset(self, 0, sizeof(EF_BSP_TimerPWM_t));
  self->max_channel = max_channel;
  self->etim = etim;

  // MSPM0需要判断是否支持4channel
  if (max_channel > 2) {
      self->mspm0g.is_support4channel = true;
  }

  // 初始化函数指针
  self->SetPWM_Float = SetPWM_Float;
  self->SetPWM = SetPWM;
  self->StartPWM = StartPWM;
  self->StopPWM = StopPWM;
  self->SetPWM_Mode = SetPWM_Mode;

  self->is_inited = true;

  return true;
}

static _Bool SetPWM(EF_BSP_TimerPWM_t *self, uint8_t channel_id,
                    uint32_t compare) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error in pwm set \r\n");
    return false;
  }
  if (self->is_inited == false) {
    return false;
  }
  // 超出最大值则设定为最大值
  if (compare > self->etim->max_auto_reload_num) {
    compare = self->etim->max_auto_reload_num;
  }
  // 对于此设定方式 不设置浮点设定值 减少性能开销
  self->channels[channel_id] = compare;

  DL_Timer_setCaptureCompareValue(self->etim->mspm0g.tim, compare, channel_id);
  return true;
}

static _Bool SetPWM_Float(EF_BSP_TimerPWM_t *self, uint8_t channel_id,
                          float radio) {

  if (self == NULL) {
    RTT_Print(0, "Null pointer error in pwm set \r\n");
    return false;
  }
  if (self->is_inited == false) {
    return false;
  }
  if (radio > 1.0f) {
    radio = 1.0f;
  } else if (radio < 0.0f) {
    radio = 0.0f;
  }
  self->radio[channel_id] = radio;
  self->channels[channel_id] = self->etim->max_auto_reload_num * radio;
  DL_Timer_setCaptureCompareValue(self->etim->mspm0g.tim,
                                  self->channels[channel_id], channel_id);

  return true;
}

static _Bool StartPWM(EF_BSP_TimerPWM_t *self) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error in pwm start \r\n");
    return false;
  }
  if (self->is_inited == false) {
    return false;
  }
  self->etim->StartCounter(self->etim);

  return true;
}

static _Bool StopPWM(EF_BSP_TimerPWM_t *self) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error in pwm stop \r\n");
    return false;
  }
  if (self->is_inited == false) {
    return false;
  }
  self->etim->StopCounter(self->etim);

  return true;
}

static _Bool SetPWM_Mode(EF_BSP_TimerPWM_t *self, EF_TimerPWMMode_e mode) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error in pwm set mode \r\n");
    return false;
  }
  if (self->is_inited == false) {
    return false;
  }
  // 按照ef参数设定实际单片机控制参数
  switch (mode) {
  case TIMER_PWM_MODE_UP:
    self->mode = TIMER_PWM_MODE_UP;
    self->mspm0g.pwm_mode = DL_TIMER_PWM_MODE_EDGE_ALIGN_UP;
    break;
  case TIMER_PWM_MODE_DOWN:
    self->mode = TIMER_PWM_MODE_DOWN;
    self->mspm0g.pwm_mode = DL_TIMER_PWM_MODE_EDGE_ALIGN;
    break;
  case TIMER_PWM_MODE_CENTER:
    self->mode = TIMER_PWM_MODE_CENTER;
    self->mspm0g.pwm_mode = DL_TIMER_PWM_MODE_CENTER_ALIGN;
    break;
  default:
    return false;
  }
  const DL_TimerA_PWMConfig config = {
      .period = 0,
      .startTimer =  DL_TIMER_STOP,
      .pwmMode = self->mspm0g.pwm_mode,
      .isTimerWithFourCC = self->mspm0g.is_support4channel,
  };
  // 设定PWM
  DL_Timer_initPWMMode(self->etim->mspm0g.tim, &config);

  return true;
}
