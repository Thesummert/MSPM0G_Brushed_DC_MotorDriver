#include "motor_runner.h"
#include "SEGGER_RTT.h"
#include "mcu_config.h"
#include "pid.h"
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

static _Bool Run(BrushedMotorRunner_t *self, float set, float dt);
static _Bool Break(BrushedMotorRunner_t *self);
static _Bool Start(BrushedMotorRunner_t *self);
static _Bool Stop(BrushedMotorRunner_t *self);

_Bool BrushedMotorRunner_Init(BrushedMotorRunner_t *self,
                              EF_BrushedMotor_t *motor,
                              PID_Init_Config_s *pid_init,
                              float_t low_pass_filter) {
  if (self == NULL || motor == NULL) {
    RTT_Print(0, "Null pointer error in motor runner init \r\n");
    return false;
  }
  memset(self, 0, sizeof(BrushedMotorRunner_t));
  self->motor = motor;
  self->low_pass_filter = low_pass_filter;
  PIDInit(&self->omega_pid, pid_init);

  // 绑定函数指针
  self->Run = Run;
  self->Break = Break;
  self->Start = Start;
  self->Stop = Stop;

  self->is_inited = true;

  return true;
}

static _Bool Run(BrushedMotorRunner_t *self, float set, float dt) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error in motor runner set \r\n");
    return false;
  }
  if (self->is_inited == false) {
    return false;
  }

  self->motor->GetSpeed(self->motor, dt, NULL, NULL);
  // 低通滤波
  self->lp_speed = (1 - self->low_pass_filter) * self->lp_speed +
                   self->low_pass_filter * self->motor->omega_output;
  PIDCalculate(&self->omega_pid, set, self->lp_speed); // 默认使用输出轴角速度
  self->motor->SetOutput(self->motor, self->omega_pid.Output);
  return false;
}

static _Bool Break(BrushedMotorRunner_t *self) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error in motor runner break \r\n");
    return false;
  }
  if (self->is_inited == false) {
    return false;
  }
  self->motor->Break(self->motor);
  PIDClear(&self->omega_pid);
  return true;
}

static _Bool Start(BrushedMotorRunner_t *self) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error in motor runner start \r\n");
    return false;
  }
  if (self->is_inited == false) {
    return false;
  }
  self->motor->Start(self->motor);
  return true;
}

static _Bool Stop(BrushedMotorRunner_t *self) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error in motor runner stop \r\n");
    return false;
  }
  if (self->is_inited == false) {
    return false;
  }
  self->motor->Stop(self->motor);
  PIDClear(&self->omega_pid);
  return true;
}
