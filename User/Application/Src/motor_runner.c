#include "motor_runner.h"
#include "SEGGER_RTT.h"
#include "mcu_config.h"
#include "pid.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

static _Bool Run(BrushedMotorRunner_t *self, float set, float dt);
static _Bool Break(BrushedMotorRunner_t *self);
static _Bool Start(BrushedMotorRunner_t *self);
static _Bool Stop(BrushedMotorRunner_t *self);

/**
 * @brief         初始化电机运行器
 * @param[in,out] self            电机运行器对象指针
 * @param[in]     motor           有刷电机对象指针
 * @param[in]     pid_init        PID初始化配置
 * @param[in]     low_pass_filter 低通滤波系数（0~1，越大越平滑）
 * @retval        true            初始化成功
 * @retval        false           初始化失败（空指针）
 */
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

/**
 * @brief         运行控制：获取当前转速 → 低通滤波 → PID计算 → 设置输出
 * @param[in]     self   电机运行器对象指针
 * @param[in]     set    目标转速
 * @param[in]     dt     控制周期（秒）
 * @retval        true   执行成功
 * @retval        false  执行失败
 */
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

/**
 * @brief         急停制动：调用电机刹车并清除PID积分
 * @param[in]     self   电机运行器对象指针
 * @retval        true   制动成功
 * @retval        false  制动失败
 */
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

/**
 * @brief         启动电机：调用电机的启动接口
 * @param[in]     self   电机运行器对象指针
 * @retval        true   启动成功
 * @retval        false  启动失败
 */
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

/**
 * @brief         停止电机：调用电机停止并清除PID积分
 * @param[in]     self   电机运行器对象指针
 * @retval        true   停止成功
 * @retval        false  停止失败
 */
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

