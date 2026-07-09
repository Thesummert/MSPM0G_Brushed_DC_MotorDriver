#ifndef __MOTOR_RUNNER_H__
#define __MOTOR_RUNNER_H__

#include <math.h>
#include <stdint.h>
#include <sys/types.h>
#ifdef __cplusplus
extern "C" {
#endif

#include "brushed_motor.h"
#include "pid.h"

/*有刷电机控制结构体*/
typedef struct BrushedMotorRunner_t {
  EF_BrushedMotor_t *motor; // 电机结构体指针
  PIDInstance omega_pid;    // 角速度PID
  float set_omega;          // 设置角速度
  float lp_speed;           // 低通滤波后的输出轴角速度
  float low_pass_filter;    // 低通滤波系数
  _Bool is_inited;          // 是否完成了初始化

  _Bool (*Run)(struct BrushedMotorRunner_t *self, float set, float dt);
  _Bool (*Break)(struct BrushedMotorRunner_t *self);
  _Bool (*Start)(struct BrushedMotorRunner_t *self);
  _Bool (*Stop)(struct BrushedMotorRunner_t *self);
} BrushedMotorRunner_t;

_Bool BrushedMotorRunner_Init(BrushedMotorRunner_t *self,
                              EF_BrushedMotor_t *motor,
                              PID_Init_Config_s *pid_init,
                              float_t low_pass_filter);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_RUNNER_H__ */
