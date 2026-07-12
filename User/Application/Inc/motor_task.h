#ifndef __MOTOR_TASK_H__
#define __MOTOR_TASK_H__

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#include "freertos_manager.h"
#include "motor_runner.h"

/*电机状态枚举*/
typedef enum {
  MOTOR_IDLE = 0,
  MOTOR_RUN,
  MOTOR_BREAK,
} MotorStatus_e;

typedef struct {
  BrushedMotorRunner_t motor; // 电机
  float time_line;            // 时间线
  float dt;
  MotorStatus_e status;       // 电机状态
  _Bool is_running;           // 是否在工作
  float set_omega;            // 设计的目标角速度

} MotorTask_t;

void MotorTask_Init();

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_TASK_H__ */
