#include "motor_task.h"
#include "bsp_mspm0g_tim_base.h"
#include "detect_task.h"
#include "mcu_device.h"
#include "motor_runner.h"
#include "pid.h"
#include <math.h>
#include <reent.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define MOTOR_PID_KP 0.0f
#define MOTOR_PID_KI 0.0f
#define MOTOR_PID_KD 0.0f
#define MOTOR_PID_MAX_OUT 0.0f
#define MOTOR_PID_INTEGRAL_LIMIT 0.0f
#define MOTOR_PID_DEADBAND 0.0f
#define MOTOR_PID_LPF 0.0f

static MotorTask_t motor_task;
static void MotorTask_Run();

// 测试用参数
// _Bool test_flag = 0;
// float set_speed = 0.0f;

/**
 * @brief 电机任务主循环。
 *
 * 该任务以 1KHz 运行，持续驱动电机状态机和
 * PID 控制。
 */
void MotorTask() {
  /*任务以1KHZ运行*/
  while (1) {
    MotorTask_Run();
    vTaskDelay(1);
  }
}

/**
 * @brief 初始化电机任务上下文和电机运行器。
 */
void MotorTask_Init() {
  // PID初始化结构体
  PID_Init_Config_s pid_init = {
      .Kp = MOTOR_PID_KP,
      .Ki = MOTOR_PID_KI,
      .Kd = MOTOR_PID_KD,
      .MaxOut = MOTOR_PID_MAX_OUT,
      .IntegralLimit = MOTOR_PID_INTEGRAL_LIMIT,
      .DeadBand = MOTOR_PID_DEADBAND,
      .Output_LPF_RC = MOTOR_PID_LPF,
      .Improve = PID_Integral_Limit | PID_OutputFilter,

  };
  memset(&motor_task, 0, sizeof(MotorTask_t));
  BrushedMotorRunner_Init(&motor_task.motor, EFDevice_Get_Motor(), &pid_init,
                          0.7f);
  motor_task.time_line = EasyFrameSysTime_GetTimeline_s();
}

/**
 * @brief 执行一次电机控制状态更新。
 *
 *
 * 根据当前电机状态切换空闲、运行和刹车逻辑，并更新运行时长。

 */
static void MotorTask_Run() {
  BrushedMotorRunner_t *motor = &motor_task.motor;
  // 由于LED控制需要传入时间线 这里多加了点浮点计算任务
  float old_timeline = motor_task.time_line;
  motor_task.time_line = EasyFrameSysTime_GetTimeline_s();
  motor_task.dt = motor_task.time_line - old_timeline;
  /*根据电机状态选择分支*/
  switch (motor_task.status) {
  case MOTOR_IDLE:
    // 空闲时 关闭PWM输出
    motor->Stop(motor);
    motor_task.is_running = false;
    break;
  case MOTOR_RUN:
    // 开启PWM输出 并设定速度
    if (motor_task.is_running == false) {
      motor->Start(motor);
      motor_task.is_running = true;
    } else {
      motor->Run(motor, motor_task.set_omega, motor_task.dt);
    }
    break;
    break;
  case MOTOR_BREAK:
    // 刹车的状态下也需要保持PWM输出
    if (motor_task.is_running == false) {
      motor->Start(motor);
      motor_task.is_running = true;
    } else {
      motor->Break(motor);
    }
    break;
  }
  //
  // if (test_flag == 1) {
  //   if (motor_task.is_running == false) {
  //     motor->Start(motor);
  //     motor_task.is_running = true;
  //   } else {
  //     motor->Run(motor, set_speed, motor_task.dt);
  //   }
  // }
  // else {
  //     motor_task.is_running = false;
  //     motor->Stop(motor);
  // }
}

/**
 * @brief 获取电机 PID 实例指针。
 * @return 电机 PID 实例指针。

 */
PIDInstance *MotorTask_GetPID() { return &motor_task.motor.omega_pid; }

/**
 * @brief 获取电机运行器实例指针。
 * @return 电机运行器实例指针。

 */
BrushedMotorRunner_t *MotorTask_GetRunner() { return &motor_task.motor; }

/**
 * @brief 获取电机任务上下文指针。
 * @return 电机任务上下文指针。

 */
MotorTask_t *MotorTask_GetTask() { return &motor_task; }
