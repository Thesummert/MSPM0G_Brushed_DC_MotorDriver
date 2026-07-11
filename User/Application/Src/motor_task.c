#include "motor_task.h"
#include "bsp_mspm0g_tim_base.h"
#include "mcu_device.h"
#include "motor_runner.h"
#include "pid.h"
#include <reent.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "detect_task.h"

#define MOTOR_PID_KP 0.0f
#define MOTOR_PID_KI 0.0f
#define MOTOR_PID_KD 0.0f
#define MOTOR_PID_MAX_OUT 0.0f
#define MOTOR_PID_INTEGRAL_LIMIT 0.0f
#define MOTOR_PID_DEADBAND 0.0f
#define MOTOR_PID_LPF 0.0f

static EF_App_SoftWDT_t wdt_motor_task;
static MotorTask_t motor_task;
static void MotorTask_Run();

void MotorTask() {
  /*任务以1KHZ运行*/
  while (1) {
    MotorTask_Run();
    vTaskDelay(1);
  }
}

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

  };
  memset(&motor_task, 0, sizeof(MotorTask_t));
  BrushedMotorRunner_Init(&motor_task.motor, EFDevice_Get_Motor(), &pid_init,
                          0.7f);
  motor_task.time_line = EasyFrameSysTime_GetTimeline_s();

  // 初始化看门狗
  uint8_t id[6] = "motor";
  EF_SoftWDT_Init(&wdt_motor_task, id, 6, 10, NULL, NULL);
  EF_App_SoftWDT_Group_Add(EF_App_SoftWDT_Group_Get(0 ), &wdt_motor_task);
}

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
}
