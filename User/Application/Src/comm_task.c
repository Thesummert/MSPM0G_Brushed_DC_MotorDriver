#include "comm_task.h"
#include "mcu_config.h"
#include "mcu_device.h"
#include "motor_can_control.h"
#include "motor_module_manager.h"
#include "motor_runner.h"
#include "motor_task.h"
#include "pid.h"
#include "qei_encoder.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/**
 * @note 通信任务 同时也负责管理控制
 * */

CommTask_t comm_task;

/*频率定时器装载值*/
const uint16_t COUNTER_RELOAD[4] = {100, 200, 500, 1000};

static void CommTask_CAN_Trasnmit(CommTask_t *self);
static void CommTask_CAN_ReloadCounter(CommTask_t *self);

void CommTask() {
  while (1) {
    vTaskDelay(1000);
  }
}

void CommTask_Init() {
  memset(&comm_task, 0, sizeof(CommTask_t));
  comm_task.ecan = EFDevice_Get_CAN();
  comm_task.euart = EFDevice_Get_Uart();
  comm_task.status_led = EFDevice_Get_LED();
  comm_task.cmd_key = EFDevice_Get_Key();
  comm_task.motor = MotorTask_GetRunner();
  MotorManager_Init(&comm_task.manager, EFDevice_Get_EEPROM());
  // 读取eeprom中的数据
  // 总共尝试读取三次
  if (comm_task.manager.Read(&comm_task.manager, 3)) {
    // 读取成功 将设置ID
    BrushedMotorRunner_t *runner = MotorTask_GetRunner();
    PIDInstance *pid = MotorTask_GetPID();
    MotorCanSlave_Init(&comm_task.can_message,
                       comm_task.manager.stroage.storge.master_id,
                       comm_task.manager.stroage.storge.slave_id);
    PID_Init_Config_s pid_init = {
        .Kp = comm_task.manager.stroage.storge.kp,
        .Ki = comm_task.manager.stroage.storge.ki,
        .Kd = comm_task.manager.stroage.storge.kd,
        .MaxOut = comm_task.manager.stroage.storge.maxout,
        .DeadBand = comm_task.manager.stroage.storge.deadband,
        .Improve = PID_Integral_Limit | PID_OutputFilter,
        .IntegralLimit = comm_task.manager.stroage.storge.integral_limit,
        .Output_LPF_RC = comm_task.manager.stroage.storge.lpf,
    };
    PIDInit(pid, &pid_init);
    // 设定电机参数
    runner->motor->qei->radio = comm_task.manager.stroage.storge.radio;
    runner->motor->qei->multiplier =
        comm_task.manager.stroage.storge.multiplier;
    runner->motor->qei->pulse_per_rev = comm_task.manager.stroage.storge.ppr;
    runner->motor->qei->inv_radio = (runner->motor->qei->radio != 0.0f)
                                        ? (1.0f / runner->motor->qei->radio)
                                        : 0.0f;
    runner->motor->qei->inv_multiplier =
        (runner->motor->qei->multiplier != 0)
            ? (1.0f / runner->motor->qei->multiplier)
            : 0.0f;
    runner->motor->qei->inv_pulse_per_rev =
        (runner->motor->qei->pulse_per_rev != 0)
            ? (1.0f / runner->motor->qei->pulse_per_rev)
            : 0.0f;
    // 设定通信频率
    comm_task.send_freq = comm_task.manager.stroage.storge.freq;
    comm_task.reload_counter = COUNTER_RELOAD[comm_task.send_freq];
  } else {
    MotorCanSlave_Init(&comm_task.can_message, MOTOR_MODULE_DEFAULT_MASTER_ID,
                       MOTOR_MODULE_DEFAULT_ID);
    comm_task.reload_counter = COUNTER_RELOAD[0];
  }
}

static void CommTask_CAN_Trasnmit(CommTask_t *self) {
  if (self->counter == self->reload_counter) {
    // self->can_message.Encoder(&self->can_message, status,
    // self->motor->motor->omega_output); // 返回输出轴角速度
    self->ecan->TransmitFIFO(self->ecan, self->can_message.master_id,
                             self->can_message.feedback.datas, 3, false);
  }
}

static void CommTask_CAN_ReloadCounter(CommTask_t *self) {
  /*单独运行 防止两种发送方式相互干扰*/
  if (self->counter == self->reload_counter) {
    self->counter = 0;
  }
}
