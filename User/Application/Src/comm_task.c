#include "comm_task.h"
#include "abstract_queue.h"
#include "mcu_config.h"
#include "mcu_device.h"
#include "motor_can_control.h"
#include "motor_module_manager.h"
#include "motor_runner.h"
#include "motor_task.h"
#include "motor_uart_control.h"
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
static const uint16_t COUNTER_RELOAD[4] = {100, 200, 500, 1000};
/*串口缓冲区 最后两位为数据长度*/
static uint8_t uart_buffer[4 * (UART_TX_DMA_BUFFER_SIZE + 2)];

static void CommTask_CAN_Trasnmit(CommTask_t *self);
static void CommTask_ReloadCounter(CommTask_t *self);
static void CommTask_Uart_SetValue(CommTask_t *self);

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
  EF_Algorithm_Queue_Init(&comm_task.uart_queue, 4, UART_TX_DMA_BUFFER_SIZE + 2,
                          uart_buffer);
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

    uint8_t status = 0;
    self->can_message.Encoder(
        &self->can_message, status,
        self->motor->motor->omega_output); // 返回输出轴角速度
    self->ecan->TransmitFIFO(self->ecan, self->can_message.master_id,
                             self->can_message.feedback.datas, 3, false);
  }
}

static void CommTask_ReloadCounter(CommTask_t *self) {
  /*单独运行 防止两种发送方式相互干扰*/
  if (self->counter == self->reload_counter) {
    self->counter = 0;
  }
}

static void CommTask_Uart_SetValue(CommTask_t *self) {
  /*串口数据解码*/
  if (self->uart_queue.length > 0) {
    // 如果队列里面有数据 则解码数据
    uint16_t master_id;
    uint16_t slave_id;
    uint8_t cmd;
    uint16_t data_len;
    uint8_t data_out[UART_TX_DMA_BUFFER_SIZE];

    // 直接读取缓冲区 加快处理速度
    MotorUartSlave_Decode(
        self->uart_queue.buffer_ptr +
            self->uart_queue.pointer_index * self->uart_queue.item_size,
        &master_id, &slave_id, &cmd, &data_len, data_out,
        self->uart_queue.pointer_index * self->uart_queue.item_size +
            UART_TX_DMA_BUFFER_SIZE + 2);
    self->uart_queue.Drop(&self->uart_queue, NULL);
    // 只处理本机数据
    // @TODO 只有在ID相同的时候才进行Decode处理
    if (slave_id == self->manager.stroage.storge.slave_id) {
      switch (cmd) {

      case MOTOR_CMD_SET:
        /*将电机模块切换到设定模式*/
        break;
      case MOTOR_CMD_DRIVE:
        /*电机驱动命令 设定电机驱动数值*/
        break;
      case MOTOR_CMD_TRANSMIT:
        /*数据传输命令 搭配MOTOR_CMD_SET进行数值设定传输*/
      // case MOTOR_CMD_FEEDBACK:
      //     /*反馈报文 电机模块不会接收到这个命令*/
      case MOTOR_CMD_ACK:
        /*回复报文 当电机设定数值成功的时候会主动发送*/
        /*同时也是主机检测从机是否在线的时候发送的报文*/
        // 只发送回复报文 不带其他数据
        self->uart_message.Encode(
            &self->uart_message, self->manager.stroage.storge.master_id,
            self->manager.stroage.storge.slave_id, MOTOR_CMD_ACK, NULL, 0);

        // 串口数据发送
        uint16_t try_time = 0;
        while (!self->euart->TransmitDMA(self->euart, self->uart_message.buffer,
                                         self->uart_message.send_len) &&
               try_time < 10) {
          // 尝试10次设定发送
          try_time++;
        }
        break;
      default:
        break;
      }
    }
  }
}
