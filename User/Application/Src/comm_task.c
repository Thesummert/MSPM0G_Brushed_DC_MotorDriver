#include "comm_task.h"
#include "abstract_queue.h"
#include "bsp_mspm0g_tim_base.h"
#include "bsp_mspm0g_usart.h"
#include "comm_led.h"
#include "crc_ref.h"
#include "detect_task.h"
#include "key_manager.h"
#include "mcu_config.h"
#include "mcu_device.h"
#include "motor_can_control.h"
#include "motor_module_manager.h"
#include "motor_runner.h"
#include "motor_task.h"
#include "motor_uart_control.h"
#include "pid.h"
#include "qei_encoder.h"
#include "ti/devices/msp/m0p/mspm0g350x.h"
#include <reent.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/select.h>

#define SET_DEFAULT_VALUE 0

/**
 * @note 通信任务 同时也负责管理控制
 * */

CommTask_t comm_task;

/*频率定时器装载值*/
static const uint16_t COUNTER_RELOAD[4] = {100, 200, 500, 1000};
/*串口缓冲区 最后两位为数据长度*/
static uint8_t uart_buffer[4 * (UART_TX_DMA_BUFFER_SIZE + 2)];

static EF_App_SoftWDT_t comm_task_dwt;

static void CommTask_CAN_Trasnmit(CommTask_t *self);
static void CommTask_ReloadCounter(CommTask_t *self);
static void CommTask_Uart_SetValue(CommTask_t *self);
static void CommTask_Uart_SetSpeed(const uint8_t *data, uint8_t data_len);
static void CommTask_Uart_Trasnmit(CommTask_t *self);
static void CommTask_LED(CommTask_t *self);
static void CommTaskWDT_Callback(void *item);

/**
 * @brief 通信任务主循环。
 *
 * 周期性处理UART设定数据，在CAN通信模式下发送电机反馈，并维护发送频率计数器。
 */
void CommTask() {
  while (1) {
    EasyFrameSysTime_GetDeltaT(&comm_task.cnt_last);
    comm_task.time_line = EasyFrameSysTime_GetTimeline_s();
    CommTask_Uart_SetValue(&comm_task);

    // 如果是CAN模式 则开始发送数据
    if (comm_task.comm_mode == 1) {
      CommTask_CAN_Trasnmit(&comm_task);
    } else {
      CommTask_Uart_Trasnmit(&comm_task);
    }
    CommTask_ReloadCounter(&comm_task);
    CommTask_LED(&comm_task);

    if ((comm_task.is_online == false) && (comm_task_dwt.is_online == true)) {
      comm_task.is_online = true;
      comm_task.status_led->Set(comm_task.status_led, EF_COMM_LED_TWINKLE, 5,
                                comm_task.time_line);
    }
    vTaskDelay(1); // 通信任务运行周期 1Khz
  }
}

/**
 * @brief 初始化通信任务上下文及通信相关模块。
 *
 * 绑定CAN、UART、LED、按键和电机运行器实例，初始化UART发送队列和电机参数管理器。
 * 若EEPROM参数读取成功，则恢复CAN
 * ID、PID参数、电机编码器参数和反馈频率；否则使用默认CAN ID和默认反馈频率。
 */
void CommTask_Init() {
  memset(&comm_task, 0, sizeof(CommTask_t));
  comm_task.ecan = EFDevice_Get_CAN();
  comm_task.euart = EFDevice_Get_Uart();
  comm_task.status_led = EFDevice_Get_LED();
  comm_task.motor = MotorTask_GetRunner();
  EasyFrameSysTime_GetDeltaT(&comm_task.cnt_last);
  EF_Algorithm_Queue_Init(&comm_task.uart_queue, 4, UART_TX_DMA_BUFFER_SIZE + 2,
                          uart_buffer);
  MotorManager_Init(&comm_task.manager, EFDevice_Get_EEPROM());
  MotorUartSlave_Init(&comm_task.uart_message);
  EF_App_KeyManager_Init(&comm_task.key_manager, EFDevice_Get_Key());
  // 设定的时候保持常量
  comm_task.status_led->Set(comm_task.status_led, EF_COMM_LED_ALWAYS_ON, 0, 0);
  comm_task.status_led->Show(comm_task.status_led, 1);
  // 读取eeprom中的数据
  // 总共尝试读取三次
  // @TODO 将设定默认值和初始化合并
  if (SET_DEFAULT_VALUE) {
    while (!comm_task.manager.SetDefaultValue(&comm_task.manager)) {
      // 如果写入失败 则重置I2C重试
      comm_task.manager.eeprom->i2c.Reset(&comm_task.manager.eeprom->i2c, 1);
      EasyFrameSysTime_Delay(0.02);
    }
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
  } else if (comm_task.manager.Read(&comm_task.manager, 3)) {
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
  comm_task.ecan->StartRec(comm_task.ecan);
  EasyFrameSysTime_GetDeltaT(&comm_task.cnt_last);
  comm_task.time_line = EasyFrameSysTime_GetTimeline_s();
  comm_task.status_led->Set(comm_task.status_led, EF_COMM_LED_TWINKLE, 5,
                            comm_task.time_line);
  uint8_t id[] = "comm";
  EF_SoftWDT_Init(&comm_task_dwt, id, 5, 50, CommTaskWDT_Callback, NULL);
  EF_App_SoftWDT_Group_Add(EF_App_SoftWDT_Group_Get(0), &comm_task_dwt);
  comm_task.is_online = true;
}

/**
 * @brief 按设定频率发送CAN反馈报文。
 * @param self 通信任务对象指针。
 *
 * 当计数器达到重装载值时，将当前电机输出轴角速度编码为反馈报文并发送到主机CAN
 * ID。
 */
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

/**
 * @brief 按设定频率发送UART反馈报文。
 * @param self 通信任务对象指针。
 *
 * 在计数器达到重装载值时，将当前电机输出轴角速度封装为UART反馈报文并通过DMA发送。
 * 报文格式为：状态字节、速度高字节、速度低字节。
 */
static void CommTask_Uart_Trasnmit(CommTask_t *self) {
  if (self->counter == self->reload_counter) {

    uint8_t status = 0;
    uint8_t datas[3];
    uint16_t value;

    uint8_t direction =
        (self->motor->motor->omega_output < 0.0f) ? true : false;
    status = status | (direction << 7);
    value = (uint16_t)(self->motor->motor->omega_output * 100.0f);
    datas[0] = status;
    datas[1] = value >> 8;
    datas[2] = value & 0xFF;
    self->uart_message.Encode(
        &self->uart_message, self->manager.stroage.storge.master_id,
        self->manager.stroage.storge.slave_id, MOTOR_CMD_FEEDBACK, datas, 3);
    // @TODO 添加串口速率与发送频率是否达标判断
    self->euart->TransmitDMA(self->euart, self->uart_message.buffer,
                             self->uart_message.send_len);
  }
}

/**
 * @brief 根据发送频率重置通信计数器。
 * @param self 通信任务对象指针。
 *
 * 该计数器独立维护，用于避免不同发送方式之间的频率控制相互干扰。
 */
static void CommTask_ReloadCounter(CommTask_t *self) {
  /*单独运行 防止两种发送方式相互干扰*/
  if (self->counter == self->reload_counter) {
    self->counter = 0;
  } else {
    ++self->counter;
  }
}

/**
 * @brief 处理UART接收队列中的电机控制报文。
 * @param self 通信任务对象指针。
 *
 * 从UART队列中取出一帧数据并解码，仅处理从机ID匹配本机的报文；当前主要响应ACK命令并通过UART
 * DMA回传确认报文。
 */
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
      comm_task_dwt.Feed(&comm_task_dwt);
      switch (cmd) {

      case MOTOR_CMD_SET:
        /*将电机模块切换到设定模式*/
        self->set_value_mode = true; // 切换到进入设定数值模式

        break;
      case MOTOR_CMD_DRIVE:
        /*电机驱动命令 设定电机驱动数值*/
        CommTask_Uart_SetSpeed(data_out, data_len);
        break;
      case MOTOR_CMD_TRANSMIT:
        /*数据传输命令 搭配MOTOR_CMD_SET进行数值设定传输*/
        if (self->set_value_mode == false) {
          break;
        }
        if (Verify_CRC16_Check_Sum(data_out, data_len)) {
          // 如果校验通过 则拷贝数据到管理器中 并且写入到EEPROM中
          memcpy(self->manager.stroage.datas, data_out, data_len);
          self->uart_message.Encode(
              &self->uart_message, self->manager.stroage.storge.master_id,
              self->manager.stroage.storge.slave_id, MOTOR_CMD_ACK, NULL, 0);
          if (!self->manager.Write(&self->manager)) {
            // 发送失败消息
            self->uart_message.Encode(
                &self->uart_message, self->manager.stroage.storge.master_id,
                self->manager.stroage.storge.slave_id, MOTOR_CMD_FAIL, NULL, 0);
            self->euart->TransmitDMA(self->euart, self->uart_message.buffer,
                                     self->uart_message.send_len);
            self->set_value_mode = false;
          }

          // 串口数据发送
          uint16_t try_time = 0;
          while (
              (!self->euart->TransmitDMA(self->euart, self->uart_message.buffer,
                                         self->uart_message.send_len)) &&
              try_time < 10) {
            // 尝试10次设定发送
            try_time++;
          }
          // 成功后将直接进行单片机重启
          // 如果最后确认成功没有发送成功 无需在意
          NVIC_SystemReset();
        } else {
          self->uart_message.Encode(
              &self->uart_message, self->manager.stroage.storge.master_id,
              self->manager.stroage.storge.slave_id, MOTOR_CMD_FAIL, NULL, 0);
          self->euart->TransmitDMA(self->euart, self->uart_message.buffer,
                                   self->uart_message.send_len);
          self->set_value_mode = false;
        }

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

/**
 * @brief 解码CAN控制报文并更新电机任务状态。
 * @param data     CAN数据缓冲区指针。
 * @param data_len CAN数据长度。
 *
 * CAN接收计数达到阈值后切换到CAN通信模式；在CAN模式下解析3字节控制报文，更新电机运行状态和目标角速度。
 */
void CommTask_CAN_Decode(const uint8_t *data, uint8_t data_len) {
  // CAN需要接收达到3次后切换到CAN通信模式
  if (comm_task.can_counter < 3) {
    ++comm_task.can_counter;
  } else {
    comm_task.comm_mode = 1;
  }
  if (comm_task.comm_mode == 1) {
    comm_task_dwt.Feed(&comm_task_dwt);
    if (data_len != 3) {
      // 不符合规范的消息内容
      return;
    }
    MotorTask_t *motor_task = MotorTask_GetTask();
    // 确定电机方向
    int8_t direction = (data[0] & (1 << 5)) ? -1 : 1;
    uint8_t status = (data[0] & (0b11 << 6)) >> 6;
    switch (status) {
    case 0b00:
      motor_task->status = MOTOR_IDLE;
      break;
    case 0b01:
      motor_task->status = MOTOR_RUN;
      break;
    case 0b11:
      motor_task->status = MOTOR_BREAK;
      break;
    default:
      break;
    }
    motor_task->set_omega =
        direction * ((float)(data[1] << 8 | data[2]) / 10000.0f);
  }
}

/**
 * @brief 解码UART驱动报文并更新电机任务状态。
 * @param data     UART数据缓冲区指针。
 * @param data_len UART数据长度。
 *
 * 解析3字节电机控制报文，更新电机运行状态和目标角速度。
 * 数据长度不符合协议时直接返回。
 */
void CommTask_Uart_SetSpeed(const uint8_t *data, uint8_t data_len) {
  if (data_len != 3) {
    return;
  }
  MotorTask_t *motor_task = MotorTask_GetTask();
  // 确定电机方向
  int8_t direction = (data[0] & (1 << 5)) ? -1 : 1;
  uint8_t status = (data[0] & (0b11 << 6)) >> 6;
  switch (status) {
  case 0b00:
    motor_task->status = MOTOR_IDLE;
    break;
  case 0b01:
    motor_task->status = MOTOR_RUN;
    break;
  case 0b11:
    motor_task->status = MOTOR_BREAK;
    break;
  default:
    break;
  }
  motor_task->set_omega =
      direction * ((float)(data[1] << 8 | data[2]) / 10000.0f);
}

/**
 * @brief 获取当前电机模块从机ID。
 * @return 当前配置中的从机ID。
 */
uint16_t CommTask_GetID() { return comm_task.manager.stroage.storge.slave_id; }

void CommTask_UartRXCallback(void *param) {
  /*串口回调函数*/
  EF_Usart_Typedef *uart0 = comm_task.euart;
  EF_DMA_Typedef *dma = &uart0->mspm0g.dma.rx_dma;

  uart0->mspm0g.it.rx_idle.rec_size =
      uart0->mspm0g.dma.rx_size_set -
      dma->mspm0g.dma->DMACHAN[dma->mspm0g.channel].DMASZ;

  DL_UART_clearInterruptStatus(uart0->mspm0g.uart,
                               DL_UART_IIDX_RX_TIMEOUT_ERROR); // 清除标志位
  // 添加到队列中 并且添加数据长度
  comm_task.uart_queue.Add(&comm_task.uart_queue,
                           uart0->mspm0g.dma.rx_buffer_ptr);
  comm_task.uart_queue.buffer_ptr[UART_RX_DMA_BUFFER_SIZE - 2] =
      uart0->mspm0g.it.rx_idle.rec_size >> 8;
  comm_task.uart_queue.buffer_ptr[UART_RX_DMA_BUFFER_SIZE - 1] =
      uart0->mspm0g.it.rx_idle.rec_size & 0xFF;

  uart0->ReceiveDMA_IDLE(uart0, uart0->mspm0g.dma.rx_buffer_ptr,
                         uart0->mspm0g.dma.rx_size_set); // 默认再次开启DMA
}

static void CommTask_LED(CommTask_t *self) {
  KeyManagerStatus_e key_status;
  uint8_t key_touch_time;
  self->key_manager.GetResult(&self->key_manager, &key_status, &key_touch_time);
  switch (key_status) {

  case KEY_IDLE:
  case KEY_SETTING_SLAVE_ID:
  case KEY_SETTING_MASTER_ID:
    break;
  default:
    self->status_led->Show(self->status_led, comm_task.time_line);
    break;
  }
}

static void CommTaskWDT_Callback(void *item) {
  if (comm_task.is_online == true) {
    comm_task.status_led->Set(comm_task.status_led, EF_COMM_LED_TWINKLE, 10,
                              comm_task.time_line);
  }
  comm_task.is_online = false;
}
