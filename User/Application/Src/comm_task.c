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
#define KEY_SET_CONFIRM_TIME 5.0f
#define KEY_SET_LED_ON_TIME 0.15f
#define KEY_SET_LED_OFF_TIME 0.15f
#define KEY_SET_LED_PAUSE_TIME 1.0f

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
static void CommTask_KEY(CommTask_t *self);
static void CommTask_KeySetLed(CommTask_t *self, _Bool on);
static void CommTask_KeySetLedStart(CommTask_t *self);
static void CommTask_KeySetLedShow(CommTask_t *self);
static void CommTask_KeySetSave(CommTask_t *self);
static void CommTaskWDT_Callback(void *item);

/**
 * @brief 通信任务主循环。
 *
 * 周期性处理UART设定数据，在CAN通信模式下发送电机反馈，并维护发送频率计数器。
 */
void CommTask() {
  while (1) {
    comm_task.dt = EasyFrameSysTime_GetDeltaT(&comm_task.cnt_last);
    comm_task.time_line = EasyFrameSysTime_GetTimeline_s();
    CommTask_Uart_SetValue(&comm_task);

    // 如果是CAN模式 则开始发送数据
    if (comm_task.comm_mode == 1) {
      CommTask_CAN_Trasnmit(&comm_task);
    } else {
      CommTask_Uart_Trasnmit(&comm_task);
    }
    CommTask_ReloadCounter(&comm_task);
    CommTask_KEY(&comm_task);
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
/**
 * @brief 初始化通信任务所需的外设、协议编解码器和运行参数。
 *
 * 该函数会恢复 EEPROM 中保存的模块配置；若读取失败，则使用默认主从机 ID
 * 和默认通信频率。
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

  EF_Usart_Typedef *uart0 = comm_task.euart;
  uart0->ReceiveDMA_IDLE(uart0, NULL,
                         UART_TX_DMA_BUFFER_SIZE + 2); // 开启DMA串口接受

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
/**
 * @brief 按 CAN 模式发送电机反馈报文。
 * @param self 通信任务对象指针。
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
/**
 * @brief 按 UART 模式发送电机反馈报文。
 * @param self 通信任务对象指针。
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
/**
 * @brief 根据设定频率刷新通信发送计数器。
 * @param self 通信任务对象指针。
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
/**
 * @brief 处理 UART 接收队列中的控制报文。
 * @param self 通信任务对象指针。
 */
static void CommTask_Uart_SetValue(CommTask_t *self) {
  /*串口数据解码*/
  if (self->uart_queue.length > 0) {
    // 如果队列里面有数据 则解码数据
    uint8_t frame[UART_RX_DMA_BUFFER_SIZE + 2];
    self->uart_queue.Drop(&self->uart_queue, frame);
    uint16_t master_id;
    uint16_t slave_id;
    uint8_t cmd;
    uint16_t data_len;
    uint8_t data_out[UART_TX_DMA_BUFFER_SIZE];
    uint16_t message_len =
        ((uint16_t)frame[self->uart_queue.item_size - 2] << 8) |
        frame[self->uart_queue.item_size - 1];

    if (!MotorUartSlave_Decode(frame, &master_id, &slave_id, &cmd, &data_len,
                               data_out, message_len)) {
      self->uart_queue.Drop(&self->uart_queue, NULL);
      return;
    }
    comm_task_dwt.Feed(&comm_task_dwt);
    switch (cmd) {

    case MOTOR_CMD_SET:
      /*将电机模块切换到设定模式*/
      self->set_value_mode = true; // 切换到进入设定数值模式

      break;
    case MOTOR_CMD_DRIVE:
      /*电机驱动命令 设定电机驱动数值*/
      // 只有在ID相同的时候才进行Decode处理
      if (slave_id == self->manager.stroage.storge.slave_id) {
        CommTask_Uart_SetSpeed(data_out, data_len);
      }
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
        // uint16_t try_time = 0;
        // while (
        //     (!self->euart->TransmitDMA(self->euart, self->uart_message.buffer,
        //                                self->uart_message.send_len)) &&
        //     try_time < 10) {
        //   // 尝试10次设定发送
        //   try_time++;
        // }
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

/**
 * @brief 解码CAN控制报文并更新电机任务状态。
 * @param data     CAN数据缓冲区指针。
 * @param data_len CAN数据长度。
 *
 * CAN接收计数达到阈值后切换到CAN通信模式；在CAN模式下解析3字节控制报文，更新电机运行状态和目标角速度。
 */
/**
 * @brief 解码 CAN 控制报文并更新电机任务状态。
 * @param data CAN 报文数据区。
 * @param data_len CAN 报文长度。
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
/**
 * @brief 解码 UART 驱动报文并更新电机任务状态。
 * @param data UART 报文数据区。
 * @param data_len UART 报文长度。
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
/**
 * @brief 获取当前模块的从机 ID。
 * @return 当前 EEPROM 中保存的从机 ID。
 */
uint16_t CommTask_GetID() { return comm_task.manager.stroage.storge.slave_id; }

/**
 * @brief UART 接收完成回调，用于将收到的数据帧放入解析队列。
 * @param param 回调参数，当前未使用。
 */
void CommTask_UartRXCallback(void *param) {
  /*串口回调函数*/
  EF_Usart_Typedef *uart0 = comm_task.euart;
  EF_DMA_Typedef *dma = &uart0->mspm0g.dma.rx_dma;

  uart0->mspm0g.it.rx_idle.rec_size =
      uart0->mspm0g.dma.rx_size_set -
      dma->mspm0g.dma->DMACHAN[dma->mspm0g.channel].DMASZ;

  while (!DL_UART_isRXFIFOEmpty(uart0->mspm0g.uart) &&
         uart0->mspm0g.it.rx_idle.rec_size <
             (uart0->mspm0g.dma.rx_size_set - 2)) {
    uart0->mspm0g.dma.rx_buffer_ptr[uart0->mspm0g.it.rx_idle.rec_size] =
        DL_UART_receiveData(uart0->mspm0g.uart);
    uart0->mspm0g.it.rx_idle.rec_size++;
  }

  DL_UART_clearInterruptStatus(
      uart0->mspm0g.uart,
      DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR); // 清除标志位

  if (uart0->mspm0g.it.rx_idle.rec_size < 9) {
    uart0->ReceiveDMA_IDLE(uart0, uart0->mspm0g.dma.rx_buffer_ptr,
                           uart0->mspm0g.dma.rx_size_set);
    return;
  }

  // 先把长度写入当前DMA接收帧末尾，再入队
  uart0->mspm0g.dma.rx_buffer_ptr[uart0->mspm0g.dma.rx_size_set - 2] =
      uart0->mspm0g.it.rx_idle.rec_size >> 8;
  uart0->mspm0g.dma.rx_buffer_ptr[uart0->mspm0g.dma.rx_size_set - 1] =
      uart0->mspm0g.it.rx_idle.rec_size & 0xFF;
  comm_task.uart_queue.Add(&comm_task.uart_queue,
                           uart0->mspm0g.dma.rx_buffer_ptr);

  uart0->ReceiveDMA_IDLE(uart0, uart0->mspm0g.dma.rx_buffer_ptr,
                         uart0->mspm0g.dma.rx_size_set); // 默认再次开启DMA
}

/**
 * @brief 刷新状态指示灯，并在按键设置阶段让出控制权给专用闪烁逻辑。
 * @param self 通信任务对象指针。
 */
static void CommTask_LED(CommTask_t *self) {
  if (self->key_set_pending) {
    CommTask_KeySetLedShow(self);
    return;
  }

  if (self->key_manager.status == KEY_IDLE) {
    self->status_led->Show(self->status_led, comm_task.time_line);
  }
}

/**
 * @brief 控制按键设置阶段的 LED 亮灭。
 * @param self 通信任务对象指针。
 * @param on true 表示点亮，false 表示熄灭。
 */
static void CommTask_KeySetLed(CommTask_t *self, _Bool on) {
  if (on) {
    if (self->status_led->off_voltage == 0) {
      self->status_led->gpio.SetHigh(&self->status_led->gpio);
    } else {
      self->status_led->gpio.SetLow(&self->status_led->gpio);
    }
  } else {
    if (self->status_led->off_voltage == 0) {
      self->status_led->gpio.SetLow(&self->status_led->gpio);
    } else {
      self->status_led->gpio.SetHigh(&self->status_led->gpio);
    }
  }
}

/**
 * @brief 初始化按键设置阶段的 LED 闪烁序列。
 * @param self 通信任务对象指针。
 */
static void CommTask_KeySetLedStart(CommTask_t *self) {
  self->key_led_blink_count = 0;
  self->key_led_on = true;
  self->key_led_next_time = self->time_line + KEY_SET_LED_ON_TIME;
  CommTask_KeySetLed(self, true);
}

/**
 * @brief 按当前按键次数循环驱动 LED 闪烁。
 * @param self 通信任务对象指针。
 */
static void CommTask_KeySetLedShow(CommTask_t *self) {
  if (self->time_line < self->key_led_next_time) {
    return;
  }

  if (self->key_led_on) {
    self->key_led_on = false;
    self->key_led_blink_count++;
    CommTask_KeySetLed(self, false);
    self->key_led_next_time = self->time_line + KEY_SET_LED_OFF_TIME;
    return;
  }

  if (self->key_led_blink_count >= self->key_set_touch_time) {
    self->key_led_blink_count = 0;
    self->key_led_next_time = self->time_line + KEY_SET_LED_PAUSE_TIME;
    return;
  }

  self->key_led_on = true;
  CommTask_KeySetLed(self, true);
  self->key_led_next_time = self->time_line + KEY_SET_LED_ON_TIME;
}

/**
 * @brief 将待确认的主从机 ID 写入 EEPROM 并重启系统。
 * @param self 通信任务对象指针。
 */
static void CommTask_KeySetSave(CommTask_t *self) {
  switch (self->key_set_status) {
  case KEY_SETTING_SLAVE_ID:
    self->manager.stroage.storge.slave_id =
        MOTOR_MODULE_DEFAULT_ID + self->key_set_touch_time;
    break;
  case KEY_SETTING_MASTER_ID:
    self->manager.stroage.storge.master_id =
        MOTOR_MODULE_DEFAULT_MASTER_ID + self->key_set_touch_time;
    break;
  default:
    self->key_set_pending = false;
    return;
  }

  while (!self->manager.Write(&self->manager)) {
    self->manager.eeprom->i2c.Reset(&self->manager.eeprom->i2c, true);
    EasyFrameSysTime_Delay(0.2);
  }
  NVIC_SystemReset();
}

/**
 * @brief 软件看门狗超时回调，用于标记通信离线并触发离线闪烁。
 * @param item 回调参数，当前未使用。
 */
static void CommTaskWDT_Callback(void *item) {
  if (comm_task.is_online == true) {
    comm_task.status_led->Set(comm_task.status_led, EF_COMM_LED_TWINKLE, 10,
                              comm_task.time_line);
  }
  comm_task.is_online = false;
}

/**
 * @brief 轮询按键输入并处理 ID 设定状态。
 * @param self 通信任务对象指针。
 */
static void CommTask_KEY(CommTask_t *self) {
  self->key_manager.Scan(&self->key_manager, self->dt * 1000000.0f);

  KeyManagerStatus_e key_status;
  uint8_t key_touch_time;

  if (self->key_manager.GetResult(&self->key_manager, &key_status,
                                  &key_touch_time)) {

    switch (key_status) {

    case KEY_IDLE:
      break;
    case KEY_SETTING_SLAVE_ID:
      if (key_touch_time > 0 && key_touch_time < 20) {
        self->key_set_pending = true;
        self->key_set_status = key_status;
        self->key_set_touch_time = key_touch_time;
        self->key_set_timer = self->time_line + KEY_SET_CONFIRM_TIME;
        CommTask_KeySetLedStart(self);
      }
      break;
    case KEY_SETTING_MASTER_ID:
      if (key_touch_time > 0 && key_touch_time < 20) {
        self->key_set_pending = true;
        self->key_set_status = key_status;
        self->key_set_touch_time = key_touch_time;
        self->key_set_timer = self->time_line + KEY_SET_CONFIRM_TIME;
        CommTask_KeySetLedStart(self);
      }
      break;
    default:
      break;
    }
  } else if (self->key_set_pending && self->time_line >= self->key_set_timer) {
    CommTask_KeySetSave(self);
  }
}
