#include "mcu_device.h"
#include "at24cxx.h"
#include "brushed_motor.h"
#include "bsp_can.h"
#include "bsp_mspm0g_dma.h"
#include "bsp_mspm0g_gpio.h"
#include "bsp_mspm0g_i2c.h"
#include "bsp_mspm0g_it.h"
#include "bsp_mspm0g_tim_base.h"
#include "bsp_mspm0g_usart.h"
#include "comm_key.h"
#include "comm_led.h"
#include "comm_task.h"
#include "detect_task.h"
#include "mcu_config.h"
#include "qei_encoder.h"
#include "ti/devices/msp/m0p/mspm0g350x.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdlib.h>

static EF_BSP_TimerBase_t motor_control_tim;
static EF_BSP_TimerBase_t motor_encoder_tim;
static EF_BSP_TimerPWM_t motor_control_tim_pwm;
static EF_BSP_TimerQEI_t motor_encoder_tim_qei;

static EasyFrame_GPIO_Typedef_t key_gpio;

static EF_Usart_Typedef uart;
static EF_BSP_CAN_t can;

static EF_BrushedMotor_t motor;
static EF_Device_AT24CXX_t at24;
static EF_Device_Comm_LED_t status_led;
static EF_Device_CommKey_t control_key;
static EF_QEI_Encoder_t qei_encoder;

_Bool EasyFrameDevice_Init() {
  EasyFrameSysTime_Init(4); // 初始化系统定时
  EasyFrame_GPIO_Init(&key_gpio, LED_PORT_PORT, KEY_PORT_KEY_PIN_PIN);
  EF_CommKey_Init(&control_key, &key_gpio, 1);
  EF_Device_Comm_LED_Init(&status_led, LED_PORT_PORT, LED_PORT_LED_PIN_PIN, 1);

  // 初始化看门狗
  EF_App_SoftWDT_Group_Init(EF_App_SoftWDT_Group_Get(0));
  EF_BSP_TimerBase_Init(&motor_control_tim, PWM_0_INST, 80000000, 0xFF, 0xFFFF);

  // 初始化通信串口
  EF_BSP_Uart_Init(&uart, UART_0_INST);
  EF_BSP_Uart_InitIT(&uart, UART0_RX_CALLBACK, NONE_IT_CALLBACK,
                     EF_BSP_Uart0_RxCallback, NULL, UART0_INT_IRQn, 2);
  EF_BSP_Uart_InitNormalDMA(&uart, DMA, DMA_CH0_CHAN_ID, DMA_CH1_CHAN_ID,
                            UART_0_DMA_RX_Callback, UART_0_DMA_TX_Callback,
                            EF_BSP_Uart0_DMA_RxCallback,
                            EF_BSP_Uart0_DMA_RxCallback, DMA_INT_IRQn, 2);
  EF_BSP_Uart_Init_IDLE_IT(
      &uart, UART0_IDLE_RX_CALLBACK,
      CommTask_UartRXCallback); // 此处使用通信任务设计的回调函数
  // 初始化CAN通信
  EF_BSP_CAN_Init(&can, MCAN0_INST, false, false);
  EF_BSP_CAN_InitIT(&can, CANFD0_INT_IRQn, 2, MCAN0_RXFIFO_0_Callback,
                    MCAN0_RXFIFO_1_Callback);

  // 初始化eeprom
  EasyFrame_GPIO_Typedef_t scl, sda;
  EasyFrame_GPIO_Init(&scl, GPIOA, DL_GPIO_PIN_0);
  EasyFrame_GPIO_Init(&sda, GPIOA, DL_GPIO_PIN_1);
  EasyFrame_GPIO_InitIOMux(&sda, GPIO_I2C_0_IOMUX_SDA);
  EasyFrame_GPIO_InitIOMux(&scl, GPIO_I2C_0_IOMUX_SCL);

  EF_Device_AT24CXX_Init(&at24, EF_AT24_TYPE_C2, 0b1010000, I2C_0_INST,
                         80000000);
  EasyFrame_I2C_InitGPIO(&at24.i2c, sda, scl);

  EF_BSP_TimerPWM_Init(&motor_control_tim_pwm, &motor_control_tim, 2);
  EF_BSP_TimerBase_Init(&motor_encoder_tim, QEI_0_INST, 40000000, 0xFF, 0xFFFF);
  EF_BSP_TimerQEI_Init(&motor_encoder_tim_qei, &motor_encoder_tim);
  motor_encoder_tim_qei.base_load = 0x4000; // 设定装载值

  // 初始化电机
  EF_BrushedMotor_Init(&motor, BRUSHED_MOTOR_TYPE1, false,
                       &motor_control_tim_pwm);
  EF_Device_QEI_Encoder_Init(
      &qei_encoder, &motor_encoder_tim_qei, MOTOR_MODULE_DEFAULT_PPR,
      MOTOR_MODULE_DEFAULT_MULTIPLY, MOTOR_MODULE_DEFAULT_RADIO);
  EF_BrushedMotor_InitQEI(&motor, &qei_encoder);

  return true;
}

EF_BSP_TimerPWM_t *EFDevice_Get_PWM() { return &motor_control_tim_pwm; }

EF_BSP_TimerQEI_t *EFDevice_Get_QEI() { return &motor_encoder_tim_qei; }

EF_BSP_CAN_t *EFDevice_Get_CAN() { return &can; }

EF_Usart_Typedef *EFDevice_Get_Uart() { return &uart; }

EF_BrushedMotor_t *EFDevice_Get_Motor() { return &motor; }

EF_Device_AT24CXX_t *EFDevice_Get_EEPROM() { return &at24; }

EF_Device_Comm_LED_t *EFDevice_Get_LED() { return &status_led; }

EF_Device_CommKey_t *EFDevice_Get_Key() { return &control_key; }
