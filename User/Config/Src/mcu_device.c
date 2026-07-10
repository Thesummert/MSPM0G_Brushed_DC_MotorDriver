#include "mcu_device.h"
#include "brushed_motor.h"
#include "bsp_mspm0g_dma.h"
#include "bsp_mspm0g_it.h"
#include "bsp_mspm0g_tim_base.h"
#include "bsp_mspm0g_usart.h"
#include "ti/devices/msp/m0p/mspm0g350x.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdlib.h>

static EF_BSP_TimerBase_t motor_control_tim;
static EF_BSP_TimerBase_t motor_encoder_tim;
static EF_BSP_TimerPWM_t motor_control_tim_pwm;
static EF_BSP_TimerQEI_t motor_encoder_tim_qei;

static EF_Usart_Typedef uart;
static EF_BSP_CAN_t can;

static EF_BrushedMotor_t motor;

_Bool EasyFrameDevice_Init() {
  EasyFrameSysTime_Init(4); // 初始化系统定时
  EF_BSP_TimerBase_Init(&motor_control_tim, PWM_0_INST, 80000000, 0xFF, 0xFFFF);

  // 初始化通信串口
  EF_BSP_Uart_Init(&uart, UART_0_INST);
  EF_BSP_Uart_InitIT(&uart, UART0_RX_CALLBACK, NONE_IT_CALLBACK,
                     EF_BSP_Uart0_RxCallback, NULL, UART0_INT_IRQn, 2);
  EF_BSP_Uart_InitNormalDMA(&uart, DMA, DMA_CH0_CHAN_ID, DMA_CH1_CHAN_ID,
                            UART_0_DMA_RX_Callback, UART_0_DMA_TX_Callback,
                            EF_BSP_Uart0_DMA_RxCallback,
                            EF_BSP_Uart0_DMA_RxCallback, DMA_INT_IRQn, 2);
  EF_BSP_Uart_Init_IDLE_IT(&uart, UART0_IDLE_RX_CALLBACK,
                           EF_BSP_Uart0_IDLE_RxCallback);
  EF_BSP_CAN_Init(&can, MCAN0_INST, false, false);
  EF_BSP_TimerPWM_Init(&motor_control_tim_pwm, &motor_control_tim, 2);
  EF_BSP_TimerBase_Init(&motor_encoder_tim, QEI_0_INST, 40000000, 0xFF, 0xFFFF);
  EF_BSP_TimerQEI_Init(&motor_encoder_tim_qei, &motor_encoder_tim);

  EF_BrushedMotor_Init(&motor, BRUSHED_MOTOR_TYPE1, false,
                       &motor_control_tim_pwm);

  return true;
}

EF_BSP_TimerPWM_t *EFDevice_Get_PWM() { return &motor_control_tim_pwm; }

EF_BSP_TimerQEI_t *EFDevice_Get_QEI() { return &motor_encoder_tim_qei; }

EF_BSP_CAN_t *EFDevice_Get_CAN() { return &can; }

EF_BrushedMotor_t *EFDevice_Get_Motor() { return &motor; }
