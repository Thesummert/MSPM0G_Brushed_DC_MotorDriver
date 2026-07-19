#include "brushed_motor.h"
#include "SEGGER_RTT.h"
#include "mcu_config.h"
#include "ti/driverlib/dl_aes.h"
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

static _Bool SetOutput(EF_BrushedMotor_t *self, float value);
static _Bool Break(EF_BrushedMotor_t *self);
static _Bool Start(EF_BrushedMotor_t *self);
static _Bool Stop(EF_BrushedMotor_t *self);
static _Bool GetSpeed(EF_BrushedMotor_t *self, float dt, float *omega_rotor,
                      float *omega_output);

/**
 * @brief       初始化有刷电机设备
 * @param[in]   self         有刷电机对象指针
 * @param[in]   type         电机驱动类型(如DRV8870/TB6612)
 * @param[in]   need_enable  是否需要使能引脚控制
 * @param[in]   pwm          PWM控制器对象指针
 * @retval      true         初始化成功
 * @retval      false        初始化失败(空指针)
 */
_Bool EF_BrushedMotor_Init(EF_BrushedMotor_t *self, EF_BrushedMotorType_e type,
                           _Bool need_enable, EF_BSP_TimerPWM_t *pwm) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error in brushed motor init \r\n");
    return false;
  }
  memset(self, 0, sizeof(EF_BrushedMotor_t));
  self->motor_type = type;
  self->pwm = pwm;
  self->need_enable = need_enable;

  // 初始化函数指针
  self->SetOutput = SetOutput;
  self->Break = Break;
  self->Start = Start;
  self->Stop = Stop;
  self->GetSpeed = GetSpeed;
  return true;
}

/**
 * @brief       为有刷电机绑定QEI编码器
 * @param[in]   self  有刷电机对象指针
 * @param[in]   qei   QEI编码器对象指针(需先初始化)
 * @retval      true  绑定成功
 * @retval      false 绑定失败(空指针)
 */
_Bool EF_BrushedMotor_InitQEI(EF_BrushedMotor_t *self, EF_QEI_Encoder_t *qei) {
  if (self == NULL || qei == NULL) {
    RTT_Print(0, "Null pointer error in brushed motor init qei \r\n");
    return false;
  }
  self->qei = qei;
  return true;
}

/**
 * @brief       为有刷电机绑定使能引脚
 * @param[in]   self  有刷电机对象指针
 * @param[in]   gpio  使能控制GPIO对象指针
 * @retval      true  绑定成功
 * @retval      false 绑定失败(空指针)
 */
_Bool EF_BrushedMotor_InitEnable(EF_BrushedMotor_t *self,
                                 EasyFrame_GPIO_Typedef_t *gpio) {
  if (self == NULL || gpio == NULL) {
    RTT_Print(0, "Null pointer error in brushed motor init enable \r\n");
    return false;
  }
  self->enable = gpio;
  return true;
}

/**
 * @brief       为有刷电机绑定方向控制引脚
 * @param[in]   self  有刷电机对象指针
 * @param[in]   dir1  方向控制GPIO对象指针1
 * @param[in]   dir2  方向控制GPIO对象指针2
 * @retval      true  绑定成功
 * @retval      false 绑定失败(空指针)
 * @note        用于需要独立GPIO控制方向的驱动(如TB6612)
 */
_Bool EF_BrushedMotor_InitDir(EF_BrushedMotor_t *self,
                              EasyFrame_GPIO_Typedef_t *dir1,
                              EasyFrame_GPIO_Typedef_t *dir2) {
  if (self == NULL || dir1 == NULL || dir2 == NULL) {
    RTT_Print(0, "Null pointer error in brushed motor init dir \r\n");
    return false;
  }
  self->dir1 = dir1;
  self->dir2 = dir2;
  return true;
}

/**
 * @brief       设置有刷电机输出(带方向)
 * @param[in]   self   有刷电机对象指针
 * @param[in]   value  输出值,正负表示方向,绝对值为占空比比较值
 * @retval      true   设置成功
 * @retval      false  设置失败(空指针或PWM未绑定)
 */
static _Bool SetOutput(EF_BrushedMotor_t *self, float value) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error in brushed motor set value \r\n");
    return false;
  }
  if (self->pwm == NULL) {
    return false;
  }
  switch (self->motor_type) {
  case BRUSHED_MOTOR_TYPE1:
    // 需要判断方向设定数值
    if (value >= 0.0f) {
      self->pwm->SetPWM(self->pwm, 0, value);
      self->pwm->SetPWM(self->pwm, 1, 0);
    } else {
      self->pwm->SetPWM(self->pwm, 0, 0);
      self->pwm->SetPWM(self->pwm, 1, -value);
    }
    break;
  case BRUSHED_MOTOR_TYPE2:
    break;
  default:
    break;
  }

  return true;
}

/**
 * @brief       有刷电机刹车(制动)
 * @param[in]   self   有刷电机对象指针
 * @retval      true   刹车成功
 * @retval      false  刹车失败(空指针或PWM未绑定)
 * @note        通过将两路PWM同时拉高实现能耗制动
 */
static _Bool Break(EF_BrushedMotor_t *self) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error in brushed motor set break \r\n");
    return false;
  }
  if (self->pwm == NULL) {
    return false;
  }
  switch (self->motor_type) {
  case BRUSHED_MOTOR_TYPE1:
    // 都是高电平的时候是刹车
    self->pwm->SetPWM(self->pwm, 0, 0xFFFFFFFF);
    self->pwm->SetPWM(self->pwm, 1, 0xFFFFFFFF);
    break;
  case BRUSHED_MOTOR_TYPE2:
    break;
  default:
    break;
  }

  return true;
}

/**
 * @brief       启动有刷电机(PWM输出与编码器计数)
 * @param[in]   self   有刷电机对象指针
 * @retval      true   启动成功
 * @retval      false  启动失败(空指针或PWM未绑定)
 */
static _Bool Start(EF_BrushedMotor_t *self) {

  if (self == NULL) {
    RTT_Print(0, "Null pointer error in brushed motor start \r\n");
    return false;
  }
  if (self->pwm == NULL) {
    return false;
  }
  self->pwm->StartPWM(self->pwm);
  self->qei->Start(self->qei);

  return true;
}

/**
 * @brief       停止有刷电机(PWM输出与编码器计数)
 * @param[in]   self   有刷电机对象指针
 * @retval      true   停止成功
 * @retval      false  停止失败(空指针或PWM未绑定)
 */
static _Bool Stop(EF_BrushedMotor_t *self) {

  if (self == NULL) {
    RTT_Print(0, "Null pointer error in brushed motor stop \r\n");
    return false;
  }
  if (self->pwm == NULL) {
    return false;
  }
  self->pwm->StopPWM(self->pwm);
  self->qei->Stop(self->qei);

  return true;
}

/**
 * @brief       获取有刷电机转速(经由QEI编码器)
 * @param[in]   self          有刷电机对象指针
 * @param[in]   dt            采样时间间隔(秒)
 * @param[out]  omega_rotor   电机轴角速度(rad/s, 可选传NULL)
 * @param[out]  omega_output  输出轴角速度(rad/s, 可选传NULL)
 * @retval      true          获取成功
 * @retval      false         获取失败(空指针、编码器未绑定或计算失败)
 */
static _Bool GetSpeed(EF_BrushedMotor_t *self, float dt, float *omega_rotor,
                      float *omega_output) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error in brushed motor get speed \r\n");
    return false;
  }
  if (self->qei == NULL) {
    return false;
  }
  if (!self->qei->getSpeed(self->qei, dt, &self->omega_rotor,
                           &self->omega_output)) {
    return false;
  }
  if (self->is_reverse == true) {
    self->omega_rotor *= -1.0f;
    self->omega_output *= -1.0f;
  }

  if (omega_rotor != NULL) {
    *omega_rotor = self->omega_rotor;
  }
  if (omega_output != NULL) {
    *omega_output = self->omega_output;
  }

  return true;
}
