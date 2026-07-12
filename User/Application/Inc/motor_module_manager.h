#ifndef __MOTOR_MODULE_MANAGER_H__
#define __MOTOR_MODULE_MANAGER_H__

#include "at24cxx.h"
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_PID_KP 0.0f
#define MOTOR_PID_KI 0.0f
#define MOTOR_PID_KD 0.0f
#define MOTOR_PID_MAX_OUT 0.0f
#define MOTOR_PID_INTEGRAL_LIMIT 0.0f
#define MOTOR_PID_DEADBAND 0.0f
#define MOTOR_PID_LPF 0.0f

#include "mcu_device.h"

/*电机模块回报频率*/
typedef enum {
  MOTOR_MODULE_FREQ_100,
  MOTOR_MODULE_FREQ_200,
  MOTOR_MODULE_FREQ_500,
  MOTOR_MODULE_FREQ_1000,
} MotorModuleFreq_e;

typedef struct {
  uint16_t slave_id; // 从机ID
  uint16_t master_id; // 主机ID
  // PID
  float kp;
  float ki;
  float kd;
  float maxout;
  float integral_limit;
  float deadband;
  float lpf;
  // 电机设定
  float radio;
  uint8_t multiplier;
  uint32_t ppr;
  uint8_t freq;
  // 回报率

  // CRC
  uint16_t crc;
} MotorModuleStroageData_t;

/*电机存储数据联合体*/
typedef union {
  MotorModuleStroageData_t storge;
  uint8_t datas[sizeof(MotorModuleStroageData_t)];
} MotorModuleData_u;

/*电机数据管理器结构体*/
typedef struct MotorManager_t {
  EF_Device_AT24CXX_t *eeprom; // EEPROM数据存储
  MotorModuleData_u stroage;   // 数据存储结构体
  _Bool write_flag;            // 写入完成标志位

  _Bool (*Write)(struct MotorManager_t *self);
  _Bool (*Read)(struct MotorManager_t *self, uint8_t try_time);
} MotorManager_t;

_Bool MotorManager_Init(MotorManager_t *self, EF_Device_AT24CXX_t *eeprom);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_MODULE_MANAGER_H__ */
