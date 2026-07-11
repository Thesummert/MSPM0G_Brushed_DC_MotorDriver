#ifndef __MOTOR_CAN_CONTROL_H__
#define __MOTOR_CAN_CONTROL_H__

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#include "mcu_config.h"
#include "stdint.h"

/**
 * @note 电机速度采用16位整数传输 前12位为整数 后4位代表小数
 * */

/*电机CAN状态控制命令*/
typedef enum {
  MOTOR_CAN_IDLE = 0b00,
  MOTOR_CAN_RUN = 0b01,
  MOTOR_CAN_BREAK = 0b11,
} MotorCanStatusCMD_e;

/*电机CAN方向控制命令*/
typedef enum {
  MOTOR_CAN_FORWARD = 0b0,
  MOTOR_CAN_BACKWARD = 0b1,
} MotorCanDirCMD_e;

// 关闭字节对齐
#pragma pack(1)
#if BRUSHED_MOTOR_IS_MASTER == 1
/*主机到从机结构体*/
typedef struct {
  uint16_t slave_id; // 从机地址

  union {
    struct {
      uint8_t
          cmd; // 电机控制命令 此处规定[7:6]为电机驱动状态 [5:5]为电机方向设定
      uint16_t value; // 电机设定数值
    } set;
    uint8_t datas[3];
  } cmd; // 数据发送union
} MotorCan_Master2Slave_t;
#else

/*从机到主机结构体*/
typedef struct MotorCan_Slave2Master_t {
  uint16_t slave_id; // 设定的从机地址
  union {
    struct {
      uint8_t status; // 驱动器状态 [7:7]被设计用来表示速度方向
      uint16_t value; // 速度值
    } cmd;
    uint8_t datas[3];
  } feedback; // 电机反馈值

  // 函数指针
  void (*Encoder)(struct MotorCan_Slave2Master_t *self, uint8_t status,
                  float speed);
} MotorCan_Slave2Master_t;

_Bool MotorCanSlave_Init(MotorCan_Slave2Master_t *self, uint16_t id);

#endif

#pragma pack()
#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CAN_CONTROL_H__ */
