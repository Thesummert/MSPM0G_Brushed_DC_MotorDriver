#include "motor_module_manager.h"
#include "bsp_mspm0g_tim_base.h"
#include "crc_ref.h"
#include "mcu_config.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

static _Bool Write(MotorManager_t *self);
static _Bool Read(MotorManager_t *self, uint8_t try_time);

_Bool MotorManager_Init(MotorManager_t *self, EF_Device_AT24CXX_t *eeprom) {
  if (self == NULL || eeprom == NULL) {
    RTT_Print(0, "Null pointer error in motor manager init \r\n");
    return false;
  }
  memset(self, 0, sizeof(MotorManager_t));
  self->eeprom = eeprom;

  self->Write = Write;
  self->Read = Read;

  return true;
}

static _Bool Write(MotorManager_t *self) {
  /*
   * @TODO 添加读取校验
   * */
  if (self == NULL) {
    RTT_Print(0, "Null pointer error in motor manager write \r\n");
    return false;
  }
  self->write_flag = false;
  // 添加CRC
  Append_CRC16_Check_Sum(self->stroage.datas, sizeof(MotorModuleData_u));
  for (uint8_t i = 0; i < 3; i++) {
    if (self->eeprom->WriteData(self->eeprom, MOTOR_MODULE_STROAGE_DATA_ADDR,
                                self->stroage.datas,
                                sizeof(MotorModuleData_u)) == true) {
      self->write_flag = true;
      return true;
    }
  }
  return false;
}

static _Bool Read(MotorManager_t *self, uint8_t try_time) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error in motor manager read \r\n");
    return false;
  }
  _Bool flag = false;
  uint8_t times = 0;
  do {
    self->eeprom->ReadData(self->eeprom, MOTOR_MODULE_STROAGE_DATA_ADDR,
                           self->stroage.datas, sizeof(MotorModuleData_u));
    flag =
        Verify_CRC16_Check_Sum(self->stroage.datas, sizeof(MotorModuleData_u));
    if (flag == false) {
      // 如果读取失败 则重置I2C总线
      self->eeprom->i2c.Reset(&self->eeprom->i2c, true);
      EasyFrameSysTime_Delay(0.05f);
    } else {
      return true;
    }
  } while (flag == false && times < try_time);
  return false;
}
