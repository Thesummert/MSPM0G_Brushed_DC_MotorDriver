#include "motor_module_manager.h"
#include "crc_ref.h"
#include "mcu_config.h"
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

static _Bool Write(MotorManager_t *self);
static _Bool Read(MotorManager_t *self);

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

static _Bool Read(MotorManager_t *self)
{
  if (self == NULL) {
    RTT_Print(0, "Null pointer error in motor manager read \r\n");
    return false;
  }
  self->eeprom->ReadData(self->eeprom, MOTOR_MODULE_STROAGE_DATA_ADDR, self->stroage.datas, sizeof(MotorModuleData_u));
  if (!Verify_CRC16_Check_Sum(self->stroage.datas, sizeof(MotorModuleData_u))) {
      return false;
  }
  return true;
}
