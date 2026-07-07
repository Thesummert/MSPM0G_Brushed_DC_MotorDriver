#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

#include "ti/devices/msp/peripherals/hw_mcan.h"
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#include "ti_msp_dl_config.h"

/*CAN发送数据长度*/
typedef enum {
  EF_CAN_DLC_NUM1,
  EF_CAN_DLC_NUM2,
  EF_CAN_DLC_NUM3,
  EF_CAN_DLC_NUM4,
  EF_CAN_DLC_NUM5,
  EF_CAN_DLC_NUM6,
  EF_CAN_DLC_NUM7,
  EF_CAN_DLC_NUM8,
  // CAN FD extended DLC (length ≠ DLC)
  EF_CAN_DLC_12, // DLC=9  → 12 bytes
  EF_CAN_DLC_16, // DLC=10 → 16 bytes
  EF_CAN_DLC_20, // DLC=11 → 20 bytes
  EF_CAN_DLC_24, // DLC=12 → 24 bytes
  EF_CAN_DLC_32, // DLC=13 → 32 bytes
  EF_CAN_DLC_48, // DLC=14 → 48 bytes
  EF_CAN_DLC_64, // DLC=15 → 64 bytes
} EF_CAN_DLC_e;

/*中断万能传参用*/
typedef struct {
  MCAN_Regs *can;
  uint8_t canid;
} EF_CAN_IT_CallbackPass_t;

typedef struct EF_BSP_CAN_t {
  _Bool is_inited;
  _Bool is_fdcan;
  _Bool is_baud_switch;

  uint8_t tx_fifo_num;
  uint8_t rx_fifo_num;
  EF_CAN_DLC_e max_dlc_len;
  uint8_t rx0_dlc_max_size;
  uint8_t rx1_dlc_max_size;

  _Bool (*TransmitFIFO)(struct EF_BSP_CAN_t *self, uint32_t id, uint8_t *data,
                        uint16_t len, _Bool is_ext_id);
  struct {
    MCAN_Regs *can;
  } mspm0g;
} EF_BSP_CAN_t;

_Bool EF_BSP_CAN_Init(EF_BSP_CAN_t *self, MCAN_Regs *can, _Bool is_fdcan,
                      _Bool is_baud_switch);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_CAN_H__ */
