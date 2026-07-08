#ifndef __QEI_ENCODER_H__
#define __QEI_ENCODER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_time.h"
#include <stdint.h>

typedef struct {
  EF_BSP_TimerQEI_t *qei;
  uint32_t pulse_per_rev; // 每转脉冲数(PPR)
  uint8_t multiplier;     // 倍频数(通常为4)
  float dt;               // 间隔时间
  float omega;            // 角速度
};

#ifdef __cplusplus
}
#endif

#endif /* __QEI_ENCODER_H__ */
