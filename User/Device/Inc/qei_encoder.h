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
  float radio;            // 减速比
  _Bool is_inited;        // 是否初始化
} EF_QEI_Encoder_t;

_Bool EF_Device_QEI_Encoder_Init(EF_QEI_Encoder_t *self, EF_BSP_TimerQEI_t *qei,
                                 uint32_t ppr, uint8_t multiplier, float radio);

#ifdef __cplusplus
}
#endif

#endif /* __QEI_ENCODER_H__ */
