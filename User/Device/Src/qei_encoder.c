#include "qei_encoder.h"
#include "mcu_config.h"
#include <stdbool.h>
#include <string.h>
#include <sys/reent.h>

_Bool EF_Device_QEI_Encoder_Init(EF_QEI_Encoder_t *self, EF_BSP_TimerQEI_t *qei,
                                 uint32_t ppr, uint8_t multiplier, float radio)
{
    if (self == NULL || qei == NULL) {
        RTT_Print(0, "Null pointer error in qei encoder init \r\n");
        return false;
    }

    memset(self, 0, sizeof(EF_QEI_Encoder_t));
    self->multiplier = multiplier;
    self->radio = radio;
    self->pulse_per_rev = ppr;
    self->qei = qei;

    self->is_inited = true;

    return true;
}


