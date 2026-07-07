#ifndef __FREERTOS_MANAGER_H__
#define __FREERTOS_MANAGER_H__

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

void FreeRTOS_Init();
void FreeRTOS_Run();

QueueHandle_t FreeRTOS_GetQueue(uint8_t index);

#ifdef __cplusplus
}
#endif

#endif /* __FREERTOS_MANAGER_H__ */
