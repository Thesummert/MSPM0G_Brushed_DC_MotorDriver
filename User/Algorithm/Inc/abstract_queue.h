#ifndef __ABSTRACT_QUEUE_H__
#define __ABSTRACT_QUEUE_H__

#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

typedef struct EF_Algorithm_Queue_t {
    uint32_t buffer_size;  // 数据缓冲区总内存大小
    uint8_t *buffer_ptr;   // 数据缓冲区指针
    uint16_t item_size;    // 单个ITEM内存大小
    uint16_t queue_size;   // 队列大小

    int32_t length;          // 现有数据长度
    uint16_t header_index;   // 数据头指针
    uint16_t pointer_index;  // 队列当前数据指针

    _Bool is_inited;  // 是否完成初始化
    _Bool (*Add)(struct EF_Algorithm_Queue_t *self, void *data);
    _Bool (*Drop)(struct EF_Algorithm_Queue_t *self, void *data);
} EF_Algorithm_Queue_t;

_Bool EF_Algorithm_Queue_Init(EF_Algorithm_Queue_t *self, uint16_t queue_size, uint16_t item_size,
                              uint8_t *buffer_ptr);

#ifdef __cplusplus
}
#endif

#endif /* __ABSTRACT_QUEUE_H__ */
