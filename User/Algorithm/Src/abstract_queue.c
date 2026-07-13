#include "abstract_queue.h"
#include <stdbool.h>
#include <stdint.h>
#include <sys/reent.h>
#include "mcu_config.h"
#include "string.h"

static _Bool Add(EF_Algorithm_Queue_t *self, void *data);
static _Bool Drop(EF_Algorithm_Queue_t *self, void *data);

/**
 * @brief 初始化队列
 * @param self      队列对象指针
 * @param queue_size 队列容量（最大元素个数）
 * @param item_size  单个元素大小（字节）
 * @param buffer_ptr 数据缓冲区指针
 * @retval true  初始化成功
 * @retval false 初始化失败（传入空指针）
 */
_Bool EF_Algorithm_Queue_Init(EF_Algorithm_Queue_t *self, uint16_t queue_size, uint16_t item_size,
                              uint8_t *buffer_ptr) {
    if (self == NULL || buffer_ptr == NULL) {
        RTT_Print(0, "Null pointer error in queue init \r\n");
        return false;
    }
    memset(self, 0, sizeof(EF_Algorithm_Queue_t));
    self->header_index = -1;
    self->queue_size = queue_size;
    self->item_size = item_size;
    self->buffer_ptr = buffer_ptr;

    self->Add = Add;
    self->Drop = Drop;

    self->is_inited = true;

    return true;
}

/**
 * @brief 向队列尾部添加一个元素
 * @param self 队列对象指针
 * @param data 待添加的数据指针
 * @retval true  添加成功
 * @retval false 添加失败（队列已满或未初始化）
 */
static _Bool Add(EF_Algorithm_Queue_t *self, void *data) {
    if (self == NULL) {
        return false;
    }
    if (self->is_inited == false) {
        return false;
    }
    if (self->length >= self->queue_size) {
        // 超出队列的最大大小
        // TODO 更改为去除队列中第一项数据
        self->Drop(self, NULL);
        return false;
    }
    self->length++;
    if (++self->header_index >= self->queue_size) {
        self->header_index = 0;
    }
    // 将数据搬运到队列中
    memcpy(self->buffer_ptr + self->header_index * self->item_size, data, self->item_size);

    return true;
}

/**
 * @brief 从队列头部取出一个元素
 * @param self 队列对象指针
 * @param data 用于接收出队数据的缓冲区指针
 * @retval true  出队成功
 * @retval false 出队失败（队列为空或传入空指针）
 */
static _Bool Drop(EF_Algorithm_Queue_t *self, void *data) {
    if (self == NULL) {
        return false;
    }
    if (self->is_inited == false) {
        return false;
    }
    if (self->length == 0) {
        // 队列中没有内容
        return false;
    }
    if (data != NULL) {
        // 搬运数据
        // 只有在data不为空指针的时候拷贝 其余时刻直接删除
        memcpy(data, self->buffer_ptr + self->pointer_index * self->item_size, self->item_size);
    }
    self->length--;
    self->pointer_index++;
    if (++self->pointer_index >= self->queue_size) {
        self->pointer_index = 0;
    }

    return true;
}
