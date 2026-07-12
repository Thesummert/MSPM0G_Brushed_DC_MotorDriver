#include "comm_task.h"

/**
 * @note 通信任务 同时也负责管理控制
 * */

void CommTask() {
    while (1) {
        vTaskDelay(1000);
    }
}
