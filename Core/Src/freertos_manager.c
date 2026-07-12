#include "freertos_manager.h"
#include "FreeRTOS.h"
#include "cmsis_gcc.h"
#include "ti/driverlib/m0p/dl_systick.h"
#include "mcu_config.h"
#include <string.h>
#include "queue.h"

__WEAK void MotorTask();
__WEAK void CommTask();
__WEAK void DetectTask();

static QueueHandle_t queues[FREE_RTOS_MAX_QUEUE];

/**
 * @brief 初始化FreeRTOS任务。
 */
void FreeRTOS_Init() {
    memset(queues, 0, FREE_RTOS_MAX_QUEUE * sizeof(QueueHandle_t));
    xTaskCreate(MotorTask, "Motor_Task", 256, NULL, 2, NULL);
    xTaskCreate(CommTask, "Comm_Task", 128, NULL, 2, NULL);
    xTaskCreate(DetectTask, "Detect_Task", 128, NULL, 2, NULL);

}

/**
 * @brief 启动FreeRTOS调度器。
 *
 * 此函数调用vTaskStartScheduler()以启动任务调度。
 */
void FreeRTOS_Run() {
    // DL_SYSTICK_enable();
    DL_SYSTICK_config(80000);
    vTaskStartScheduler();
}

QueueHandle_t FreeRTOS_GetQueue(uint8_t index) { return queues[index]; }

/*默认任务行为*/
__WEAK void MotorTask() {
    while (1) {
        vTaskDelay(1000);
    }
}
__WEAK void CommTask() {
    while (1) {
        vTaskDelay(1000);
    }
}
__WEAK void DetectTask() {
    while (1) {
        vTaskDelay(1000);
    }
}
