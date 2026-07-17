#include "detect_task.h"
#include "FreeRTOS.h"
#include "SEGGER_RTT.h"
#include "mcu_config.h"
#include "stdbool.h"
#include <assert.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

static EF_App_SoftWDT_Group_t wdt_group;

/**
 * @brief 离线检测任务入口
 * @note  任务循环以一定周期运行，实际检测逻辑由内部软件看门狗组完成
 */
void DetectTask() {
  while (1) {
    wdt_group.Check(&wdt_group);
    vTaskDelay(10);
  }
}

static _Bool Feed(EF_App_SoftWDT_t *self);
static _Bool IsOffline(EF_App_SoftWDT_t *self);
static _Bool Check(EF_App_SoftWDT_Group_t *self);

/**
 * @brief          初始化单个软件看门狗实例
 * @param[in] self 看门狗实例指针
 * @param[in] id   看门狗名称 ID（用于日志输出标识）
 * @param[in] id_len ID 长度
 * @param[in] load 超时计数值（减到 0 视为离线）
 * @param[in] Callback 离线回调函数指针
 * @param[in] item 回调参数
 * @retval true    初始化成功
 * @retval false   初始化失败（self 为空）
 */
_Bool EF_SoftWDT_Init(EF_App_SoftWDT_t *self, uint8_t *id, uint8_t id_len,
                      uint32_t load, void *Callback, void *item) {
  if (self == NULL) {
    RTT_Print(0, "Nullpointer error in wdt init \r\n");
    return false;
  }
  memset(self, 0, sizeof(EF_App_SoftWDT_t));
  self->load = load;
  self->cnt = load;
  self->is_online = true;
  self->Callback = Callback;
  self->item = item;
  self->Feed = Feed;
  if (id_len > SOFT_WDT_ID_MAX_LEN) {
    // 超出最长ID 仅保留前面最长长度
    memcpy(self->wdt_name, id, SOFT_WDT_ID_MAX_LEN - 1);
    self->wdt_name[SOFT_WDT_ID_MAX_LEN - 1] = 0;
  }

  return true;
}

/**
 * @brief          检查单个看门狗是否离线（计数器递减并判断）
 * @param[in] self 看门狗实例指针
 * @retval true    仍在在线状态（cnt > 0）
 * @retval false   已离线（cnt == 0），会触发回调
 */
static _Bool IsOffline(EF_App_SoftWDT_t *self) {
  /*检查是否在线*/
  if (self == NULL) {
    RTT_Print(0, "Nullpointer error in wdt check offline \r\n");
    return false;
  }
  if (--self->cnt == 0) {
    RTT_Print(0, "wdt:%s offline! \r\n", self->wdt_name);
    if (self->Callback != NULL) {
      self->Callback(self->item);
    }
    return false;
  }
  return true;
}

/**
 * @brief          喂狗，重置看门狗计数器
 * @param[in] self 看门狗实例指针
 * @retval true    喂狗成功
 * @retval false   喂狗失败（self 为空）
 */
static _Bool Feed(EF_App_SoftWDT_t *self) {
  if (self == NULL) {
    RTT_Print(0, "Nullpointer error in wdt check feed\r\n");
    return false;
  }
  self->cnt = self->load;
  self->is_online = true;
  return true;
}

/**
 * @brief          初始化软件看门狗组
 * @param[in] self 看门狗组指针
 * @retval true    初始化成功
 * @retval false   初始化失败（self 为空）
 */
_Bool EF_App_SoftWDT_Group_Init(EF_App_SoftWDT_Group_t *self) {
  if (self == NULL) {
    RTT_Print(0, "Nullpointer error in wdt group init \r\n");
    return false;
  }
  memset(self, 0, sizeof(EF_App_SoftWDT_Group_t));
  self->Check = Check;
  return true;
}
/**
 * @brief          向看门狗组中添加一个看门狗实例
 * @param[in] self 看门狗组指针
 * @param[in] wdt  待添加的看门狗实例指针
 * @retval true    添加成功
 * @retval false   添加失败（指针为空或组已满）
 */
_Bool EF_App_SoftWDT_Group_Add(EF_App_SoftWDT_Group_t *self,
                               EF_App_SoftWDT_t *wdt) {
  if (self == NULL || wdt == NULL) {
    RTT_Print(0, "Nullpointer error in wdt group add \r\n");
    return false;
  }
  if (self->num >= SOFT_WDT_ID_MAX_LEN) {
    RTT_Print(0, "Out of soft wdt max len \r\n");
    return false;
  }
  self->wdts[self->num] = wdt;
  self->num++;

  return true;
}

/**
 * @brief          遍历组内所有看门狗，检查各实例的在线状态
 * @param[in] self 看门狗组指针
 * @retval true    检查完成（即使有离线实例也返回 true）
 * @retval false   检查失败（self 为空）
 */
static _Bool Check(EF_App_SoftWDT_Group_t *self) {
  if (self == NULL) {
    RTT_Print(0, "Nullpointer error in wdt group check \r\n");
    return false;
  }
  uint16_t online = 0;
  for (uint16_t i = 0; i < self->num; i++) {
    if (IsOffline(self->wdts[i])) {
      online++;
    }
  }
  self->online_num = online;
  return true;
}

/**
 * @brief          根据 ID 获取软件看门狗组指针
 * @param[in] id   组 ID（当前仅支持 0）
 * @return         看门狗组指针，无效 ID 返回 NULL
 */
EF_App_SoftWDT_Group_t *EF_App_SoftWDT_Group_Get(uint16_t id) {
  switch (id) {
  case 0:
    return &wdt_group;
    break;
  default:
    break;
  }
  return  NULL;
}
