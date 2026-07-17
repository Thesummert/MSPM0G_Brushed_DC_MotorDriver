/**
 *
 * @brief 这是通用普通LED驱动
 *
 *
 */
#include "comm_led.h"
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "bsp_mspm0g_gpio.h"
#include "mcu_config.h"

static _Bool Set(EF_Device_Comm_LED_t *self, EF_COMM_LED_State_e state, float freq, float time_line);
static _Bool Show(EF_Device_Comm_LED_t *self, float time_line);

/**
 * @brief 初始化通用LED设备对象
 * @param[in,out] self         LED设备对象指针
 * @param[in]     gpio         LED连接的GPIO外设基地址
 * @param[in]     pin          LED对应引脚号
 * @param[in]     off_voltage  熄灭时的电平，false表示低电平熄灭，true表示高电平熄灭
 * @return _Bool 初始化成功返回true，参数非法返回false
 */
_Bool EF_Device_Comm_LED_Init(EF_Device_Comm_LED_t *self, GPIO_Regs *gpio, uint32_t pin,
                              _Bool off_voltage) {
    if (self == NULL || gpio == NULL) {
        RTT_Print(0, "Null pointer error happened in comm_led init \r\n");
        return false;
    }
    memset(self, 0, sizeof(EF_Device_Comm_LED_t));
    EasyFrame_GPIO_Init(&self->gpio, gpio, pin);  // 初始化对应的GPIO
    // 设定熄灭电平
    self->off_voltage = off_voltage;
    if (off_voltage == 0) {
        self->gpio.SetLow(&self->gpio);
    } else {
        self->gpio.SetHigh(&self->gpio);
    }
    // 设定函数指针
    self->Set = Set;
    self->Show = Show;
    self->is_inited = true;
    return true;
}

/**
 * @brief 设置通用LED的工作状态
 * @param[in,out] self       LED设备对象指针
 * @param[in]     state      目标状态：关闭、常亮或闪烁
 * @param[in]     freq       闪烁频率，只有在闪烁状态下有效
 * @param[in]     time_line  当前时间线，用于计算闪烁时序
 * @return _Bool 设置成功返回true，参数非法或无需更新返回false
 */
static _Bool Set(EF_Device_Comm_LED_t *self, EF_COMM_LED_State_e state, float freq, float time_line) {
    if (self == NULL) {
        RTT_Print(0, "Null pointer error happened in comm_led set\r\n");
        return false;
    }
    if (self->is_inited == false) {
        RTT_Print(0, "Comm led not inited \r\n");
        return false;
    }
    // 如果设定值相同 则不重置
    if (self->state == state && self->twinkle_data.freq == freq) {
        self->is_set = false;
        return false;
    }
    self->state = EF_COMM_LED_TWINKLE;
    if (state == EF_COMM_LED_TWINKLE) {
        self->twinkle_data.freq = freq;
        self->twinkle_data.interval_time = 1.0 / freq;  // 计算间隔时间
        self->twinkle_data.time_line_start = time_line;
        self->twinkle_data.time_line_end = time_line + self->twinkle_data.interval_time;
    }
    self->is_set = true;
    return true;
}

/**
 * @brief 刷新通用LED输出状态
 * @param[in,out] self       LED设备对象指针
 * @param[in]     time_line  当前时间线，用于处理闪烁翻转
 * @return _Bool 刷新成功返回true，参数非法或对象未初始化返回false
 */
static _Bool Show(EF_Device_Comm_LED_t *self, float time_line) {
    if (self == NULL) {
        RTT_Print(0, "Null pointer error happened in comm_led show\r\n");
        return false;
    }
    if (self->is_inited == false) {
        RTT_Print(0, "Comm led not inited \r\n");
        return false;
    }
    // 只有在要求更新的状态下才更新
    if (self->is_set == true && self->state != EF_COMM_LED_TWINKLE) {
        if (self->off_voltage == 0) {
            self->gpio.SetLow(&self->gpio);
        } else {
            self->gpio.SetHigh(&self->gpio);
        }
        self->is_set = false;
        return true;
    }
    // 闪烁部分单独处理
    // 进入第一次闪烁先亮灯
    if (self->is_set == true) {
        if (self->off_voltage == 0) {
            self->gpio.SetLow(&self->gpio);
        } else {
            self->gpio.SetHigh(&self->gpio);
        }
        self->is_set = false;
    }
    if (time_line > self->twinkle_data.time_line_end) {
        // 到时间翻转led
        self->gpio.Toggle(&self->gpio);
        self->twinkle_data.time_line_start = time_line;
        self->twinkle_data.time_line_end = time_line + self->twinkle_data.interval_time;
    }
    return true;
}
