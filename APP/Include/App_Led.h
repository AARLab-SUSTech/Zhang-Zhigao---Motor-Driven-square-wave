#ifndef _APP_LED_H
#define _APP_LED_H


/* 在 App_Led.h 中 */
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief  系统运行状态枚举 (与你的电机状态机联动)
 */
typedef enum {
    SYS_STAT_INIT = 0,    /* 初始化中：常亮 */
    SYS_STAT_IDLE,        /* 空闲待机：心跳呼吸 (亮 100ms, 灭 1900ms) */
    SYS_STAT_WORKING,     /* 正常工作：快速连闪 (亮 100ms, 灭 100ms) */
	SYS_STAT_ERROR,     /* 故障-过流：闪 2 下，停 1 秒 */
} System_Status_e;

/**
 * @brief  万能 LED 闪烁时序结构体
 */
typedef struct {
    uint16_t On_Time_ms;     /* 单次亮起时长 */
    uint16_t Off_Time_ms;    /* 单次熄灭时长 */
    uint8_t  Blink_Count;    /* 连闪次数 (0 代表无限循环不断闪) */
    uint16_t Pause_Time_ms;  /* 连闪一轮结束后的长暂停时长 */
} Led_Pattern_t;


/**
 * @brief  LED 非阻塞状态机驱动任务 (需在 1ms 节拍中调用)
 */
void App_Led_Task_1ms(void);
void App_Led_Set_State(System_Status_e new_state);












#endif

