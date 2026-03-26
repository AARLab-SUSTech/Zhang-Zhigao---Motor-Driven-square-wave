#ifndef _APP_BUZZER_H
#define _APP_BUZZER_H

#include "stdint.h"
#include "stdbool.h"

/**
 * @brief  启动蜂鸣器 (非阻塞)
 * @param  ms: 响鸣时间
 */
void App_Buzzer_Beep(uint32_t ms);

/**
 * @brief  蜂鸣器后台守护任务 (放在 main.c 的 while(1) 中循环调用)
 */
void App_Buzzer_Task(void);





#endif




