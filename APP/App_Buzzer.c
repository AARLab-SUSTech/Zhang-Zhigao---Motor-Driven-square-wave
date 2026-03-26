#include "App_Buzzer.h"
#include "Bsp_Control.h"
#include "stm32g4xx_hal.h"

/* 记录蜂鸣器应该关闭的时间戳 */
static uint32_t Buzzer_Stop_Tick = 0;
/* 记录蜂鸣器当前是否在响 */
static bool Buzzer_Is_On = false;

/**
 * @brief  启动蜂鸣器 (非阻塞)
 * @param  ms: 响鸣时间
 */
void App_Buzzer_Beep(uint32_t ms)
{
    if (ms == 0) return;

    Bsp_Buzzer_Control(true);            // 1. 硬件引脚拉高，开始发声
    Buzzer_Stop_Tick = HAL_GetTick() + ms; // 2. 计算并记录未来的停止时间点
    Buzzer_Is_On = true;                 // 3. 标记状态
}

/**
 * @brief  蜂鸣器后台守护任务 (放在 main.c 的 while(1) 中循环调用)
 */
void App_Buzzer_Task(void)
{
    /* 只有当蜂鸣器在响，并且当前系统时间已经超过了设定的停止时间时，才执行关闭 */
    if (Buzzer_Is_On && (HAL_GetTick() >= Buzzer_Stop_Tick))
    {
        Bsp_Buzzer_Control(false); // 关闭硬件
        Buzzer_Is_On = false;      // 清除状态
    }
}












