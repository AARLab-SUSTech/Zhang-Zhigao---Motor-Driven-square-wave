#include "App_Buzzer.h"
#include "Bsp_Control.h"
#include "stm32g4xx_hal.h"

/* 定义蜂鸣器的工作模式 */
typedef enum {
    BUZZER_MODE_IDLE = 0,    // 静音
    BUZZER_MODE_ONESHOT,     // 单次发声 (如按键音)
    BUZZER_MODE_ERROR_ALARM  // 周期性报警音 (故障)
} Buzzer_Mode_t;

static Buzzer_Mode_t Buzzer_Mode = BUZZER_MODE_IDLE;

/* 状态机时间戳变量 */
static uint32_t Buzzer_Next_Toggle_Tick = 0;
static bool     Buzzer_HW_State = false;

/* ==================================================== *
 * 1. 单次发声 API (按键提示音调用这个)
 * ==================================================== */
void App_Buzzer_Beep(uint32_t ms)
{
    if (ms == 0) return;

    /* 如果系统正在严重报警，按键音不要去打断报警音 */
    if (Buzzer_Mode == BUZZER_MODE_ERROR_ALARM) return;

    Buzzer_Mode = BUZZER_MODE_ONESHOT;
    Buzzer_HW_State = true;
    Bsp_Buzzer_Control(true);
    Buzzer_Next_Toggle_Tick = HAL_GetTick() + ms; // 记录关闭时间
}

/* ==================================================== *
 * 2. 持续报警 API (故障中心调用这个)
 * ==================================================== */
void App_Buzzer_Set_Alarm(bool enable)
{
    if (enable)
    {
        if (Buzzer_Mode != BUZZER_MODE_ERROR_ALARM) {
            Buzzer_Mode = BUZZER_MODE_ERROR_ALARM;
            Buzzer_HW_State = true;
            Bsp_Buzzer_Control(true);
            Buzzer_Next_Toggle_Tick = HAL_GetTick() + 300; // 首次响 300ms
        }
    }
    else
    {
        Buzzer_Mode = BUZZER_MODE_IDLE;
        Buzzer_HW_State = false;
        Bsp_Buzzer_Control(false);
    }
}

/* ==================================================== *
 * 3. 蜂鸣器后台任务
 * ==================================================== */
void App_Buzzer_Task(void)
{
    if (Buzzer_Mode == BUZZER_MODE_IDLE) {
        return; /* 静音模式，直接退出*/
    }

    /* 时间到了！执行动作 */
    if (HAL_GetTick() >= Buzzer_Next_Toggle_Tick)
    {
        if (Buzzer_Mode == BUZZER_MODE_ONESHOT)
        {
            /* 单次发声结束，关闭硬件并切回 IDLE */
            Buzzer_HW_State = false;
            Bsp_Buzzer_Control(false);
            Buzzer_Mode = BUZZER_MODE_IDLE;
        }
        else if (Buzzer_Mode == BUZZER_MODE_ERROR_ALARM)
        {
            /* 周期性报警：翻转蜂鸣器状态 (滴、滴、滴...) */
            Buzzer_HW_State = !Buzzer_HW_State;
            Bsp_Buzzer_Control(Buzzer_HW_State);

            /* 设置下一次翻转的时间 (你可以随意调节这个节奏) */
            if (Buzzer_HW_State == true) {
                Buzzer_Next_Toggle_Tick = HAL_GetTick() + 200; // 响 200ms
            } else {
                Buzzer_Next_Toggle_Tick = HAL_GetTick() + 1200; // 停 1200ms
            }
        }
    }
}
