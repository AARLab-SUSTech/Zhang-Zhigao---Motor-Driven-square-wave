#include "App_Led.h"

/* 在 App_Led.c 中 */
#include "App_Led.h"
#include "Bsp_Led.h"

/* 定义各个状态对应的闪烁时序参数 */
static const Led_Pattern_t Led_Config_Table[] = {
    /* On_Time, Off_Time, Count, Pause_Time */
    [SYS_STAT_INIT]    = {1000, 0,    0, 0},    /* 常亮 (配置为亮1000ms, 灭0ms) */
    [SYS_STAT_IDLE]    = {100,  900, 0, 0},    /* 心跳：亮 100ms，灭 1900ms，无限循环 */
    [SYS_STAT_WORKING] = {600,  600,  0, 0},    /* 狂闪：亮 250ms，灭 250ms，无限循环 */
    [SYS_STAT_ERROR] = {100,  100,  6, 600}, /* 报故障 2：短闪 2 下，长停 500ms */
};

/* 当前正在执行的状态 */
static System_Status_e Current_Sys_State = SYS_STAT_INIT;


/**
 * @brief  LED 非阻塞状态机驱动任务 (需在 1ms 节拍中调用)
 */
void App_Led_Task_1ms(void)
{
    static uint16_t time_counter = 0;    /* 毫秒计步器 */
    static uint8_t  flash_counter = 0;   /* 当前已闪烁次数 */
    static bool     is_led_on = false;   /* 当前 LED 物理亮灭状态 */
    static bool     is_pausing = false;  /* 是否处于长暂停阶段 */

    /* 1. 获取当前状态对应的配方 */
    Led_Pattern_t pattern = Led_Config_Table[Current_Sys_State];

    /* 2. 时钟滴答累加 */
    time_counter++;

    /* 3. 状态机流转逻辑 */
    if (is_pausing)
    {
        /* 处于周期长暂停阶段 */
        Bsp_Led_Off();
        if (time_counter >= pattern.Pause_Time_ms)
        {
            is_pausing = false;
            time_counter = 0;
            flash_counter = 0; /* 开启下一轮 */
        }
    }
    else if (is_led_on)
    {
        /* 处于亮起阶段 */
        Bsp_Led_On();
        if (time_counter >= pattern.On_Time_ms)
        {
            is_led_on = false;
            time_counter = 0;
        }
    }
    else
    {
        /* 处于短暂熄灭阶段 */
        Bsp_Led_Off();
        if (time_counter >= pattern.Off_Time_ms)
        {
            is_led_on = true;
            time_counter = 0;

            /* 如果配置了连闪次数，则进行计数核对 */
            if (pattern.Blink_Count > 0)
            {
                flash_counter++;
                if (flash_counter >= pattern.Blink_Count)
                {
                    is_pausing = true; /* 次数达标，进入长暂停 */
                    is_led_on = false;
                }
            }
        }
    }
}

/**
 * @brief  外部接口：供主程序改变系统状态
 */
void App_Led_Set_State(System_Status_e new_state)
{
    /* 只有当状态发生真正变化时才重置状态机，防止死锁 */
    if (Current_Sys_State != new_state)
    {
        Current_Sys_State = new_state;
        /* 这里可以加两行代码强制复位内部的 static 计数器，保证切换瞬间响应 */
    }
}

































