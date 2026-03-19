#include "App_Fault.h"
#include "App_EMA.h"
#include "App_Led.h"
#include "Bsp_Control.h"

extern Motor EMA_DATA;

/**
 * @brief  上报故障 (只负责记录，不负责动作)
 * @param  fault_code 故障码字典中的值
 */
void App_Fault_Report(uint32_t fault_code)
{
    /* 使用按位或 (|) 记录故障，保护其他已存在的故障不被覆盖 */
    EMA_DATA.Fault_Flags |= fault_code;
}

/**
 * @brief  清除故障 (上位机发送复位指令时调用)
 */
void App_Fault_Clear(uint32_t fault_code)
{
    /* 使用按位与非 (& ~) 清除特定故障 */
    EMA_DATA.Fault_Flags &= ~fault_code;
    App_Led_Set_State(SYS_STAT_IDLE);
}

/**
 * @brief  全局故障裁决任务 (在 main 函数 while(1) 中高频轮询)
 */
void App_Fault_Task_Handler(void)
{
    /* 1. 如果没有任何故障，直接退出 */
    if (EMA_DATA.Fault_Flags == FAULT_NONE)
    {
        return;
    }

    /* 2. 只要有故障，立刻强制系统进入 ERROR 模式 */
    if (EMA_DATA.Motor_mode != MOTOR_ERROR)
    {
        EMA_DATA.Motor_mode = MOTOR_ERROR;

        /* 执行统一的紧急安全动作序列 */
        App_Led_Set_State(SYS_STAT_ERROR);
        Bsp_Close_All_Output();           /* 封锁 PWM */
        Bsp_Dc_Power_Control(false);      /* 切断高压 */
        //Bsp_Buzzer_Control(true);         /* 鸣叫报警 */
    }


}
