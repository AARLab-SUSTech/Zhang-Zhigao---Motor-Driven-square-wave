#include "App_Usart.h"
#include "Mid_Command_Usart.h"

/* 在 App_EMA.c 或专门的 App_Usart.c 中 */

int dma_print_flag = 0;

/**
 * @brief  USART 遥测数据发送任务 (APP 层)
 * @note   在定时器或主循环中周期性调用 (如 100Hz 或 1kHz)
 */
void App_Usart_Telemetry_Task(void)
{
    /* 1. 检查打印使能标志 */
    if (dma_print_flag == 0)
    {
        return;
    }

    /* 2. 采集需要上传的业务数据 */
    float TelemetryData[6];

//    TelemetryData[0] = HV_V;                                /* 高压电压 V */
//    TelemetryData[1] = HV_I;                                /* 高压电流 mA */
//    TelemetryData[2] = (float)absolute_step_counter;        /* 绝对步数 (需强转为 float 适应协议) */
//    TelemetryData[3] = EMA_DATA.position_mm;                /* 当前位置 mm */
//    TelemetryData[4] = Motor_Position_Controller.TargetPos; /* 目标位置 */
//    TelemetryData[5] = DC_I;                                /* 低压电流 A */

    /* 3. 调用 MID 层协议接口发送 */
    Mid_Usart_Send_JustFloat(TelemetryData, 6);
}
