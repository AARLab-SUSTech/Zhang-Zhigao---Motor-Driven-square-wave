#include "App_Voltage_Current.h"
#include "Mid_Adc.h"
#include "App_Led.h"
#include "App_EMA.h"
#include "App_Fault.h"
#include "Mid_Control.h"
#include "Bsp_Control.h"

/* ==========================================
 * 外部依赖声明 (Extern Declarations)
 * ========================================== */
extern Motor EMA_DATA;

float MAX_HV_voltage=2600;
float MAX_HV_current=12;
float MAX_DC_current=4;

// 1. 算法参数 (您可以根据实际情况调整)
const float  SPIKE_CURRENT_THRESHOLD   = 16.0f;    // 定义尖峰电流的阈值 (单位: mA)
const uint16_t CHECK_WINDOW_SAMPLES    = 60;      // 定义检查窗口的大小 (N个采样点)。200个点 @ 20kHz = 1ms
const uint16_t MAX_SPIKES_IN_WINDOW    = 50;      // 定义在一个窗口期内，允许出现的最大尖峰次数

// 2. 算法工作变量
uint16_t window_sample_counter = 0;   // 用于在窗口内计数的采样点计数器 (从0数到CHECK_WINDOW_SAMPLES)
uint16_t spike_count_in_window = 0;   // 用于累计一个窗口期内的尖峰次数

void App_Upate_Voltage_Current_Data(void)
{
	EMA_DATA.Hv_V_V = Mid_Get_Hv_V();
	EMA_DATA.Hv_I_mA = Mid_Get_Hi_uA();
	EMA_DATA.Dc_I_A = Mid_Get_Dc_I_A();
}

/**
 * @brief  系统级安全与故障监控任务 (APP 层)
 * @note   负责监控高压母线与低压供电的电压/电流状态。
 * 包含滑动窗口滤波的软保护 (防误触)，以及绝对阈值的硬保护 (极速熔断)。
 * 建议在控制主循环或高频定时器任务中周期调用 (如 1kHz)。
 */
void App_System_Safety_Monitor(void)
{
    /* ========================================================================== *
     * 1. 硬件级绝对阈值保护：实时拦截 (针对严重短路或失控)
     * ========================================================================== */

    /* 1.1 高压母线过压保护 */
    if (EMA_DATA.Hv_V_V > MAX_HV_voltage)
    {
        App_Fault_Report(FAULT_HV_OVER_VOLTAGE);
        Bsp_Close_All_Output();
        App_Led_Set_State(SYS_STAT_ERROR);
    }

    /* 1.2 高压母线严重过流保护 (阈值为常规额定值的 3 倍) */
    if (EMA_DATA.Hv_I_mA > (MAX_HV_current * 3.0f))
    {
        App_Fault_Report(FAULT_HV_OVER_CURRENT);
        Bsp_Close_All_Output();
        Bsp_Buzzer_Control(true);
        App_Led_Set_State(SYS_STAT_ERROR);
    }

    /* 1.3 低压直流输入过流保护 */
    if (EMA_DATA.Dc_I_A > MAX_DC_current)
    {
        App_Fault_Report(FAULT_DC_OVER_CURRENT);
        Bsp_Close_All_Output();
        App_Led_Set_State(SYS_STAT_ERROR);
    }

    /* ========================================================================== *
     * 2. 软过流保护：滑动窗口尖峰计数 (针对瞬态干扰或早期过流的防误触设计)
     * ========================================================================== */

    /* 2.1 检查当前电流是否形成了一次尖峰 */
    if (EMA_DATA.Hv_I_mA > SPIKE_CURRENT_THRESHOLD)
    {
        spike_count_in_window++;
    }

    /* 2.2 采样点计数器自增 */
    window_sample_counter++;

    /* 2.3 判断一个检查窗口是否已经结束 */
    if (window_sample_counter >= CHECK_WINDOW_SAMPLES)
    {
        /* 窗口结束，进行故障判定 */
        if (spike_count_in_window > MAX_SPIKES_IN_WINDOW)
        {
            /* 在过去的 N 个采样点中，尖峰次数过多，判定为真实过流故障！ */
        	App_Fault_Report(FAULT_HV_OVER_CURRENT);

            /* 紧急安全序列 */
            Bsp_Close_All_Output();         /* 1. 封锁 PWM 输出 */
            Bsp_Dc_Power_Control(false);    /* 2. 切断高压 DCDC 电源 */
        }

        /* 1.4 重置计数器，为下一个检查窗口做准备 */
        window_sample_counter = 0;
        spike_count_in_window = 0;
    }

}




