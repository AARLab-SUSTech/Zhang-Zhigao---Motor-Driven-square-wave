/**
 ******************************************************************************
 * @file    App_Position_Sensor.c
 * @author  Letian
 * @date    [20260316]
 * @brief   电机位置与速度状态处理应用层 (APP)
 * @note    负责调用 MID 层算法获取纯净的角度数据，并结合电机极距参数
 * (2.0mm) 进行运动学转换，计算绝对物理位置与滤波后的速度。
 ******************************************************************************
 */

#include "App_EMA.h"
#include "App_Position_Sensor.h"
#include "Mid_Position_Sensor.h"

/* ==========================================
 * 外部依赖声明 (Extern Declarations)
 * ========================================== */
extern Motor EFA_DATA;

/* ==========================================
 * 函数实现 (Function Implementations)
 * ========================================== */

/**
 * @brief  更新电机位置与速度状态数据
 * @note   建议在控制主循环或高频定时器中断中周期性调用 (如 1kHz)。
 * 内部包含了对速度计算的 10 倍降频处理 (100Hz) 及一阶低通滤波。
 */
void App_Update_Position_Sensor_Data(void)
{
    /* 1. 触发 MID 层更新传感器数据
     * (APP 层把自己的偏置参数传给 MID 层算，但不管它是怎么去底层拿ADC的)
     */
    Mid_Encoder_Update(EFA_DATA.sin_offset, EFA_DATA.cos_offset);

    /* 2. 直接从 MID 层拿走已经算好的、干干净净的角度和圈数！ */
    EFA_DATA.theta_degrees = Mid_Encoder_Get_Angle_Degrees();
    EFA_DATA.cycle_count   = Mid_Encoder_Get_Cycle_Count();

    /* 3. 运动学转换：角度转直线位移 (极距 2.0mm) */
    EFA_DATA.current_displacement_within_cycle_mm = (EFA_DATA.theta_degrees / 360.0f) * 2.0f;

    /* 4. 应用标定偏置，计算绝对物理位置 */
    if (EFA_DATA.Min_Max_cal_status == false)
    {
        EFA_DATA.position_mm = (float)EFA_DATA.cycle_count * 2.0f + EFA_DATA.current_displacement_within_cycle_mm;
    }
    else
    {
        EFA_DATA.position_mm = (float)EFA_DATA.cycle_count * 2.0f + EFA_DATA.current_displacement_within_cycle_mm - EFA_DATA.min_position_mm + 1.5f;
    }

    /* 5. 降频计算物理速度与滤波 */
    EFA_DATA.Loop_count++;
    if (EFA_DATA.Loop_count >= 10)
    {
        /* 1kHz 下降频 10 次为 100Hz，dt = 0.01s */
        EFA_DATA.speed = (EFA_DATA.position_mm - EFA_DATA.Last_position_mm) / 0.01f;

        /* 一阶低通滤波 (当前权重 0.8，历史权重 0.2) */
        EFA_DATA.speed = 0.8f * EFA_DATA.speed + 0.2f * EFA_DATA.Last_speed;

        /* 更新历史状态 */
        EFA_DATA.Last_speed = EFA_DATA.speed;
        EFA_DATA.Last_position_mm = EFA_DATA.position_mm;

        /* 清零降频计数器 */
        EFA_DATA.Loop_count = 0;
    }
}

