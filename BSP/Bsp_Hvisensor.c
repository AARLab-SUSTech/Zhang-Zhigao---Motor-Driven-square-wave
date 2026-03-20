/**
 ******************************************************************************
 * @file    hvisensor.c
 * @brief   高压/电流传感器中断处理模块 (BSP 级回调)
 * @note    包含 ADC 转换完成回调 (用于物理量换算) 和
 * 比较器触发回调 (用于硬件级极速过压/过流保护)。
 ******************************************************************************
 */

#include <Bsp_Hvisensor.h>
#include <Mid_Control.h>
#include "main.h"
#include "Bsp_Control.h"
#include "App_EFA.h"
#include "App_Fault.h"
/* ==========================================
 * 外部依赖声明 (Extern Declarations)
 * ========================================== */
extern Motor EMA_DATA;

/* ==========================================
 * 回调函数实现 (Callback Implementations)
 * ========================================== */

/**
 * @brief  ADC 转换完成回调函数
 * @param  hadc ADC 句柄
 * @note   该函数由 HAL 库在 ADC DMA 转换完成后自动调用。
 * 目前包含将 ADC 原始值转换为物理量 (电压/电流) 的预留逻辑。
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    /* 检查是否为 ADC1 (高压母线采集) */
    if (hadc->Instance == ADC1)
    {
        /* ========================================================== */
        /* [DEBUG 预留] 高压电压与电流的物理量换算逻辑 */
        /*
        int temp_data[2];
        temp_data[0] = (ADC1_RAW_data[0] - 2048) * 2;
        temp_data[1] = (ADC1_RAW_data[1] - 2048) * 2;

        // V 和 mA 的计算公式
        // HV_V = (temp_data[0] / 4096) * 3.3f * 2 * 501 / 0.4f
        HV_V = temp_data[0] * 1.00909423828125f;

        // HV_I = (temp_data[1] / 4096) * 3.3 / 8 / 20 * 1000
        HV_I = temp_data[1] * 0.005035400390625f;

        // printf("%.3f,%.3f\r\n", HV_V, HV_I);
        */
        /* ========================================================== */
    }

    /* 检查是否为 ADC2 (低压直流电流与位置传感器采集) */
    if (hadc->Instance == ADC2)
    {
        /* 将原始数据换算为低压 DC 电流
         * 换算公式: (ADC2_RAW_data[0] / 4096) * 3.3 / 200 / 0.005
         */
//        DC_I = ADC2_RAW_data[0] * 0.0008056640625f;

        /* ========================================================== */
        /* [DEBUG 预留] 打印测试数据 */
        /*
        // printf("%.3f,%ld,%ld\r\n", DC_I, ADC2_RAW_data[1], ADC2_RAW_data[2]);
        */
        /* ========================================================== */
    }
}

/**
 * @brief  模拟比较器 (COMP) 触发回调函数
 * @param  hcomp 比较器句柄
 * @note   用于极速的硬件级过压、过流保护响应。触发后直接切断输出并上报状态。
 */
void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp)
{
    /* 检查是否为比较器 1 触发 (过压保护) */
    if (hcomp == &hcomp1)
    {
        /* 1. 更新电机状态机为高压过压故障 */
    	App_Fault_Report(FAULT_HV_OVER_VOLTAGE);
        /* 2. 紧急关闭 PWM 输出 */
        Bsp_Close_All_Output();

        /* ========================================================== */
        /* [代码预留] 蜂鸣器报警与 DCDC 电源切断逻辑 */
        /*
        // Bsp_Buzzer_Control(true); // 提示：解除注释时请确保包含分号
        // if(DC_ON_State == true) { DC_Power_CTR(false); }
        // printf("HV error\r\n");
        */
        /* ========================================================== */
    }

    /* 检查是否为比较器 2 触发 (过流保护) */
    if (hcomp == &hcomp2)
    {
        /* 1. 更新电机状态机为高压过流故障 */
        App_Fault_Report(FAULT_HV_OVER_CURRENT);

        /* 2. 开启蜂鸣器报警 */
        Bsp_Buzzer_Control(true);

        /* 3. 紧急关闭 PWM 输出 */
        Bsp_Close_All_Output();

        /* ========================================================== */
        /* [代码预留] DCDC 电源切断逻辑 */
        /*
        // if(DC_ON_State == true) { DC_Power_CTR(false); }
        // printf("HV I \r\n");
        */
        /* ========================================================== */
    }
}
