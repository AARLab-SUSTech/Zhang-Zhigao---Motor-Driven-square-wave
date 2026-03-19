/**
 ******************************************************************************
 * @file    Mid_Position_Sensor.c
 * @author  Letian
 * @date    [20260316]
 * @brief   位置传感器中间件 (MID)
 * @note    负责从底层抓取 ADC 原始数据，完成正余弦解码与角度回绕计算。
 * 对外隐藏算法状态，仅暴露只读接口 (Getters)。
 ******************************************************************************
 */

#include "Mid_Position_Sensor.h"
#include "Bsp_Adc.h"
#include <math.h>

/* ==========================================
 * 私有全局变量定义 (Private Variables)
 * ========================================== */

/* 【安全修复】：加上 static 关键字，使其作用域仅限当前文件，真正做到信息隐蔽 */
static MagEncoder_State_t s_MagSensor;

/* ==========================================
 * 算法核心实现 (Core Implementations)
 * ========================================== */

/**
 * @brief 初始化传感器状态
 */
void Mid_Encoder_Init(void)
{
    s_MagSensor.Theta_Radians          = 0.0f;
    s_MagSensor.Theta_Degrees          = 0.0f;
    s_MagSensor.Previous_Theta_Degrees = 0.0f;
    s_MagSensor.Cycle_Count            = 0;
    s_MagSensor.First_Calculation      = 1; /* 标记为首次计算 */
}

/**
 * @brief  【核心流】抓取底层数据并完成角度与回绕解算
 * @param  sin_offset APP层下发的正弦通道标定零点
 * @param  cos_offset APP层下发的余弦通道标定零点
 */
void Mid_Encoder_Update(float sin_offset, float cos_offset)
{
    /* 1. MID 层亲自去底层拿原始数据，并完成电压换算 */
    float sin_v = Bsp_Get_Position_Sensor_Sin_Raw_Data() * 0.0008056640625f;
    float cos_v = Bsp_Get_Position_Sensor_Cos_Raw_Data() * 0.0008056640625f;

    /* 2. 角度解算 */
    s_MagSensor.Theta_Radians = atan2f(sin_v - sin_offset, cos_v - cos_offset);

    /* 弧度转角度 (180/PI 预结算常量，避免运行时除法开销) */
    float Current_Theta_Degrees = s_MagSensor.Theta_Radians * 57.2957795131f;

    if (Current_Theta_Degrees < 0)
    {
        Current_Theta_Degrees += 360.0f;
    }
    s_MagSensor.Theta_Degrees = Current_Theta_Degrees;

    /* 3. 过零点回绕检测 */
    float delta_angle = s_MagSensor.Theta_Degrees - s_MagSensor.Previous_Theta_Degrees;

    if (!s_MagSensor.First_Calculation)
    {
        if (delta_angle > 180.0f)
        {
            s_MagSensor.Cycle_Count--;
        }
        else if (delta_angle < -180.0f)
        {
            s_MagSensor.Cycle_Count++;
        }
    }
    else
    {
        s_MagSensor.First_Calculation = 0;
    }

    s_MagSensor.Previous_Theta_Degrees = s_MagSensor.Theta_Degrees;
}


/* ==========================================
 * 提供给 APP 层的只读数据接口 (Getters)
 * ========================================== */

/**
 * @brief 获取当前解算出的电气角度 (0~360度)
 */
float Mid_Encoder_Get_Angle_Degrees(void)
{
    return s_MagSensor.Theta_Degrees;
}

/**
 * @brief 获取当前累计的完整圈数/周期数
 */
int32_t Mid_Encoder_Get_Cycle_Count(void)
{
    return s_MagSensor.Cycle_Count;
}
