/**
 ******************************************************************************
 * @file    Mid_Position_Sensor.h
 * @author  Letian
 * @date    [填入当前日期]
 * @brief   位置传感器中间件头文件 (MID)
 * @note    提供磁栅/旋转变压器传感器状态结构体定义，
 * 以及底层数据抓取、角度解算与状态获取的接口声明。
 ******************************************************************************
 */

#ifndef _MID_POSITION_SENSOR_H_
#define _MID_POSITION_SENSOR_H_

#include <stdint.h> /* 系统标准库建议使用尖括号 */

/* ========================================================================== *
 * 数据结构定义 (Data Structures)
 * ========================================================================== */

/**
 * @brief 磁栅/旋转变压器纯粹的传感器状态结构体
 * @note  完全脱离具体的电机业务，仅描述纯数学与物理运动的角度和圈数。
 */
typedef struct {
    float   Theta_Radians;          /* 当前电气角度 (弧度) */
    float   Theta_Degrees;          /* 当前电气角度 (角度 0~360) */
    float   Previous_Theta_Degrees; /* 上一次的角度 (用于过零点回绕检测) */
    int32_t Cycle_Count;            /* 累计周期数 (圈数) */
    uint8_t First_Calculation;      /* 首次计算标志位 (1: 是, 0: 否) */
} MagEncoder_State_t;


/* ========================================================================== *
 * 函数接口声明 (Function Prototypes)
 * ========================================================================== */

/**
 * @brief  初始化传感器内部状态与历史数据
 */
void Mid_Encoder_Init(void);

/**
 * @brief  【核心流】抓取底层 ADC 数据并完成角度与过零点回绕解算
 * @param  sin_offset APP 层下发的正弦通道标定零点
 * @param  cos_offset APP 层下发的余弦通道标定零点
 */
void Mid_Encoder_Update(float sin_offset, float cos_offset);

/**
 * @brief  获取当前解算出的电气角度
 * @retval float 角度值 (0~360度)
 */
float Mid_Encoder_Get_Angle_Degrees(void);

/**
 * @brief  获取当前累计的完整圈数/周期数
 * @retval int32_t 累计圈数 (支持正反转负数累计)
 */
int32_t Mid_Encoder_Get_Cycle_Count(void);

#endif /* _MID_POSITION_SENSOR_H_ */
