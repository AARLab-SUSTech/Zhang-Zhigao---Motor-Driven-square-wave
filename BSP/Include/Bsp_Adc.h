/**
 ******************************************************************************
 * @file    Bsp_Adc.h
 * @brief   ADC 底层硬件驱动模块头文件
 * @note    提供 ADC 初始化、触发配置以及各通道原始数据读取的接口声明。
 ******************************************************************************
 */

#ifndef _BSP_ADC_H
#define _BSP_ADC_H

#include <stdint.h> /* 系统标准库建议使用尖括号 */

/* ==========================================
 * 初始化与控制接口
 * ========================================== */

/**
 * @brief  ADC 硬件初始化 (包含校准、DMA 配置及定时器触发)
 */
void Bsp_Adc_Init(void);

/**
 * @brief  开启定时器以触发 ADC 采样
 */
void Bsp_Adc_Trigger_Timer_Start(void);


/* ==========================================
 * 数据读取接口 (获取 ADC 12位原始采样值)
 * ========================================== */

/**
 * @brief  获取高压母线电压原始数据
 * @retval uint16_t ADC 原始采样值
 */
uint16_t Bsp_Get_Hv_Raw_Data(void);

/**
 * @brief  获取高压母线电流原始数据
 * @retval uint16_t ADC 原始采样值
 */
uint16_t Bsp_Get_Hi_Raw_Data(void);

/**
 * @brief  获取低压 DC 电流原始数据
 * @retval uint16_t ADC 原始采样值
 */
uint16_t Bsp_Get_Dc_I_Raw_Data(void);

/**
 * @brief  获取磁栅传感器 SIN 信号原始数据
 * @retval uint16_t ADC 原始采样值
 */
uint16_t Bsp_Get_Position_Sensor_Sin_Raw_Data(void);

/**
 * @brief  获取磁栅传感器 COS 信号原始数据
 * @retval uint16_t ADC 原始采样值
 */
uint16_t Bsp_Get_Position_Sensor_Cos_Raw_Data(void);

#endif /* _BSP_ADC_H */
