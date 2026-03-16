/**
 ******************************************************************************
 * @file    Bsp_Adc.c
 * @brief   ADC 底层硬件驱动模块
 * @note    负责 ADC 校准、DMA 数据自动搬运配置、以及定时器触发采样。
 * 提供各个通道原始数据的读取接口，供 MID 层进行物理量转换。
 ******************************************************************************
 */

#include "Bsp_Adc.h"
#include "stm32g4xx_hal.h"

/* ==========================================
 * 外部句柄声明 (定义在 main.c 中)
 * ========================================== */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim7;

/* ==========================================
 * 私有变量定义 (Static Variables)
 * ========================================== */
/* ADC DMA 搬运目标缓冲区 (加 volatile 防止被编译器优化) */
static volatile uint32_t Adc1_Raw_Data[2];
static volatile uint32_t Adc2_Raw_Data[3];


/* ==========================================
 * 函数实现
 * ========================================== */

/**
 * @brief  ADC 硬件初始化
 * @note   执行 ADC 校准，开启 DMA 数据搬运，并启动定时器触发采样
 */
void Bsp_Adc_Init(void)
{
    /* 1. ADC 校准 */
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_DIFFERENTIAL_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

    /* 2. 开启 ADC 数据 DMA 自动搬运
     * 注: (uint32_t *) 强制转换用于消除传递 volatile 指针时的编译器警告
     */
    /* 采集高压母线电压 (HV_V) 和电流 (HV_I) 数据 */
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)Adc1_Raw_Data, 2);

    /* 采集低压 DC 电流 (DC_I) 和磁栅传感器正交信号 (SIN, COS) 值 */
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)Adc2_Raw_Data, 3);

    /* 3. 开启定时器触发 ADC DMA 自动采样和搬运 */
    Bsp_Adc_Trigger_Timer_Start();
}

/**
 * @brief  开启定时器以触发 ADC 采样
 * @note   使用 TIM7 作为 ADC DMA 的外部硬件触发源
 */
void Bsp_Adc_Trigger_Timer_Start(void)
{
    /* 启动 ADC DMA 外部触发定时器 */
    HAL_TIM_Base_Start_IT(&htim7);
}

/**
 * @brief  获取高压母线电压原始数据
 * @retval uint16_t ADC 原始采样值 (通常为 12 位)
 */
uint16_t Bsp_Get_Hv_Raw_Data(void)
{
    return (uint16_t)Adc1_Raw_Data[0];
}

/**
 * @brief  获取高压母线电流原始数据
 * @retval uint16_t ADC 原始采样值
 */
uint16_t Bsp_Get_Hi_Raw_Data(void)
{
    return (uint16_t)Adc1_Raw_Data[1];
}

/**
 * @brief  获取低压 DC 电流原始数据
 * @retval uint16_t ADC 原始采样值
 */
uint16_t Bsp_Get_Dc_I_Raw_Data(void)
{
    return (uint16_t)Adc2_Raw_Data[0];
}

/**
 * @brief  获取磁栅传感器 SIN 信号原始数据
 * @retval uint16_t ADC 原始采样值
 */
uint16_t Bsp_Get_Position_Sensor_Sin_Raw_Data(void)
{
    return (uint16_t)Adc2_Raw_Data[1];
}

/**
 * @brief  获取磁栅传感器 COS 信号原始数据
 * @retval uint16_t ADC 原始采样值
 */
uint16_t Bsp_Get_Position_Sensor_Cos_Raw_Data(void)
{
    return (uint16_t)Adc2_Raw_Data[2];
}
