#ifndef _BSP_CONTROL_H
#define _BSP_CONTROL_H

#include "stdbool.h"
#include "stdint.h"

/**
 * @brief  控制高压 DCDC 电源状态
 */
void Bsp_Dc_Power_Control(bool state);

/**
 * @brief  控制蜂鸣器报警状态
 */
void Bsp_Buzzer_Control(bool state);

/**
 * @brief  三相无刷电机 (BLDC) 六步换向底层驱动
 * @param  new_step 新的换向节拍 (0~5)
 * @note   在切换前先关闭所有通道，防止上下桥臂直通短路 (Shoot-through)。
 */
void Bsp_Update_Output(uint8_t new_step);

/**
 * @brief  紧急关闭所有底层输出 (硬件级安全切断)
 * @note   封锁高压 DCDC 并强行关闭 TIM1 的所有六个 PWM 桥臂通道。
 */
void Bsp_Close_All_Output(void);

#endif
