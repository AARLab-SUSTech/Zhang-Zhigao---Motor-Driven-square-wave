/*
 * control.h
 *
 *  Created on: Sep 2, 2025
 *      Author: 16964
 */

#ifndef _BSP_CONTROL_H
#define _BSP_CONTROL_H

#include "main.h"

/* ==========================================
 * 数据结构定义
 * ========================================== */
typedef struct {
    // --- 参数区 (调试时只需改这几个) ---
    float Kp;           // 比例增益：决定电机响应的快慢
    float MaxSpeed;     // 速度限幅：对应您的开环速度最大值(比如PWM占空比或频率)

    // --- 状态区 (只读) ---
    float TargetPos;    // 目标位置
    float CurrentPos;   // 当前位置
    float Error;        // 当前误差
    float OutSpeed;     // 计算出的输出速度

} P_Controller;
void P_Control_Init(P_Controller *ctrl, float kp, float max_speed);
float P_Control_Compute(P_Controller *ctrl, float target_pos, float current_pos) ;

void DC_Power_CTR(bool state);
void set_speed(float speed_hz);
void move_to_position(int32_t target_steps, float speed_hz);
void move_repeatedly(int32_t pos1, int32_t pos2, uint32_t count, float speed_hz);
void start_move_to_position(int32_t target_steps, float speed_hz);
void step_move(int32_t relative_steps, float speed_hz);
void Update_output(uint8_t new_step);
void Close_output(void);
#endif /* SRC_CONTROL_H_ */
