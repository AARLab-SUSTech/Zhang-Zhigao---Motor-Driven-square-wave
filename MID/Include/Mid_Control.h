/**
 ******************************************************************************
 * @file    Mid_Control.h (原 control.h)
 * @author  Letian
 * @date    Sep 2, 2025
 * @brief   电机综合控制模块头文件
 * @note    提供 P 控制器数据结构及算法、电机运动模式配置、
 * 以及底层 PWM 和电源开关的函数接口声明。
 ******************************************************************************
 */

#ifndef _MID_CONTROL_H_
#define _MID_CONTROL_H_

#include "main.h"

/* ========================================================================== *
 * 数据结构定义 (Data Structures)
 * ========================================================================== */

/**
 * @brief  位置环 P 控制器结构体
 */
typedef struct {
    /* --- 参数区 (调试时只需改这几个) --- */
    float Kp;           /* 比例增益：决定电机响应的快慢 */
    float MaxSpeed;     /* 速度限幅：对应开环速度最大值 (如 PWM 占空比或频率) */

    /* --- 状态区 (只读) --- */
    float TargetPos;    /* 目标位置 */
    float CurrentPos;   /* 当前实际位置 */
    float Error;        /* 当前计算误差 */
    float OutSpeed;     /* 计算出的输出速度控制量 */
} P_Controller;


/* ========================================================================== *
 * 【MID 层】算法控制接口声明
 * ========================================================================== */

/**
 * @brief  初始化 P 控制器参数
 */
void P_Control_Init(P_Controller *ctrl, float kp, float max_speed);

/**
 * @brief  P 控制器计算核心函数
 * @retval float 计算得出的输出速度控制量
 */
float P_Control_Compute(P_Controller *ctrl, float target_pos, float current_pos);


/* ========================================================================== *
 * 【APP 层】顶层业务控制接口声明
 * ========================================================================== */

/**
 * @brief  设置电机速度和方向 (开环速度模式)
 */
void set_speed(float speed_hz);

/**
 * @brief  设置电机移动到目标位置 (开环位置模式)
 */
void move_to_position(int32_t target_steps, float speed_hz);

/**
 * @brief  启动往复运动模式
 */
void move_repeatedly(int32_t pos1, int32_t pos2, uint32_t count, float speed_hz);

/**
 * @brief  启动一次单次定位任务
 */
void start_move_to_position(int32_t target_steps, float speed_hz);

/**
 * @brief  执行一次相对步进移动
 */
void step_move(int32_t relative_steps, float speed_hz);


/* ========================================================================== *
 * 【BSP 层】底层硬件控制接口声明
 * ========================================================================== */

/**
 * @brief  DCDC 高压模块供电开关控制
 */
void DC_Power_CTR(bool state);

/**
 * @brief  三相无刷电机六步换向底层驱动
 */
void Update_output(uint8_t new_step);

/**
 * @brief  紧急关闭所有底层输出 (包括 PWM 和 DCDC)
 */
void Close_output(void);


#endif /* _MID_CONTROL_H_ */
