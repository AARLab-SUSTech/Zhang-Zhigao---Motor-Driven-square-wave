/*
 * FOC.h
 *
 *  Created on: May 14, 2025
 *      Author: 16964
 */

#ifndef _APP_EMA_H_
#define _APP_EMA_H_

#include "stdbool.h"
#include "stdint.h"

// 电机控制模式
typedef enum {
    MOTOR_IDLE = 0,              // 空闲模式
	MOTOR_READY,				 //待机模式，高压上电
    MOTOR_ERROR,                 // 错误/急停模式
    MOTOR_OPEN_SPEED,            // 速度模式
    MOTOR_OPEN_POSITION,         // 单次定位模式
	MOTOR_OPEN_REPEATED,         // 往复运动模式
	MOTOR_OPEN_VELOCITY,         //开环速度模式
	MOTOR_CLOSE_POSITION,	     //闭环位置模式
	MOTOR_CLOSE_FORCE,	         //闭环力模式
	MOTOR_CLOSE_VELOCITY,		 //闭环速度模式
	MOTOR_SYNC_POSITION,         //多电机同步模式
	MOTOR_OVER_HV_CURRENT,
	MOTOR_OVER_HV_VOLTAGE,
	MOTOR_OVER_DC_IN_CURRENT
} Motor_Control_Mode_e;

typedef struct {

    uint32_t sin_cos_ADC_RAW[2];
    float sin_V;
    float cos_V;
    float sin_offset;
    float cos_offset;
	float theta_radians;
	float theta_degrees;
	float current_displacement_within_cycle_mm;
	float position_mm;
	float Last_position_mm;
	float speed;
	float Last_speed;
    float previous_theta_degrees; // 用于存储上一次的角度值
    int32_t cycle_count;          // 用于累计完整的周期数 (圈数)
    uint8_t first_calculation;

    uint8_t Loop_count;//计数用于低速位置和速度环

    // Store calibration results for displacement
    float min_position_mm;
    float max_position_mm;
    float distance_of_travel;

    bool Min_Max_cal_status;
    bool theta_offset_cal_status;

    Motor_Control_Mode_e Motor_mode;

    float recip_theta_A;
    float recip_theta_B;
    bool recip_moving_to_B;
    bool recip_is_homing;
    uint16_t recip_target_cycles;
    uint16_t recip_current_cycle;

    struct {
    	float kp;                 // 位置环比例增益
        float kd;                 // (可选) 微分增益
        float error_sum_pos_rad;
    } Position_P;
    float Id_ref;                // d轴期望电流 (mA)
    float Iq_ref;                // q轴期望电流 (mA)

}Motor;

extern Motor EMA_DATA;

#endif /* SRC_FOC_H_ */
