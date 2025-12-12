/*
 * control.c
 *
 *  Created on: Sep 2, 2025
 *      Author: Letian
 */

#include "control.h"
#include "main.h"
#include "command.h"
#include "FOC.h"

#ifndef _CONTROL_H_
#define _CONTROL_H_



extern volatile MotorDirection_t motor_direction; // 默认为正
extern volatile Motor_mode_t Motor_mode;

extern uint32_t ADC2_RAW_data[3];

/* ==========================================
 * 初始化函数
 * ========================================== */
void P_Control_Init(P_Controller *ctrl, float kp, float max_speed) {
    ctrl->Kp = kp;
    ctrl->MaxSpeed = max_speed;
    ctrl->TargetPos = 0.0f;
    ctrl->CurrentPos = 0.0f;
    ctrl->Error = 0.0f;
    ctrl->OutSpeed = 0.0f;
}
/* ==========================================
 * 计算函数 (建议在定时器中断中调用，例如 1kHz)
 * ========================================== */
float P_Control_Compute(P_Controller *ctrl, float target_pos, float current_pos) {

	float output;
    // 1. 更新状态
    ctrl->TargetPos = target_pos;
    ctrl->CurrentPos = current_pos;

    // 2. 计算误差
    ctrl->Error = ctrl->TargetPos - ctrl->CurrentPos;

    // 3. 纯 P 计算 (核心公式)
    if((ctrl->Error < 0.25f) && (ctrl->Error > -0.25f))
    	{output = 0;}
    else
		{output = ctrl->Kp * ctrl->Error;}

    // 4. 输出限幅 (Saturation)
    // 防止计算出的速度超过电机或驱动器的物理极限
    if (output > ctrl->MaxSpeed) {
        output = ctrl->MaxSpeed;
    } else if (output < -ctrl->MaxSpeed) {
        output = -ctrl->MaxSpeed;
    }

    // 5. 保存输出并返回
    ctrl->OutSpeed = output;
    return output;
}

/**
  * @brief  DCDC高压模块供电开关
  * @param  true 供电 false 关闭电源
  */
void DC_Power_CTR(bool state)
{
	if(state == true)
	{
		DC_Power_ON;
		DC_ON_State = true;
//		 printf("%d\r\n",DC_ON_State);
	}
	else
	{
		DC_Power_OFF;
		DC_ON_State = false;
//		 printf("%d\r\n",DC_ON_State);
	}
}
/**
  * @brief  设置电机速度和方向
  * @param  speed_hz: 目标频率, 单位 Hz。正数代表正转，负数代表反转。
  */
void set_speed(float speed_hz)
{
    //明确设置模式为速度模式 ***
    Motor_mode = MOTOR_OPEN_SPEED;
    if (speed_hz >= 0)
    {
        motor_direction = MOTOR_FORWARD;
        // 使用 speed_hz 的绝对值来计算增量
        float increment_f = (speed_hz / interrupt_freq_hz) * PHASE_2_32;
        phase_increment = (uint32_t)increment_f;
    }
    else
    {
        motor_direction = MOTOR_REVERSE;
        float increment_f = (-speed_hz / interrupt_freq_hz) * PHASE_2_32;
        phase_increment = (uint32_t)increment_f;
    }
}
/**
  * @brief  设置电机移动到目标位置 (开环)
  * @param  target_steps: 目标绝对步数
  * @param  speed_hz: 移动到目标位置时使用的速度 (Hz, 必须为正数)
  */
void move_to_position(int32_t target_steps, float speed_hz)
{
	Motor_mode = MOTOR_OPEN_POSITION;
    // 1. 更新目标位置
    target_step_position = target_steps;

    // 3. 根据 speed_hz 计算并设置在位置模式下移动时要使用的速度增量
    set_speed(speed_hz);
//    if (speed_hz < 0) speed_hz = -speed_hz; // 确保速度为正
//    double increment_f = (speed_hz / interrupt_freq_hz) * PHASE_2_32;
//    position_mode_increment = (uint32_t)increment_f;
}
/**
  * @brief  启动往复运动模式
  * @param  pos1: 往返点1 (绝对步数)
  * @param  pos2: 往返点2 (绝对步数)
  * @param  count: 往返次数 (从pos1->pos2->pos1 算一次)
  * @param  speed_hz: 往返时使用的速度 (Hz, 必须为正数)
  */
void move_repeatedly(int32_t pos1, int32_t pos2, uint32_t count, float speed_hz)
{
    // 1. 存储往复运动参数
    repeated_pos_A = pos1;
    repeated_pos_B = pos2;
    repeated_count_total = count;
    repeated_speed_hz = speed_hz;
    repeated_count_current = 0; // 重置当前计数

    // 2. 切换到往复运动模式
    Motor_mode = MOTOR_OPEN_REPEATED;

    // 3. 【关键修复】: 直接设置第一个目标和速度，避免模式冲突
    target_step_position = repeated_pos_A;
    set_speed(speed_hz);
//    if (repeated_speed_hz < 0) repeated_speed_hz = -repeated_speed_hz;
//    double increment_f = (repeated_speed_hz / interrupt_freq_hz) * PHASE_2_32;
//    position_mode_increment = (uint32_t)increment_f;
}
/**
  * @brief  启动一次单次定位任务
  */
void start_move_to_position(int32_t target_steps, float speed_hz)
{
    Motor_mode = MOTOR_OPEN_POSITION;
    move_to_position(target_steps, speed_hz);
}
/**
  * @brief  执行一次相对步进移动
  * @param  relative_steps: 需要移动的相对步数 (正数向前, 负数向后)
  * @param  speed_hz: 本次步进移动时使用的速度 (Hz, 必须为正数)
  */
void step_move(int32_t relative_steps, float speed_hz)
{

    int32_t current_pos;

    // 【重要】安全地读取当前的位置计数器
    __disable_irq();
    current_pos = absolute_step_counter;
    __enable_irq();

    // 计算出本次移动的绝对目标位置
    int32_t target_pos = current_pos + relative_steps;

    // 调用已经写好的单次定位函数来执行这次移动
    move_to_position(target_pos, speed_hz);
}

void Close_output(void)
{
	 HAL_TIM_Base_Stop(&htim16);//dma printf
	DC_Power_OFF;
	    	    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	    	    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
	    	    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	    	    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
	    	    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	    	    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
}

void Update_output(uint8_t new_step)
{
    // 安全第一：在改变状态前，先关闭所有6个通道的输出
    // 这确保了在切换瞬间不会有意外的导通

    switch(new_step)
    {
//    	        	        case 0: // Step 1: U+ V- (A->B)
//    	        	    	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 10000);
//    	        	    	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
//    	        	            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);       // U相上桥臂输出
//    	        	            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
//    	        	            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);		// V相下桥臂导通
//    	        	            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
//    	        	            break;
//
//    	        	        case 1: // Step 2: W+ V- (C->B)
//    	        	    	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
//    	        	    	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 10000);
//    	        	            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);		// V相下桥臂导通
//    	        	            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
//    	        	            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
//    	        	            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);    // W相下桥臂导通
//    	        	            break;
//
//    	        	        case 2: // Step 3: W+ U- (C->A)
//    	        	    	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//    	        	    	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 10000);
//    	        	            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);       // U相上桥臂输出
//    	        	            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
//    	        	            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
//    	        	            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);    // W相下桥臂导通
//    	        	            break;
//
//    	        	        case 3: // Step 4: V+ U- (B->A)
//    	        	    	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//    	        	    	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 10000);
//    	        	            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);       // U相上桥臂输出
//    	        	            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
//    	        	            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);		// V相下桥臂导通
//    	        	            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
//    	        	            break;
//
//    	        	        case 4: // Step 5: V+ W- (B->C)
//    	        	    	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 10000);
//    	        	    	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
//    	        	            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);		// V相下桥臂导通
//    	        	            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
//    	        	            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
//    	        	            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);    // W相下桥臂导通
//    	        	            break;
//
//    	        	        case 5: // Step 6: U+ W- (A->C)
//    	        	    	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 10000);
//    	        	    	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
//    	        	            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);       // U相上桥臂输出
//    	        	            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
//    	        	            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
//    	        	            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);    // W相下桥臂导通
//    	        	            break;
        case 0: // Step 1: U+ V- (A->B)
    	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_TIM1);
    	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty_TIM1);
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);       // U相上桥臂输出
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);		// V相下桥臂导通
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);    // W相下桥臂导通
            break;

        case 1: // Step 2: W+ V- (C->B)
    	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_TIM1);
    	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);       // U相上桥臂输出
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);		// V相下桥臂导通
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);    // W相下桥臂导通
            break;

        case 2: // Step 3: W+ U- (C->A)
    	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_TIM1);
    	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty_TIM1);
    	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);       // U相上桥臂输出
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);		// V相下桥臂导通
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);    // W相下桥臂导通
            break;

        case 3: // Step 4: V+ U- (B->A)
    	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty_TIM1);
    	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);       // U相上桥臂输出
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);		// V相下桥臂导通
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);    // W相下桥臂导通
            break;

        case 4: // Step 5: V+ W- (B->C)
    	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty_TIM1);
    	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty_TIM1);
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);       // U相上桥臂输出
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);		// V相下桥臂导通
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);    // W相下桥臂导通
            break;

        case 5: // Step 6: U+ W- (A->C)
    	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty_TIM1);
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);       // U相上桥臂输出
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);		// V相下桥臂导通
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);    // W相下桥臂导通
            break;
    }
}










#endif













