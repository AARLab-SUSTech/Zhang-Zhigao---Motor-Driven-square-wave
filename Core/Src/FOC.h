/*
 * FOC.h
 *
 *  Created on: May 14, 2025
 *      Author: 16964
 */

#ifndef SRC_FOC_H_
#define SRC_FOC_H_

#include "main.h"

#define OPAMP_GAIN (1200.0f)
#define MAX_Voltage 1200.0f
#define Vd_MAX_Voltage 1200.0f
#define Vq_MAX_Voltage 1200.0f

#define sqrt3_2  0.8660254037844386f
#define sqrt3  	 1.732050807568877f

// --- Calibration Parameters (USER MUST TUNE THESE CAREFULLY!) ---


#define CAL_ALIGN_VD_REQUEST            (600.0f) // Vd voltage for rotor alignment (e.g., 20-50% of max V)
#define CAL_ALIGN_VQ_REQUEST            (0.0f)
#define CAL_ALIGN_DURATION_MS           (2000)   // Hold alignment voltage for this duration (ms)
#define CAL_POST_ALIGN_PAUSE_MS         (1000)   // Pause after alignment before sweeping

#define CAL_SWEEP_VD_REQUEST            (0.0f)   // Usually Vd=0 for sweeping with Vq
#define CAL_SWEEP_VQ_FORWARD            (600.0f) // Vq for forward sweep (positive direction)
#define CAL_SWEEP_VQ_REVERSE            (-600.0f)// Vq for reverse sweep (negative direction)
                                                 // Or use positive Vq and negative Target_speed

#define CAL_SWEEP_TARGET_ELEC_SPEED_RAD_S (5.0f * M_PI) // Electrical speed for sweeping (rad/s)
                                                        // Adjust for slow, controlled movement.
                                                        // MOTOR1.Target_speed in your code seems to be elec rad/s.
                                                        // The factor 0.00005f for Theta_elec update implies TIM16 freq is 20kHz.
                                                        // So Target_speed * 0.00005 = rad per 50us.
                                                        // Target_speed = desired_rad_per_sec / (1 / (TIM16_Period_s))
                                                        // If TIM16 freq = 20kHz (50us period)
                                                        // Then CAL_SWEEP_TARGET_ELEC_SPEED_RAD_S / 20000.0f would be more direct if Target_speed is this scaled value.
                                                        // Given your usage: MOTOR1.Target_speed = 10*2*3.14159265f;
                                                        // This is already in rad/s. Let's keep it simple.
                                                        // The 0.00005f is 1/20000, which is the TIM16 period if its clock is high and ARR is low.
                                                        // TIM16 Prescaler=16, Period=499. SystemClock likely 170MHz for HSI PLL.
                                                        // TIM16 CLK = 170MHz / (16+1) = 10 MHz. Period = (499+1) / 10MHz = 500 / 10e6 = 50us. Freq = 20kHz.
                                                        // So, `0.00005f` is correct for `TIM16_period_seconds`.

#define CAL_SWEEP_MAX_DURATION_PER_DIR_MS (8000) // Max time for one sweep direction (ms)
#define CAL_STALL_MOVEMENT_THRESHOLD_MM   (0.05f) // If pos changes less than this, assume stall/limit (mm)
#define CAL_STALL_CHECK_INTERVAL_MS       (500)   // Check for stall every this interval (ms)
#define CAL_POLARITY_CHECK_MOVEMENT_MM    (0.2f)  // Minimum movement to confirm polarity (mm)


// 电机控制模式
typedef enum {
    MOTOR_MODE_RELEASE = 0,         // 电机释放/禁用/自由旋转
    MOTOR_MODE_OPEN_LOOP_POSITION,  // 开环位置模式（保持在指定角度）
    MOTOR_MODE_OPEN_LOOP_STEP,      // 开环步进模式
    MOTOR_MODE_OPEN_LOOP_RECIPROCATE, // 开环往复模式
    MOTOR_MODE_VOLTAGE_OPEN_LOOP,   // 电压开环控制模式
    MOTOR_MODE_VOLTAGE_POSITION,    // 电压位置模式
    MOTOR_MODE_TORQUE,              // 转矩控制模式 (通常是 Iq 控制)
    MOTOR_MODE_SPEED,               // 速度闭环控制模式
    MOTOR_MODE_POSITION,            // 位置闭环控制模式 (需要额外的位置环PI)
    MOTOR_MODE_CALIBRATION,         // 电机参数或传感器校准模式
	MOTOR_MODE_ALIGN_D_AXIS,
    MOTOR_MODE_STOPPING,            // 主动停止中
    MOTOR_MODE_FAULT,               // 故障状态
	MOTOR_MODE_OVER_CURRENT,
	MOTOR_MODE_IDLE
} Motor_Control_Mode_e;

// 编码器正反转
typedef enum {
    ENCODER_CW = 0,
	ENCODER_CCW,
	ENCODER_UNKNOW
} ENCODER_DIR;
// 编码器正反转
typedef enum {
    ENCODER_ABZ = 0,
	ENCODER_UVW,
	ENCODER_ABS,
	ENCODER_HALL
} ENCODER_MODE;

// --- Calibration State Machine ---
typedef enum {
    CAL_STATE_IDLE,
    CAL_STATE_START_ALIGNMENT,
    CAL_STATE_ALIGNING,
    CAL_STATE_POST_ALIGNMENT_PAUSE,
    CAL_STATE_START_POSITIVE_SWEEP, // To find max position and check polarity
    CAL_STATE_SWEEPING_POSITIVE,
    CAL_STATE_START_NEGATIVE_SWEEP, // To find min position
    CAL_STATE_SWEEPING_NEGATIVE,
    CAL_STATE_DONE,
    CAL_STATE_ERROR
} CalibrationState_t;

typedef struct {

	uint16_t dac_8568_raw_data[3];

	//DCDC电源高压电压及输入电流原始数据
	uint32_t ADC_HV_P_RAW[3];
	uint32_t ADC_HV_N_RAW[3];
	uint32_t ADC_DC_I_P_RAW[3];
	uint32_t ADC_DC_I_N_RAW[3];
	//DCDC电源高压电压及输入电流转换数据
	float ADC_HVP[3];
	float ADC_HVN[3];
	float ADC_I_P[3];
	float ADC_I_N[3];

	float Theta_elec;//电角度（0-2pi）
	float Target_speed;//目标速度
	float Vd;
	float Vq;
	float V_alpha[1];
	float V_beta[1];
	float V_MAX_amplitude;
	float DC_Voltage;

	float inv_clark_VA[1];
	float inv_clark_VB[1];
	float inv_clark_VC[1];

    float A_Voltage;
    float B_Voltage;
    float C_Voltage;

    float A_DAC1_CH1_V;
    float B_DAC1_CH2_V;
    float C_DAC2_CH1_V;

    uint32_t A_DAC1_CH1_COUNT;
    uint32_t B_DAC1_CH2_COUNT;
    uint32_t C_DAC2_CH1_COUNT;

    float HVOUT_A_V;
    float HVOUT_B_V;
    float HVOUT_C_V;
    float HVOUT_D_V;
    float LAST_Current_A_uA;
    float LAST_Current_B_uA;
    float LAST_Current_C_uA;
    float Current_A_uA;
    float Current_B_uA;
    float Current_C_uA;
    float Current_alpha_uA[1];
    float Current_beta_uA[1];
    float Current_Id_uA[1];
    float Current_Iq_uA[1];

    float Voltage_A_kV;
    float Voltage_B_kV;
    float Voltage_C_kV;

    uint32_t sin_cos_ADC_RAW[2];
    float sin_V;
    float cos_V;
    float sin_offset;
    float cos_offset;
	float theta_radians;
	float theta_degrees;
	float theta_ele_from_mech;
	float electrical_offset_rad;
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

    uint8_t encoder_polarity;         // Default assumption, will be verified
    float mechanical_angle_at_alignment_rad;

    bool Min_Max_cal_status;
    bool theta_offset_cal_status;

    CalibrationState_t Cal_state;

    Motor_Control_Mode_e MOTOR_MODE;

    bool THI;

    float step_end_theta_elec;//步进模式目标值
    float target_theta_elec;//开环目标电角度值

    /* 串级控制内部传递的目标值 (重要) */
    float internal_Speed_rpm_Target; // 位置环输出的速度目标，给速度环用
    float internal_Iq_Target;        // 速度环输出的Iq目标，给电流环用
    float internal_Id_Target;        // 通常为0，速度环设定，给电流环用
    float recip_theta_A;
    float recip_theta_B;
    bool recip_moving_to_B;
    bool recip_is_homing;
    uint16_t recip_target_cycles;
    uint16_t recip_current_cycle;


    /* PI控制器状态 (如果需要存储) */
    struct {
    	float error_sum_q;
    	float prev_error_q;
    	float Kp;
    	float Ki;
    	float Kd;
    	float integral_limit;
    	float output_limit;
    } Iq_PI;

    struct {
    	float error_sum_d;
    	float prev_error_d;
    	float integral_limit;
    	float Kp;
    	float Ki;
    	float Kd;
    	float output_limit;
    } Id_PI;

    struct {
    	float kp;
    	float ki;
    	float error_sum_speed;
    	float prev_error_speed;
    } Speed_PI;

    struct {
    	float kp;                 // 位置环比例增益
        float kd;                // (可选) 微分增益
        float error_sum_pos_rad;
    } Position_P;
    float Id_ref;              // d轴期望电流 (mA)
    float Iq_ref;              // q轴期望电流 (mA)

}Motor;


float normalize_angle_0_to_2pi_f32(float angle);
float normalize_angle_0_to_6pi_f32(float angle);


#endif /* SRC_FOC_H_ */
