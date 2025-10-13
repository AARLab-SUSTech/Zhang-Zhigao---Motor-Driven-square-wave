#include "FOC.h"
/**
 * @brief 将角度归一化到 [0, 6*PI) 区间 (float 版本)
 *
 * @param angle 输入的角度 (弧度)，可以是任意实数
 * @return float 归一化后的角度，范围在 [0, 6*PI)
 */
float normalize_angle_0_to_6pi_f32(float angle) {
    // 1. 使用 fmodf 计算 angle 除以 2*PI 的余数
    //    fmodf 的结果符号与被除数 angle 相同，范围是 (-2*PI, 2*PI)
    float remainder = fmodf(angle, 6.0f * PI);

    // 2. 如果余数是负数，则加上 6*PI 使其变为正数
    if (remainder < 0.0f) {
        remainder += 6.0f * PI;
    }

    return remainder;
}
/**
 * @brief 将角度归一化到 [0, 2*PI) 区间 (float 版本)
 *
 * @param angle 输入的角度 (弧度)，可以是任意实数
 * @return float 归一化后的角度，范围在 [0, 2*PI)
 */
float normalize_angle_0_to_2pi_f32(float angle) {
    // 1. 使用 fmodf 计算 angle 除以 2*PI 的余数
    //    fmodf 的结果符号与被除数 angle 相同，范围是 (-2*PI, 2*PI)
    float remainder = fmodf(angle, 2.0f * PI);

    // 2. 如果余数是负数，则加上 6*PI 使其变为正数
    if (remainder < 0.0f) {
        remainder += 2.0f * PI;
    }

    return remainder;
}
void DAC_Set_Voltage(float Va,float Vb,float Vc)
{
//	float temp_Va,temp_Vb,temp_Vc;
//
//	if(Va >= MAX_Voltage) 				Va = MAX_Voltage;
//		else if(Va <= (-MAX_Voltage))	Va = -MAX_Voltage;
//	if(Vb >= MAX_Voltage) 				Vb = MAX_Voltage;
//		else if(Vb <= (-MAX_Voltage))	Vb = -MAX_Voltage;
//	if(Vc >= MAX_Voltage) 				Vc = MAX_Voltage;
//		else if(Vc <= (-MAX_Voltage))	Vc = -MAX_Voltage;
//
//	temp_Va = Va/OPAMP_GAIN;
//	temp_Vb = Vb/OPAMP_GAIN;
//	temp_Vc = Vc/OPAMP_GAIN;
//
//	DAC8568_Set_voltage(0,Va/1000.0f);
//	DAC8568_Set_voltage(1,Vb/1000.0f);
//	DAC8568_Set_voltage(2,Vc/1000.0f);

}
