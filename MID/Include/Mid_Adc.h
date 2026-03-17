#ifndef _MID_ADC_H
#define _MID_ADC_H




/**
 * @brief  获取高压母线电压值
 * @retval float 实际电压值，单位：V
 */
float Mid_Get_Hv_V(void);

/**
 * @brief  获取高压母线电流
 * @retval float 实际电流值，单位：uA
 */
float Mid_Get_Hi_uA(void);

/**
 * @brief  获取低压 DC 电流
 * @retval float 实际电流值，单位：A
 */
float Mid_Get_Dc_I_A(void);




#endif


