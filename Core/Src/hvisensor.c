/*
 * hvisensor.c
 *
 *  Created on: Sep 3, 2025
 *      Author: Letian
 */

#include "hvisensor.h"
#include "control.h"
#include "main.h"

extern volatile Motor_mode_t Motor_mode;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1)
    {
//    	int temp_data[2];
//    	temp_data[0] = (ADC1_RAW_data[0] - 2048) * 2;
//    	temp_data[1] = (ADC1_RAW_data[1] - 2048) * 2;
//    	//V  mA
//    	HV_V = temp_data[0] * 1.00909423828125f;//HV_V = (temp_data[0] / 4096) * 3.3f * 2 * 501 / 0.4f
//        HV_I = temp_data[1] * 0.005035400390625f;//HV_I = (temp_data[1] / 4096) * 3.3 / 8 / 20 * 1000

     	//printf("%.3f,%.3f\r\n",HV_V,HV_I);
    }
    if(hadc->Instance == ADC2)
    {
    	DC_I = ADC2_RAW_data[0] * 0.0008056640625f;//(ADC2_RAW_data[0] / 4096) * 3.3 / 200 / 0.005
     	//printf("%.3f,%ld,%ld\r\n",DC_I,ADC2_RAW_data[1],ADC2_RAW_data[2]);
    }
}

void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp)
{
	if(hcomp == &hcomp1)
	{
		Motor_mode = MOTOR_OVER_HV_VOLTAGE;
		Close_output();
//		Buzzer_ON
//		if(DC_ON_State == true) { DC_Power_CTR(false); }
//		printf("HV error\r\n");
	}
	if(hcomp == &hcomp2)
	{
		Motor_mode = MOTOR_OVER_HV_CURRENT;
		Buzzer_ON
		Close_output();
//		if(DC_ON_State == true) {  DC_Power_CTR(false); }
//		printf("HV I \r\n");
	}
}


