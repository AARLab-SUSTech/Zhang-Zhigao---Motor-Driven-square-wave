#include "App_Control.h"
#include "stm32g4xx_hal.h"

#include "App_Led.h"
#include "App_Can.h"
#include "App_EMA.h"
#include "App_Voltage_Current.h"
#include "App_Position_Sensor.h"

extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(htim == &htim15)
	{
		App_Led_Task_1ms();
	}

	if(htim == &htim16)//用于往复模式下打印往复次数以及电流电压等
	{
		App_Can_Send_Data();
	}

	if(htim == &htim6)
	{

		App_Upate_Voltage_Current_Data();

		App_System_Safety_Monitor();

		App_Update_Position_Sensor_Data();

		App_EMA_Motion_Task();

		App_EMA_Commutation_Task();

	}

	if(htim == &htim17)
	{
		App_Can_Heart_Send();//Can Heart
	}

}





