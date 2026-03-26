#include <App_EFA.h>
#include "App_Control.h"
#include "stm32g4xx_hal.h"

#include "Bsp_Control.h"

#include "App_Led.h"
#include "App_Can.h"
#include "App_Voltage_Current.h"
#include "App_Position_Sensor.h"

extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

#define Over_Current_Sensor_Enable

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(htim == &htim6)
	{

	#ifdef Over_Current_Sensor_Enable
		App_Upate_Voltage_Current_Data();//更新电压电流数据

		App_System_Safety_Monitor();//安全监测
	#endif
		App_Update_Position_Sensor_Data();//磁栅尺位移传感器数据更新

		App_EFA_Motion_Task();//输出计算

		App_EFA_Commutation_Task();//更新输出

	}

	if(htim == &htim8)
	{
		HAL_TIM_Base_Stop(&htim8);//高频计算中断
		Bsp_Buzzer_Control(false);
	}

	if(htim == &htim15)
	{
		App_Led_Task_1ms();//Led任务 1Khz中断
	}

	if(htim == &htim16)//用于往复模式下打印往复次数以及电流电压等
	{
		App_Can_Send_Data();//Can报文定时反馈
	}

	if(htim == &htim17)
	{
		App_Can_Heart_Send();//Can心跳发送
	}

}





