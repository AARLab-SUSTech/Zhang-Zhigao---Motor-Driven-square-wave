#include "Bsp_Adc.h"
#include "stm32g4xx_hal.h"

//外部声明Adc句炳，定义在Main.c中
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
//外部声明Tim句炳，定义在Main.c中
extern TIM_HandleTypeDef htim7;

static volatile uint32_t Adc1_Raw_Data[2];
static volatile uint32_t Adc2_Raw_Data[3];

void Bsp_Adc_Init(void)
{
	//ADC校准
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_DIFFERENTIAL_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    //开启ADC数据DMA自动搬运  (uint32_t *) 为数据类型强制转换
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)Adc1_Raw_Data, 2);// HV_V HV_I  	采集高压母线电压和电流数据
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)Adc2_Raw_Data, 3);//DC_I SIN COS	采集低压DC电流和磁栅传感器正交信号值
    //开启定时器触发Adc Dma自动采样和搬运
    Bsp_Adc_Trigger_Timer_Start();
}

void Bsp_Adc_Trigger_Timer_Start(void)
{
    HAL_TIM_Base_Start_IT(&htim7);//Adc Dma外部触发定时器
}

uint16_t Bsp_Get_Hv_Raw_Data(void)
{
	return (uint16_t)Adc1_Raw_Data[0];
}

uint16_t Bsp_Get_Hi_Raw_Data(void)
{
	return (uint16_t)Adc1_Raw_Data[1];
}

uint16_t Bsp_Get_Dc_I_Raw_Data(void)
{
	return (uint16_t)Adc2_Raw_Data[0];
}

uint16_t Bsp_Get_Position_Sensor_Sin_Raw_Data(void)
{
    return (uint16_t)Adc2_Raw_Data[1];
}

uint16_t Bsp_Get_Position_Sensor_Cos_Raw_Data(void)
{
    return (uint16_t)Adc2_Raw_Data[2];
}



























