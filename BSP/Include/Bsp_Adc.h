#ifndef _BSP_ADC_H
#define _BSP_ADC_H

#include "stdint.h"

typedef struct {
    uint16_t Sin;
    uint16_t Cos;
} Adc_SinCos_t;



void Bsp_Adc_Init(void);

void Bsp_Adc_Trigger_Timer_Start(void);
uint16_t Bsp_Get_Hv_Raw_Data(void);

uint16_t Bsp_Get_Hi_Raw_Data(void);

uint16_t Bsp_Get_Dc_I_Raw_Data(void);

uint16_t Bsp_Get_Position_Sensor_Sin_Raw_Data(void);
uint16_t Bsp_Get_Position_Sensor_Cos_Raw_Data(void);














#endif


