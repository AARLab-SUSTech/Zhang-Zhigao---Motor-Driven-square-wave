#include <Bsp_Ad7190.h>
#include "Bsp_Force_Sensor.h"
#include "stdio.h"


volatile Force_sensor Force_Sensor1;

void Bsp_Force_Sensor_Init(void)
{
	  Force_Sensor1.weight_proportion=86742;  // 电压值与重量变换比例，这个需要实际测试计算才能得到
	  Force_Sensor1.weight_Zero_Data=0;   	  // 零值

	  Force_sensor_init();
	  weight_ad7190_conf();

	  HAL_Delay(500);
	  Force_Sensor1.weight_Zero_Data = weight_ad7190_ReadAvg(6);
	  printf("zero:%ld\n",Force_Sensor1.weight_Zero_Data);

	  Force_Sensor1.RAW_Data=weight_ad7190_ReadAvg(1);
	  Force_Sensor1.weight_g=(Force_Sensor1.RAW_Data-Force_Sensor1.weight_Zero_Data)*1000/Force_Sensor1.weight_proportion;
}













