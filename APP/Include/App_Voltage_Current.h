#ifndef _APP_VOLTAGE_SENSOR_H
#define _APP_VOLTAGE_SENSOR_H






void App_Upate_Voltage_Current_Data(void);


/**
 * @brief  系统级安全与故障监控任务 (APP 层)
 * @note   负责监控高压母线与低压供电的电压/电流状态。
 * 包含滑动窗口滤波的软保护 (防误触)，以及绝对阈值的硬保护 (极速熔断)。
 * 建议在控制主循环或高频定时器任务中周期调用 (如 1kHz)。
 */
void App_System_Safety_Monitor(void);






#endif
