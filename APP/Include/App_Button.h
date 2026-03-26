#ifndef _APP_BUTTON_H
#define _APP_BUTTON_H

#include "stdint.h"

void App_Button_Init(void);

uint8_t Read_Button1_Level(void);
void Btn1_Down_CallBack();
void Btn1_Double_CallBack();
void Btn1_Long_Free_CallBack();

/**
 * @brief  延时启动后台扫描任务 (必须放在 main 的 while(1) 中不断调用)
 */
void App_Button_Task(void);

#endif


