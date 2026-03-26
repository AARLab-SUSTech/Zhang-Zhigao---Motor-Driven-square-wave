#include "App_Button.h"

#include "stm32g4xx_hal.h"
#include "stdio.h"
#include "Bsp_Button.h"

#include "App_Fault.h"
#include "App_EFA.h"
#include <Mid_Control.h>
#include "Mid_Command_Usart.h"

#include "Bsp_Control.h"
#include "App_Buzzer.h"

Button_t Button1;
extern Motor EFA_DATA;


/* ==================================================== *
 * 延时启动控制相关的静态变量 (非阻塞状态机)
 * ==================================================== */
static bool     Motor_Pending_Start = false; // 是否处于“等待延时启动”阶段
static uint32_t Motor_Start_Tick    = 0;     // 记录计划启动的时间戳
#define DELAY_START_MS  600                 // 设定长按松开后，延迟 2 秒 (2000ms) 启动

static float Button_Set_Fre = 10.0f;

void App_Button_Init(void)
{
	  Button_Create("Button1",				//按键名字
	                &Button1, 				//按键句柄
	                Read_Button1_Level, 	//按键电平检测函数接口
	                1);		   	//触发电平

	  Button_Attach(&Button1,BUTTON_DOWM,Btn1_Down_CallBack);		//按键单击
	  Button_Attach(&Button1,BUTTON_DOUBLE,Btn1_Double_CallBack);	//双击
	  Button_Attach(&Button1,BUTTON_LONG_FREE,Btn1_Long_Free_CallBack);		//长按
}

uint8_t Read_Button1_Level(void)
{
  return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);
}

/* ==================================================== *
 * 1. 单击事件：根据电机状态决定是“急停”还是“调频”
 * ==================================================== */
void Btn1_Down_CallBack()
{
    /* 状态判断：如果电机正在运行，或者正在“倒计时准备启动” */
    if ((EFA_DATA.Motor_mode != MOTOR_IDLE) || Motor_Pending_Start)
    {
        App_Buzzer_Beep(200); // 急停长提示音

        /* 1. 取消正在倒计时的启动任务 */
        Motor_Pending_Start = false;

        /* 2. 状态机切回 IDLE，并立刻硬件封锁输出 (极其重要) */
        EFA_DATA.Motor_mode = MOTOR_IDLE;
        Bsp_Close_All_Output();

        printf("EMERGENCY STOP! Motor Halted.\r\n");
    }
    else
    {
        /* 状态判断：电机处于停止状态，允许修改频率 */
        App_Buzzer_Beep(50); // 调频短提示音

        Button_Set_Fre += 10.0f;
        if(Button_Set_Fre >= 200.0f)  Button_Set_Fre = 200.0f;

        Mid_Set_Fre_Ele(Button_Set_Fre);
        printf("Single Click! Target Freq: %.1f\r\n", Button_Set_Fre);
    }
}


/* ==================================================== *
 * 2. 双击事件：仅在停止状态下允许降频
 * ==================================================== */
void Btn1_Double_CallBack()
{
    /* 安全锁：运行期间禁止双击调频 */
    if ((EFA_DATA.Motor_mode != MOTOR_IDLE) || Motor_Pending_Start) {
        return;
    }

    App_Buzzer_Beep(100);

    Button_Set_Fre -= 10.0f;
    if(Button_Set_Fre <= 1.0f)    Button_Set_Fre = 1.0f;

    Mid_Set_Fre_Ele(Button_Set_Fre);
    printf("Double Click! Target Freq: %.1f\r\n", Button_Set_Fre);
}

/* ==================================================== *
 * 3. 长按松开事件：触发延时启动倒计时
 * ==================================================== */
void Btn1_Long_Free_CallBack()
{
    /* 如果已经在运行了，就不要重复触发启动 */
    if (EFA_DATA.Motor_mode != MOTOR_IDLE) return;

    App_Buzzer_Beep(600); // 发出 0.5 秒的长鸣，警告操作员即将启动

    /* 标记等待启动，并记录未来要启动的时间戳 (当前时间 + 延时) */
    Motor_Pending_Start = true;
    Motor_Start_Tick = HAL_GetTick() + DELAY_START_MS;

    printf("Motor will start in %d ms...\r\n", DELAY_START_MS);
}

/**
 * @brief  延时启动后台扫描任务 (必须放在 main 的 while(1) 中不断调用)
 */
void App_Button_Task(void)
{
    /* 如果处于“等待启动”状态，并且当前系统时间已经超过了设定的启动时间 */
    if (Motor_Pending_Start && (HAL_GetTick() >= Motor_Start_Tick))
    {
        /* 1. 清除等待标志 */
        Motor_Pending_Start = false;

        /* 2. 真正执行开环运动 */
        Bsp_Dc_Power_Control(true);
        EFA_DATA.Motor_mode = MOTOR_OPEN_REPEATED;
        move_repeatedly(0, 60, 50, Button_Set_Fre);

        printf("Delay finished. Motor STARTED!\r\n");
    }
}

















