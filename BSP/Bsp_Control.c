/**
 ******************************************************************************
 * @file    Bsp_Control.c
 * @author  Letian
 * @date    [填入当前日期]
 * @brief   底层硬件驱动支持包 (BSP)
 * @note    负责高压 DCDC 电源的启停、PWM 输出通道的紧急封锁，
 * 以及电机三相桥臂的六步换向底层驱动操作。
 * 绝不包含任何运动规划或状态机等业务逻辑。
 ******************************************************************************
 */

#include "Bsp_Control.h"
#include "stm32g4xx_hal.h"

uint8_t Dc_Power_State = 0;

/* ========================================================================== *
 * 全局变量声明 (如在头文件已声明，此处可按需保留或删除)
 * ========================================================================== */
extern bool DC_ON_State;
extern uint32_t duty_TIM1;


extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim16;

/* ========================================================================== *
 * 【BSP 层】电源与安全控制接口
 * ========================================================================== */

/**
 * @brief  紧急关闭所有底层输出 (硬件级安全切断)
 * @note   封锁高压 DCDC 并强行关闭 TIM1 的所有六个 PWM 桥臂通道。
 */
void Bsp_Close_All_Output(void)
{
    /* 1. 停止 DMA 打印定时器 */
    HAL_TIM_Base_Stop(&htim16);

    /* 2. 切断高压电源 */
    Bsp_Dc_Power_Control(false);

    /* 3. 彻底关闭三相六个桥臂的 PWM 输出 */
    HAL_TIM_PWM_Stop(&htim1,   TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);

    HAL_TIM_PWM_Stop(&htim1,   TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);

    HAL_TIM_PWM_Stop(&htim1,   TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
}


/* ========================================================================== *
 * 【BSP 层】电机换向执行接口
 * ========================================================================== */

/**
 * @brief  三相无刷电机 (BLDC) 六步换向底层驱动
 * @param  new_step 新的换向节拍 (0~5)
 * @note   在切换前先关闭所有通道，防止上下桥臂直通短路 (Shoot-through)。
 */
void Bsp_Update_Output(uint8_t new_step)
{
    /* 安全第一：在改变状态前，先关闭所有 6 个通道的输出 */
    HAL_TIM_PWM_Stop(&htim1,   TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1,   TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1,   TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

    /* 执行六步换向 (矩阵式排版，便于核对硬件真值表) */
    switch(new_step)
    {
        case 0: /* Step 1: U+ V- (A->B) */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_TIM1);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty_TIM1);

            HAL_TIM_PWM_Start(&htim1,   TIM_CHANNEL_1);     HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
            HAL_TIM_PWM_Start(&htim1,   TIM_CHANNEL_2);     HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
            HAL_TIM_PWM_Start(&htim1,   TIM_CHANNEL_3);     HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
            break;

        case 1: /* Step 2: W+ V- (C->B) */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_TIM1);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

            HAL_TIM_PWM_Start(&htim1,   TIM_CHANNEL_1);     HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
            HAL_TIM_PWM_Start(&htim1,   TIM_CHANNEL_2);     HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
            HAL_TIM_PWM_Start(&htim1,   TIM_CHANNEL_3);     HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
            break;

        case 2: /* Step 3: W+ U- (C->A) */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_TIM1);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty_TIM1);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

            HAL_TIM_PWM_Start(&htim1,   TIM_CHANNEL_1);     HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
            HAL_TIM_PWM_Start(&htim1,   TIM_CHANNEL_2);     HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
            HAL_TIM_PWM_Start(&htim1,   TIM_CHANNEL_3);     HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
            break;

        case 3: /* Step 4: V+ U- (B->A) */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty_TIM1);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

            HAL_TIM_PWM_Start(&htim1,   TIM_CHANNEL_1);     HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
            HAL_TIM_PWM_Start(&htim1,   TIM_CHANNEL_2);     HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
            HAL_TIM_PWM_Start(&htim1,   TIM_CHANNEL_3);     HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
            break;

        case 4: /* Step 5: V+ W- (B->C) */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty_TIM1);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty_TIM1);

            HAL_TIM_PWM_Start(&htim1,   TIM_CHANNEL_1);     HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
            HAL_TIM_PWM_Start(&htim1,   TIM_CHANNEL_2);     HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
            HAL_TIM_PWM_Start(&htim1,   TIM_CHANNEL_3);     HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
            break;

        case 5: /* Step 6: U+ W- (A->C) */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty_TIM1);

            HAL_TIM_PWM_Start(&htim1,   TIM_CHANNEL_1);     HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
            HAL_TIM_PWM_Start(&htim1,   TIM_CHANNEL_2);     HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
            HAL_TIM_PWM_Start(&htim1,   TIM_CHANNEL_3);     HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
            break;
    }
}

/**
 * @brief  控制高压 DCDC 电源状态
 * @param  state true: 开启高压供电, false: 切断高压供电
 */
void Bsp_Dc_Power_Control(bool state)
{
    if (state == true)
    {
        /* 硬件层：拉高 GPIOB_PIN_9 */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
        Dc_Power_State = 1;
    }
    else
    {
        /* 硬件层：拉低 GPIOB_PIN_9 */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
        Dc_Power_State = 0;
    }
}

/**
 * @brief  返回高压 DCDC 电源状态
 * @param  state 1: 开启高压供电, 0: 切断高压供电
 */
uint8_t Bsp_Get_Dc_Power_State(void)
{
	return Dc_Power_State;
}

/**
 * @brief  控制蜂鸣器报警状态
 * @param  state true: 开始鸣叫, false: 停止鸣叫
 * @note   通过开启或关闭 TIM2 的 PWM 输出通道来控制无源蜂鸣器。
 */
void Bsp_Buzzer_Control(bool state)
{
    if (state == true)
    {
        /* 硬件层：启动 TIM2 PWM 输出 */
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    }
    else
    {
        /* 硬件层：停止 TIM2 PWM 输出 */
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    }
}

