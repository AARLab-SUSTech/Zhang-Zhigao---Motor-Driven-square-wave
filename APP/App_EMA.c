#include "App_EMA.h"
#include "Mid_Control.h"
#include "Bsp_Control.h"
/* ==========================================
 * 外部依赖声明 (Extern Declarations)
 * ========================================== */
extern Motor EMA_DATA;

P_Controller Motor_Position_Controller;// 定义控制器实例

volatile MotorDirection_t motor_direction = MOTOR_FORWARD; // 默认为正

volatile int32_t repeated_pos_A = 0;      // 往复运动点A
volatile int32_t repeated_pos_B = 0;      // 往复运动点B
volatile uint32_t repeated_count_total = 0; // 需要往复的总次数
volatile uint32_t repeated_count_current = 0; // 当前已完成的次数
volatile float repeated_speed_hz = 0.0f;    // 往复运动时使用的速度
volatile uint32_t phase_accumulator = 0;
volatile uint32_t phase_increment = 0;
volatile uint16_t pwm_duty_value = 0;
volatile uint8_t step = 0;
float interrupt_freq_hz = 10000.0f; // 您TIM6中断的频率 (1 / 0.00005s)

// --- 新增：用于累计步数的全局变量 ---
// 使用 signed 32-bit 整数，可以记录正反转，且范围足够大
// 使用 volatile 关键字，确保在中断和主循环中安全访问
volatile int32_t absolute_step_counter = 0;
// 目标步数位置
volatile int32_t target_step_position = 0;
// 在位置模式下，电机移动到目标点时使用的速度
volatile uint32_t position_mode_increment = 10;
// 在速度模式下，电机移动速度
volatile uint32_t velocity_mode_increment = 0;

/**
 * @brief  EMA 核心运动规划与状态机执行任务 (APP 层)
 * @note   该函数是电机的“小脑”，负责根据当前模式 (Motor_mode) 计算
 * 每一刻的运动方向和步进增量 (phase_increment)。
 * 【执行上下文】：必须在固定的控制环定时器中断中周期调用 (例如 1kHz 甚至 10kHz)，
 * 且必须放在位置传感器读取 (App_Update_Position_Sensor_Data) 之后执行。
 */
void App_EMA_Motion_Task(void)
{
    /* ========================================================================== *
     * 1. 安全检查层：处理最高优先级的 IDLE, ERROR 和硬件故障状态
     * ========================================================================== */
    if (EMA_DATA.Motor_mode == MOTOR_IDLE ||
        EMA_DATA.Motor_mode == MOTOR_ERROR ||
        EMA_DATA.Motor_mode == MOTOR_OVER_HV_CURRENT ||
        EMA_DATA.Motor_mode == MOTOR_OVER_HV_VOLTAGE ||
        EMA_DATA.Motor_mode == MOTOR_OVER_DC_IN_CURRENT)
    {
        /* 发生致命错误或进入空闲，强制速度增量为 0，确保电机静止 */
        phase_increment = 0;
        return; /* 直接退出，不执行后续运动规划 */
    }

    /* ========================================================================== *
     * 2. 状态决策层：处理复合运动模式的逻辑流转 (如往复模式)
     * ========================================================================== */
    if (EMA_DATA.Motor_mode == MOTOR_OPEN_REPEATED)
    {
        /* 在往复模式下，检查上一个移动是否已完成 (表现为电机已停止) */
        if (phase_increment == 0)
        {
            if (repeated_count_current >= repeated_count_total)
            {
                /* 检查总次数是否已完成：任务结束，切换到空闲模式 */
                EMA_DATA.Motor_mode = MOTOR_IDLE;

                /* 向总线报告往复任务完成 (指令码 0x06, 状态 0x10) */
                //Queue_Reply_Request(0x06, 0x10);
            }
            else
            {
                /* 任务未完成，决定下一个目标点 */
                if (absolute_step_counter == repeated_pos_A)
                {
                    /* 当前在 A 点，下一个目标是 B 点 */
                    target_step_position = repeated_pos_B;
                }
                else /* 当前在 B 点 (或者刚初始化的未知位置) */
                {
                    /* 下一个目标是 A 点 */
                    target_step_position = repeated_pos_A;

                    /* 完成一次 B->A 的移动，才算一个完整的往复周期 */
                    if (absolute_step_counter == repeated_pos_B)
                    {
                        repeated_count_current++;
                    }
                }
            }
        }
    }

    /* ========================================================================== *
     * 3. 运动执行层：根据当前模式与目标，计算底层的物理驱动参数
     * ========================================================================== */

    /* A. 位置控制类模式 (单次定位、往复、同步) */
    if (EMA_DATA.Motor_mode == MOTOR_OPEN_POSITION ||
        EMA_DATA.Motor_mode == MOTOR_OPEN_REPEATED ||
        EMA_DATA.Motor_mode == MOTOR_SYNC_POSITION)
    {
        if (absolute_step_counter < target_step_position)
        {
            motor_direction = MOTOR_FORWARD;
            phase_increment = position_mode_increment;
        }
        else if (absolute_step_counter > target_step_position)
        {
            motor_direction = MOTOR_REVERSE;
            phase_increment = position_mode_increment;
        }
        else
        {
            /* 已到达目标，停止发脉冲/步进 */
            phase_increment = 0;
        }
    }

    /* B. 开环速度控制模式 */
    else if (EMA_DATA.Motor_mode == MOTOR_OPEN_VELOCITY)
    {
        phase_increment = velocity_mode_increment;
    }

    /* C. 闭环位置控制模式 (PID) */
    else if (EMA_DATA.Motor_mode == MOTOR_CLOSE_POSITION)
    {
        /* 调用 MID 层的 PID 算法，获取输出量 */
        float output = P_Control_Compute(&Motor_Position_Controller, Motor_Position_Controller.TargetPos, EMA_DATA.position_mm);

        if (output >= 0)
        {
            motor_direction = MOTOR_REVERSE;
            float increment_f = (output / interrupt_freq_hz) * PHASE_2_32;
            phase_increment = (uint32_t)increment_f;
        }
        else
        {
            motor_direction = MOTOR_FORWARD;
            float increment_f = (-output / interrupt_freq_hz) * PHASE_2_32;
            phase_increment = (uint32_t)increment_f;
        }
    }

    /* D. 闭环力矩/推力控制模式 (预留) */
    else if (EMA_DATA.Motor_mode == MOTOR_CLOSE_FORCE)
    {
        /*
        float output = P_Control_Compute(&Motor_Force_Controller, Motor_Force_Controller.TargetPos, Force_Sensor1.weight_g);
        if (output >= 0)
        {
            motor_direction = MOTOR_REVERSE;
            float increment_f = (output / interrupt_freq_hz) * PHASE_2_32;
            phase_increment = (uint32_t)increment_f;
        }
        else
        {
            motor_direction = MOTOR_FORWARD;
            float increment_f = (-output / interrupt_freq_hz) * PHASE_2_32;
            phase_increment = (uint32_t)increment_f;
        }
        */
    }
}

void App_EMA_Position_Control(void)
{
	      Motor_Position_Controller.Kp = 100;
	      Motor_Position_Controller.MaxSpeed = 100;
}


/**
 * @brief  EMA 电机六步换向与相位执行任务 (APP 层 / 算法底层)
 * @note   该函数是电机的“心脏起搏器”，基于数控振荡器 (NCO) 原理。
 * 它消费运动规划层产生的 phase_increment，将其转化为 0~5 的六步换向扇区，
 * 并触发底层硬件进行实际的 PWM 切换。
 * 【执行上下文】：必须在控制环定时器中断中调用，且必须位于 App_EMA_Motion_Task() 之后！
 */
void App_EMA_Commutation_Task(void)
{

    /* ========================================================================== *
     * 1. NCO 相位累加器 (Phase Accumulator) 更新
     * ========================================================================== */
    /* 无论何种模式，都基于上一级状态机计算出的 phase_increment 进行累加/累减 */
    if (motor_direction == MOTOR_FORWARD)
    {
        phase_accumulator += phase_increment;
    }
    else
    {
        phase_accumulator -= phase_increment;
    }

    /* ========================================================================== *
     * 2. 相位到电气扇区 (0~5) 的高速映射
     * ========================================================================== */
    /* 【神级优化】：将 0x00000000 ~ 0xFFFFFFFF 的全量程等分成 6 份。
     * 使用 64 位整数乘法后右移 32 位，完美避开了耗时的浮点除法，单周期即可完成！ */
    uint8_t new_step = (uint8_t)(((uint64_t)phase_accumulator * 6) >> 32);

    /* 安全钳位：虽然理论上右移不会超过 5，但为防位运算溢出做最后一道防线 */
    if (new_step > 5)
    {
        new_step = 5;
    }

    /* ========================================================================== *
     * 3. 扇区跨越检测与硬件触发 (Edge Detection & Hardware Trigger)
     * ========================================================================== */
    /* 只有当计算出的扇区发生变化时，才进行底层硬件操作，避免无意义的频繁刷新 */
    if (step != new_step)
    {
        /* --------------------------------------------------- *
         * 3.2 更新绝对步进计数器 (全局物理位置的基础)
         * --------------------------------------------------- */
        if (motor_direction == MOTOR_FORWARD)
        {
            absolute_step_counter++;
        }
        else
        {
            absolute_step_counter--;
        }

        /* --------------------------------------------------- *
         * 3.3 触发底层硬件执行换向
         * --------------------------------------------------- */
        step = new_step;         /* 更新全局当前步进状态 */
        Bsp_Update_Output(step);     /* 【调用 BSP 层】真正改变 MOS 管的开关状态 */
    }
}


void App_Singal_Set(void)
{
//阶跃信号
//	  if(HAL_GetTick() - last_time_ms <= 1000)
//	  {
//		  Motor_Position_Controller.TargetPos = 0.0f;
//	  }
//	  else if((HAL_GetTick() - last_time_ms <= 2000) && (HAL_GetTick() - last_time_ms >= 1000))
//	  {
//		  Motor_Position_Controller.TargetPos = 6.0f;
//	  }
//	  else
//	  {
//		  last_time_ms = HAL_GetTick();
//	  }
//正弦位置
//	  	  Motor_Position_Controller.TargetPos = (5 * sin(HAL_GetTick() * Omega_Sin_Velocity)) + 5;
//	  	  time_gap = HAL_GetTick() - last_time_ms;
//	  	  if(time_gap <= 500)
//	  	  {
//	  		  Motor_Position_Controller.TargetPos = time_gap * 0.02f;
//	  	  }
//	  	  else if((time_gap <= 1000) && (time_gap >= 500))
//	  	  {
//	  		  Motor_Position_Controller.TargetPos = 20 - (time_gap * 0.02f);
//	  	  }
//	  	  else
//	  	  {
//	  		  last_time_ms = HAL_GetTick();
//	  	  }
}











