/**
 ******************************************************************************
 * @file    App_EMA.h (原 FOC.h)
 * @author  Letian
 * @date    May 14, 2025
 * @brief   机电作动器 (EMA) 顶层应用与状态机头文件 (APP)
 * @note    定义了电机全局状态机枚举类型、核心控制数据结构 (Motor)，
 * 囊括了从传感器原始数据、物理运动状态、标定参数到闭环控制的所有全局变量。
 ******************************************************************************
 */

#ifndef _APP_EMA_H_
#define _APP_EMA_H_

/* 包含头文件 ----------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

/* ========================================================================== *
 * 枚举类型定义 (Enumerations)
 * ========================================================================== */

/**
 * @brief  电机全局控制与运行模式枚举
 */
typedef enum {
    /* --- 基础与系统状态 --- */
    MOTOR_IDLE = 0,              /* 空闲模式 (完全下电/释放) */
    MOTOR_READY,                 /* 待机模式 (高压已上电，等待指令) */
	MOTOR_ERROR,                 /* 错误/急停模式 (具体故障原因请查阅 Fault_Flags) */

    /* --- 开环控制模式 --- */
    MOTOR_OPEN_SPEED,            /* 开环速度模式 (基于定频/占空比) */
    MOTOR_OPEN_POSITION,         /* 开环单次定位模式 (步进运动) */
    MOTOR_OPEN_REPEATED,         /* 开环往复运动模式 */
    MOTOR_OPEN_VELOCITY,         /* 开环速度模式 (基于增量) */

    /* --- 闭环控制模式 --- */
    MOTOR_CLOSE_POSITION,        /* 闭环位置模式 (PID) */
    MOTOR_CLOSE_FORCE,           /* 闭环力矩模式 (电流环/测力计反馈) */
    MOTOR_CLOSE_VELOCITY,        /* 闭环速度模式 */

    /* --- 高级与特定功能模式 --- */
    MOTOR_SYNC_POSITION,         /* 多电机同步位置模式 */

} Motor_Control_Mode_e;

typedef enum {
    MOTOR_REVERSE = 0,
    MOTOR_FORWARD = 1
} MotorDirection_t;

extern  volatile MotorDirection_t motor_direction;

/* ========================================================================== *
 * 核心数据结构 (Core Data Structures)
 * ========================================================================== */

/**
 * @brief  电机核心对象数据结构
 * @note   该结构体是整个 EMA 控制系统的大脑，包含了全部的运行上下文和实时状态。
 */
typedef struct {
    /* --- 1. 状态机与调度变量 --- */
    Motor_Control_Mode_e Motor_mode;         /* 当前电机所处的工作模式 */
    uint8_t              first_calculation;  /* 首次计算标志位 (用于初始化差分等历史状态) */
    uint8_t              Loop_count;         /* 降频计数器 (用于低速位置环和速度环的执行节拍) */

    /* --- 2. 磁栅/编码器传感器数据 --- */
    uint32_t             sin_cos_ADC_RAW[2]; /* 原始 ADC 采样值 (0: SIN, 1: COS) */
    float                sin_V;              /* 换算后的正弦通道电压 (V) */
    float                cos_V;              /* 换算后的余弦通道电压 (V) */
    float                sin_offset;         /* 正弦通道零点偏置 (标定值) */
    float                cos_offset;         /* 余弦通道零点偏置 (标定值) */

    /* --- 3. 角度与位置计算状态 --- */
    float                theta_radians;      /* 当前电气角度 (弧度) */
    float                theta_degrees;      /* 当前电气角度 (角度) */
    float                previous_theta_degrees; /* 上一次的电气角度 (用于检测过零点/圈数) */
    int32_t              cycle_count;        /* 累计的完整电气周期数 (圈数) */

    float                current_displacement_within_cycle_mm; /* 单个周期内的直线位移 (mm) */
    float                position_mm;        /* 全局绝对直线位置 (mm) */
    float                Last_position_mm;   /* 上一次的绝对位置 (用于计算速度) */

    /* --- 4. 速度计算状态 --- */
    float                speed;              /* 当前运行速度 (mm/s 或度/s) */
    float                Last_speed;         /* 上一次的速度值 (用于加速度计算或滤波) */

    /* --- 5. 运动行程与标定参数 --- */
    float                min_position_mm;    /* 标定出的最小物理极限位置 (mm) */
    float                max_position_mm;    /* 标定出的最大物理极限位置 (mm) */
    float                distance_of_travel; /* 标定出的总有效行程 (mm) */
    bool                 Min_Max_cal_status; /* 行程极值标定完成标志 */
    bool                 theta_offset_cal_status; /* 角度偏置标定完成标志 */

    /* --- 6. 往复运动模式 (Repeated Mode) 专有参数 --- */
    float                recip_theta_A;      /* 往复点 A 的目标位置/角度 */
    float                recip_theta_B;      /* 往复点 B 的目标位置/角度 */
    bool                 recip_moving_to_B;  /* 方向标志：当前是否正在向 B 点运动 */
    bool                 recip_is_homing;    /* 归位标志：当前是否处于寻零/回初始点阶段 */
    uint16_t             recip_target_cycles;/* 设定的目标往复总次数 */
    uint16_t             recip_current_cycle;/* 当前已完成的往复次数 */

    /* --- 7. 闭环控制 (FOC/PID) 参数 --- */
    struct {
        float kp;                 /* 位置环比例增益 (P) */
        float kd;                 /* 位置环微分增益 (D, 可选) */
        float error_sum_pos_rad;  /* 位置环误差积分项 (I, 弧度/位置) */
    } Position_P;

    /* --- 8. 电压电流参数  Hv_V Hv_I Dc_I --- */

    float                Hv_V_V; 				 /* 高压母线电压 (V)  */
    float                Hv_I_mA; 				 /* 高压母线电流 (mA)  */
    float                Dc_I_A; 				 /* 低压母线电流 (A)  */

    float                Id_ref;             /* d 轴期望目标电流 (mA) - 励磁分量 */
    float                Iq_ref;             /* q 轴期望目标电流 (mA) - 转矩分量 */

    /* --- 9. --- 故障记录本 --- */
        uint32_t Fault_Flags;   // 全局故障标志位 (0 表示无故障，非 0 表示有故障)

} Motor;

/* ========================================================================== *
 * 外部全局变量声明 (Extern Variables)
 * ========================================================================== */

/**
 * @brief 全局电机对象实体
 * @note  在 App_EMA.c 中定义，其他所有子模块均通过此变量监控与控制电机状态。
 */
extern Motor EMA_DATA;

void App_EMA_Position_Control(void);

/**
 * @brief  EMA 核心运动规划与状态机执行任务 (APP 层)
 * @note   该函数是电机的“小脑”，负责根据当前模式 (Motor_mode) 计算
 * 每一刻的运动方向和步进增量 (phase_increment)。
 * 【执行上下文】：必须在固定的控制环定时器中断中周期调用 (例如 1kHz 甚至 10kHz)，
 * 且必须放在位置传感器读取 (App_Update_Position_Sensor_Data) 之后执行。
 */
void App_EMA_Motion_Task(void);


/**
 * @brief  EMA 电机六步换向与相位执行任务 (APP 层 / 算法底层)
 * @note   该函数是电机的“心脏起搏器”，基于数控振荡器 (NCO) 原理。
 * 它消费运动规划层产生的 phase_increment，将其转化为 0~5 的六步换向扇区，
 * 并触发底层硬件进行实际的 PWM 切换。
 * 【执行上下文】：必须在控制环定时器中断中调用，且必须位于 App_EMA_Motion_Task() 之后！
 */
void App_EMA_Commutation_Task(void);

#endif /* _APP_EMA_H_ */
