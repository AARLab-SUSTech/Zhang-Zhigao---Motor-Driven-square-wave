/**
 * @file    App_Fault.h
 * @brief   全局故障管理中心 (APP)
 */
#ifndef _APP_FAULT_H_
#define _APP_FAULT_H_

#include <stdint.h>
#include <stdbool.h>

/* ========================================================================== *
 * 1. 全局故障码字典 (采用位域设计，支持多故障并发)
 * ========================================================================== */
#define FAULT_NONE                  (0x00000000U) /* 状态正常 */

/* --- 严重硬件故障 (Fatal) --- */
#define FAULT_HV_OVER_VOLTAGE       (1 << 0)      /* 0x01: 高压母线过压 */
#define FAULT_HV_OVER_CURRENT       (1 << 1)      /* 0x02: 高压母线过流 */
#define FAULT_DC_OVER_CURRENT       (1 << 2)      /* 0x04: 低压供电过流 */

/* --- 通信与系统故障 (System) --- */
#define FAULT_CAN_OFFLINE           (1 << 3)      /* 0x08: CAN 总线掉线/满载 */
#define FAULT_UART_DMA_ERROR        (1 << 4)      /* 0x10: 串口 DMA 溢出/错误 */


/* --- 传感器逻辑故障 (Sensor) --- */
#define FAULT_ENCODER_LOSS          (1 << 5)      /* 0x20: 磁栅尺信号丢失 */
#define FAULT_POSITION_LIMIT        (1 << 6)      /* 0x40: 超过机械极限限位 */

/* --- Usart故障 --- */
#define FAULT_USART                 (1 << 7)      /* 0x80: 串口DMA故障 */

/* ========================================================================== *
 * 2. 统一故障操作接口
 * ========================================================================== */
void App_Fault_Report(uint32_t fault_code);
void App_Fault_Clear(uint32_t fault_code);
void App_Fault_Task_Handler(void);

#endif /* _APP_FAULT_H_ */
