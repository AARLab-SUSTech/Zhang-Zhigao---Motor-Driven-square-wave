/**
 ******************************************************************************
 * @file    Mid_Command_Usart.h (原 command.h)
 * @author  Letian
 * @date    Sep 2, 2025
 * @brief   串口通信协议中间件头文件 (MID)
 * @note    提供 UART 数据解析、DMA 变长发送等核心业务流的接口声明与外部变量。
 ******************************************************************************
 */

#ifndef _MID_COMMAND_USART_H_
#define _MID_COMMAND_USART_H_

#include "main.h"

/* ========================================================================== *
 * 外部全局变量声明 (Extern Variables)
 * ========================================================================== */

/**
 * @brief  用于协议解析时临时存储的浮点型速度/位置变量
 */
extern float temp_speed_float;

/**
 * @brief  用于协议解析时临时拼接的 16 位整型速度/位置变量
 */
extern int16_t temp_speed_int16;


/* ========================================================================== *
 * 函数接口声明 (Function Prototypes)
 * ========================================================================== */

/**
 * @brief  解析接收到的串口指令帧及业务逻辑分发
 * @param  data 指向接收缓冲区的指针
 * @param  size 本次接收到的实际总字节数
 */
void Mid_Process_Usart_Data(uint8_t* data, uint16_t size);

/**
 * @brief  使用 DMA 发送一个浮点数数组 (自动格式化为 ASCII 文本)
 * @param  arr   指向浮点数数组的指针
 * @param  count 数组中浮点数的个数
 */
void Mid_Float_Array_Dma_Send(float* arr, int count);

/**
 * @brief  将一个大数组按指定分组大小格式化，并使用 DMA 一次性发送
 * @param  arr                指向 int16_t 数组的指针
 * @param  total_count        数组中元素的总个数
 * @param  elements_per_group 每组包含多少个元素 (用于插入换行符)
 */
void Mid_Int16_Groups_Dma_Send(int16_t* arr, int total_count, int elements_per_group);

/**
 * @brief  使用 JustFloat 协议通过 USART DMA 发送浮点数组
 * @param  pData 浮点数数组指针
 * @param  count 浮点数的个数
 * @note   通常用于对接 Vofa+ 等上位机波形显示软件
 */
void Mid_Usart_Send_JustFloat(float *pData, uint8_t count);

#endif /* _MID_COMMAND_USART_H_ */
