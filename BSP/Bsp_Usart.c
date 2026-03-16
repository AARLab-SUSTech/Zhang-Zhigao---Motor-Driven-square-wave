/**
 ******************************************************************************
 * @file    Bsp_Usart.c
 * @brief   USART/UART 底层硬件驱动模块
 * @note    负责串口 DMA 空闲中断接收机制的初始化与底层配置。
 ******************************************************************************
 */

#include "Bsp_Usart.h"
#include "stm32g4xx_hal.h"

/* ==========================================
 * 外部依赖声明 (Extern Declarations)
 * ========================================== */
extern UART_HandleTypeDef huart1;

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;


/* ==========================================
 * 全局变量定义 (Global Variables)
 * ========================================== */

/* 用于存放串口 DMA 自动接收的原始数据缓存区 */
uint8_t rx_buffer[RX_BUFFER_SIZE];

/* 用于在业务层或 MID 层处理和解析数据的缓存区 */
uint8_t process_buffer[RX_BUFFER_SIZE];


/* ==========================================
 * 函数实现 (Function Implementations)
 * ========================================== */

/**
 * @brief  USART 硬件初始化及 DMA 接收配置
 * @note   配置串口使用 DMA 方式接收数据，直到发生空闲中断 (IDLE line detection)。
 * 同时禁用了 DMA 的半传输中断 (HT)，防止长数据包被意外截断。
 */
void Bsp_Usart_Init(void)
{
    /* 1. 开启串口 DMA 空闲中断接收模式 */
    /* 将接收到的数据自动搬运到 rx_buffer 中，最大监听长度为 RX_BUFFER_SIZE */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer, RX_BUFFER_SIZE);

    /* 2. 禁用 DMA 半传输中断 (Half Transfer Interrupt) */
    /* 【非常关键】：如果不禁用此中断，当接收数据量达到 RX_BUFFER_SIZE 的一半时，
     * 底层也会触发一次回调，这会导致原本完整的一长帧指令被错误地劈成两半。 */
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
}
