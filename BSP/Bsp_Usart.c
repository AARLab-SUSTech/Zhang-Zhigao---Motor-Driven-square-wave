#include "Bsp_Usart.h"
#include "stm32g4xx_hal.h"

extern UART_HandleTypeDef huart1;

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

uint8_t rx_buffer[RX_BUFFER_SIZE];        // 在这里为 rx_buffer 分配了 256 字节
uint8_t process_buffer[RX_BUFFER_SIZE];   // 在这里为 process_buffer 分配了 256 字节

void Bsp_Usart_Init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer, RX_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);
}


















