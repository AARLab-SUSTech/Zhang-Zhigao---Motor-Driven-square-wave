/*
 * command.c
 *
 *  Created on: Sep 2, 2025
 *      Author: Letian
 */

#include "command.h"

#ifndef _COMMAND_H
#define _COMMAND_H


// --- 串口 DMA 接收相关 ---
#define RX_BUFFER_SIZE 256   // 定义接收缓冲区的最大长度
uint8_t rx_buffer[RX_BUFFER_SIZE]; // DMA 接收缓冲区
uint8_t process_buffer[RX_BUFFER_SIZE]; // 用于安全处理和打印数据的缓冲区
extern volatile Motor_mode_t Motor_mode;

#define UART_TX_BUFFER_SIZE 128 // 定义发送缓冲区大小，确保足够长
uint8_t g_uart_tx_buffer[UART_TX_BUFFER_SIZE]; // DMA发送缓冲区

// volatile关键字很重要，因为它可能在主程序和中断服务程序中同时被访问
volatile uint8_t g_uart_dma_transfer_complete = 1; // 1: 空闲, 0: 忙碌

/**
 * @brief 使用DMA发送一个浮点数数组
 * @param arr 指向浮点数数组的指针
 * @param count 数组中浮点数的个数
 */
void send_float_array_dma(float* arr, int count) {
    // 1. 检查DMA是否空闲
    if (g_uart_dma_transfer_complete == 0) {
    	Buzzer_ON;
    	printf("dma printf error\r\n");

    	while(1)
    	{

    	}
        return; // DMA忙，直接返回
    }
    if (count <= 0) {
        return; // 没有数据需要发送
    }

    g_uart_dma_transfer_complete = 0; // 设置为忙碌状态

    // 2. 动态构建格式化字符串
    int current_len = 0; // 当前已写入缓冲区的长度
    int remaining_size = UART_TX_BUFFER_SIZE; // 缓冲区剩余大小

    for (int i = 0; i < count; i++) {
        int written_len;
        // 判断是否为最后一个元素
        if (i < count - 1) {
            // 不是最后一个，后面加逗号
            written_len = snprintf((char*)g_uart_tx_buffer + current_len, remaining_size, "%.2f,", arr[i]);
        } else {
            // 是最后一个，后面加换行符
            written_len = snprintf((char*)g_uart_tx_buffer + current_len, remaining_size, "%.2f\r\n", arr[i]);
        }

        // 检查snprintf是否成功，以及是否超出缓冲区
        if (written_len <= 0 || written_len >= remaining_size) {
            // 写入失败或缓冲区已满，提前终止
            // 这里可以添加错误处理，例如发送一个错误提示
            g_uart_dma_transfer_complete = 1; // 释放DMA
            return;
        }

        // 更新长度和剩余空间
        current_len += written_len;
        remaining_size -= written_len;
    }

    // 3. 启动DMA传输
    if (current_len > 0) {
        HAL_UART_Transmit_DMA(&huart1, g_uart_tx_buffer, current_len);
    } else {
        // 如果因为某种原因没有内容被写入，则直接释放DMA
        g_uart_dma_transfer_complete = 1;
    }
}

/**
 * @brief UART DMA发送完成回调函数
 * @param huart UART句柄
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    // 检查是哪个UART完成了发送
    if (huart->Instance == USART1) { // 假设您使用的是USART1
        // 将DMA状态标志设置为“完成/空闲”
        g_uart_dma_transfer_complete = 1;
    }
}


// 添加这个正确的 DMA 空闲中断回调函数
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart->Instance == USART1)
    {
        // 1. 成功进入中断，翻转一个LED作为最直观的指示
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        Buzzer_ON;
        // 2. 检查确实收到了数据 (size > 0)，然后处理它
        //    size 参数是本次DMA实际接收到的字节数，非常重要
        if (size > 0)
        {
            process_received_data(rx_buffer, size);
        }

        // 3. ***** 至关重要的一步 *****
        // 重新启动DMA接收，等待下一次空闲中断事件
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer, RX_BUFFER_SIZE);
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
        Buzzer_OFF;
    }
}

void process_received_data(uint8_t* data, uint16_t size) {
	//PRINT RECIEVE DATA
//	         printf("Received data (size = %d): \r\n", size);
//	         for (uint16_t i = 0; i < size; i++) {
//	             printf("%02X ", data[i]);
//	         }
	         if (data[0] == 0x55 && data[1] == 0xAA) {
	             // Êý¾Ý³¤¶È = Ö¡Í·(2×Ö½Ú) + ³¤¶È(1×Ö½Ú) + ID(1×Ö½Ú) + Ö¸ÁîºÅ(1×Ö½Ú) + Ë÷Òý(1×Ö½Ú) + Êý¾Ý¶Î³¤¶È + Ð£ÑéºÍ(1×Ö½Ú)
	             uint8_t data_length = rx_buffer[2];  // Êý¾Ý³¤¶È×Ö¶Î
	             uint16_t total_length = 2 + 1 + 1 + data_length + 1;  // Ö¡×Ü³¤¶È£¨°üÀ¨Ö¡Í·¡¢³¤¶È×Ö¶Î¡¢ID¡¢Ö¸Áî¡¢Ë÷Òý¡¢Êý¾Ý¡¢Ð£ÑéºÍ£©

	             // ¼ì²éÊÇ·ñÊý¾Ý×ã¹»³¤
	             if (size >= total_length) {
	                 // Ð£ÑéºÍ¼ÆËã
	                 uint8_t checksum = 0;
	                 for (uint16_t i = 2; i < 2 + 1 + 1 + data_length; i++) {
	                     checksum += data[i];  // ÀÛ¼Ó³ýÖ¡Í·ÒÔÍâµÄËùÓÐ×Ö½Ú
	                 }
	                 // ±È½ÏÐ£ÑéºÍ
	                 uint8_t received_checksum = rx_buffer[2 + 1 + 1 + data_length];  // ½ÓÊÕµÄÐ£ÑéºÍ
	                 if (checksum == received_checksum) {
															 printf("ID: %02X, Command: %02X, Index: %02X Data: ", data[3], data[4], data[5]);
															 for (uint8_t i = 0; i < data_length-2; i++) {printf("%02X ", data[6 + i]);}
															 printf("\r\n");
	                     if(data[3] != 0x01)  return;
	                     if((Motor_mode == MOTOR_OVER_HV_VOLTAGE) || (Motor_mode == MOTOR_OVER_HV_CURRENT) || (Motor_mode == MOTOR_OVER_DC_IN_CURRENT)) return;//不处理指令，直接退出
	                     switch(data[4])
	                     {
	                         case 0x24://repeat mode
	                        	 Motor_mode = MOTOR_OPEN_REPEATED;
	                        	 move_repeatedly(data[7], data[6]/2, (((uint16_t)rx_buffer[9] << 8) | rx_buffer[10]), data[8]);
//	                        	 printf("min:%d,max:%d,count:%d,speed:%d\r\n",data[7],data[6],(((uint16_t)rx_buffer[9] << 8) | rx_buffer[10]),data[8]);
	                             break;

	                         case 0x25://STOP
	                        	DC_Power_OFF;
	                        	if((Motor_mode != MOTOR_OVER_HV_VOLTAGE) || (Motor_mode != MOTOR_OVER_HV_CURRENT) || (Motor_mode != MOTOR_OVER_DC_IN_CURRENT || Motor_mode != MOTOR_ERROR))
	                        	{
	                     		Motor_mode = MOTOR_IDLE;
	                        	}
	                     		Close_output();
	                     		 HAL_TIM_Base_Stop(&htim16);//dma printf
	                             break;

	                         case 0x27://step p
	                        	 Motor_mode = MOTOR_OPEN_POSITION;
	                             move_to_position(data[6] , (((uint16_t)data[8] << 8) | data[7]));
//	                             printf("speed:%d step:%d\r\n",(((uint16_t)data[8] << 8) | data[7]),data[6]);
	                             break;

	                         case 0x28://STEP+
	                        	 Motor_mode = MOTOR_OPEN_POSITION;
	                        	 step_move(1, 10);
	                             break;

	                         case 0x29://STEP-
	                        	 Motor_mode = MOTOR_OPEN_POSITION;
	                        	 step_move(-1, 10);
	                             break;
	                         case 0x30://SET SPEED

	                             break;

	                         default:
	                             break;
	                     }
	                 } else {
	                     printf("Checksum error! Calculated: %02X, Received: %02X\n", checksum, received_checksum);
	                 }
	             } else {
	                 printf("Invalid frame: Frame too short. Received length: %d, Expected length: %d\n", size, total_length);
	             }
	         } else {
	             printf("Invalid frame header. Received header: %02X %02X, Expected header: %02X %02X\n",
	            		 data[0], data[1], 0x55, 0xAA);
	         }
	         if((Motor_mode == MOTOR_OPEN_POSITION) || (Motor_mode == MOTOR_OPEN_REPEATED))
	         {
	        	 HAL_TIM_Base_Start_IT(&htim16);//dma printf
	         }
}

#endif



