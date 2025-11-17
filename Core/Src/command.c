/*
 * command.c
 *
 *  Created on: Sep 2, 2025
 *      Author: Letian
 */


#ifndef _COMMAND_H
#define _COMMAND_H

#include "control.h"
#include "command.h"
#include "string.h"

// --- 串口 DMA 接收相关 ---
extern uint8_t rx_buffer[RX_BUFFER_SIZE];
extern uint8_t process_buffer[RX_BUFFER_SIZE];
extern volatile Motor_mode_t Motor_mode;
extern int dma_print_flag;

#define UART_TX_BUFFER_SIZE 512 // 定义发送缓冲区大小，确保足够长
uint8_t g_uart_tx_buffer[UART_TX_BUFFER_SIZE]; // DMA发送缓冲区

// volatile关键字很重要，因为它可能在主程序和中断服务程序中同时被访问
volatile uint8_t g_uart_dma_transfer_complete = 1; // 1: 空闲, 0: 忙碌

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

/**
 * @brief 使用DMA发送一个浮点数数组
 * @param arr 指向浮点数数组的指针
 * @param count 数组中浮点数的个数
 */
void send_float_array_dma(float* arr, int count) {
    // 1. 检查DMA是否空闲
    if (g_uart_dma_transfer_complete == 0) {
    	Buzzer_ON;
    	while(1)
    	{
        	printf("dma busy error\r\n");
        	HAL_Delay(1000);
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
            Buzzer_ON;
        	printf("dma buffer too small error\r\n");
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
 * @brief 将一个大数组按指定分组大小格式化，并使用DMA一次性发送
 * @param arr                指向int16_t数组的指针
 * @param total_count        数组中元素的总个数
 * @param elements_per_group 每组包含多少个元素
 */
void send_int16_groups_dma(int16_t* arr, int total_count, int elements_per_group)
{

	    // 1. 检查DMA是否空闲
	    if (g_uart_dma_transfer_complete == 0) {
	    	Buzzer_ON;
	    	while(1)
	    	{
	        	printf("dma busy error\r\n");
	        	HAL_Delay(1000);
	    	}
	        return; // DMA忙，丢弃本次任务
	    }
	    if (total_count <= 0 || elements_per_group <= 0) {
	        return; // 无效参数
	    }

    // --- 缓冲区大小检查 (非常重要!) ---
    // 每个元素占 2字节(int16_t) + 1字节(分隔符) = 3字节
    // 确保定义的 UART_TX_BUFFER_SIZE 足够大
    if (total_count * 3 > UART_TX_BUFFER_SIZE) {
        Buzzer_ON;
        printf("dma buffer too small error\r\n");
        return;
    }

    // 2. 将DMA状态标志设置为“忙碌”
    g_uart_dma_transfer_complete = 0;

    // 3. 【核心修改】使用 memcpy 和指针高效构建数据流
    // 创建一个写入指针，指向缓冲区的开头
    uint8_t* p_write = g_uart_tx_buffer;

    for (int i = 0; i < total_count; i++)
    {
        // **步骤 A: 搬运二进制数据**
        // 使用 memcpy 将2个字节的 int16_t 原封不动地复制到缓冲区
        memcpy(p_write, &arr[i], sizeof(int16_t));
        // 将写入指针向前移动2个字节
        p_write += sizeof(int16_t);

        // **步骤 B: 添加分隔符**
        // 判断是否为一组的末尾
        if (((i + 1) % elements_per_group == 0) || (i == total_count - 1))
        {
            // 是末尾，添加换行符
            *p_write++ = '\n'; // 写入1个字节的换行符，并使指针+1
        }
        else
        {
            // 不是末尾，添加逗号
            *p_write++ = ','; // 写入1个字节的逗号，并使指针+1
        }
    }

    // 计算最终写入的数据总长度
    int current_len = p_write - g_uart_tx_buffer;

    // 4. 启动DMA传输 (这部分逻辑保持不变)
    if (current_len > 0)
    {
        HAL_UART_Transmit_DMA(&huart1, g_uart_tx_buffer, current_len);
    }
    else
    {
        g_uart_dma_transfer_complete = 1;
    }

//    // 1. 检查DMA是否空闲
//    if (g_uart_dma_transfer_complete == 0) {
//    	Buzzer_ON;
//    	while(1)
//    	{
//        	printf("dma busy error\r\n");
//        	HAL_Delay(1000);
//    	}
//        return; // DMA忙，丢弃本次任务
//    }
//    if (total_count <= 0 || elements_per_group <= 0) {
//        return; // 无效参数
//    }
//
//    // 2. 将DMA状态标志设置为“忙碌”
//    g_uart_dma_transfer_complete = 0;
//
//    // 3. 动态构建格式化字符串
//    int current_len = 0;
//    int remaining_size = UART_TX_BUFFER_SIZE;
//
//    for (int i = 0; i < total_count; i++)
//    {
//        int written_len;
//
//        // ***** 核心逻辑修改 *****
//        // 判断条件:
//        // 1. (i + 1) % elements_per_group == 0 : 当前元素是某一组的最后一个
//        // 2. i == total_count - 1             : 当前元素是整个大数组的最后一个 (处理最后一组不完整的情况)
//        if (((i + 1) % elements_per_group == 0) || (i == total_count - 1))
//        {
//            // 是组的末尾或是总的末尾，添加换行符
//            written_len = snprintf((char*)g_uart_tx_buffer + current_len, remaining_size, "%d\r\n", arr[i]);
//        }
//        else
//        {
//            // 在组的中间，添加逗号
//            written_len = snprintf((char*)g_uart_tx_buffer + current_len, remaining_size, "%d,", arr[i]);
//        }
//
//        // 检查snprintf是否成功，以及是否超出缓冲区
//        if (written_len <= 0 || written_len >= remaining_size)
//        {
//            // 缓冲区太小是常见错误，需要增大 UART_TX_BUFFER_SIZE
//            g_uart_dma_transfer_complete = 1; // 释放DMA标志
//            // 在这里可以添加错误处理代码，例如通过LED闪烁来报警
//            Buzzer_ON;
//        	printf("dma buffer too small error\r\n");
//            return;
//        }
//
//        // 更新长度和剩余空间
//        current_len += written_len;
//        remaining_size -= written_len;
//    }
//
//    // 4. 启动DMA传输
//    if (current_len > 0)
//    {
//        HAL_UART_Transmit_DMA(&huart1, g_uart_tx_buffer, current_len);
//    }
//    else
//    {
//        g_uart_dma_transfer_complete = 1;
//    }
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
//	                     		 HAL_TIM_Base_Stop(&htim6);
	                     		dma_print_flag = 0;
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
	                         case 0x31://OPEN VELOCITY MODE
	                        	 Motor_mode = MOTOR_OPEN_VELOCITY;
	                    			open_loop_velocity = (((uint16_t)data[6] << 8) | data[5]);
	                        		velocity_mode_increment = (uint32_t)(((uint64_t)open_loop_velocity * 0x100000000) / interrupt_freq_hz);
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
	        	 dma_print_flag = 1;
//	        	 HAL_TIM_Base_Start_IT(&htim6);//dma printf
	         }
}

#endif



