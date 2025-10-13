/*
 * can.c
 *
 *  Created on: Oct 11, 2025
 *      Author: Letian
 */
#include "can.h"
#include "main.h"
#include "control.h"

extern volatile Motor_mode_t Motor_mode;

FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader; // 用于存储接收报文的头信息
uint8_t RxData[8];              // 用于存储接收报文的数据
uint8_t TxData[8];

void CAN_init(void)
{
	  FDCAN_FilterTypeDef sFilterConfig;

	  sFilterConfig.IdType = FDCAN_STANDARD_ID;       // ID类型：标准ID
	  sFilterConfig.FilterIndex = 0;                  // 过滤器索引，0-27
	  sFilterConfig.FilterType = FDCAN_FILTER_DUAL;   // 过滤器类型：
	  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 匹配成功后存入Rx FIFO 0
	  sFilterConfig.FilterID1 = MY_NODE_ID;                // 要匹配的ID
	  sFilterConfig.FilterID2 = BROADCAST_ID;              // 广播ID

	  // 应用此过滤器配置
	  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
	  {
	  	printf("rxerror\r\n");
	      Error_Handler();
	  }
	    /* 1. 启动 FDCAN 外设 */
	    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
	    {
	      // 如果HAL_FDCAN_Start失败，会在这里打印信息然后卡死
	      printf("FATAL: HAL_FDCAN_Start() FAILED%d!\r\n",HAL_FDCAN_Start(&hfdcan1));
	      Error_Handler();
	    }
	    printf("INFO: HAL_FDCAN_Start() OK.\r\n");
	    printf("--- FIRMWARE VERSION 2.0 -- FILTER TEST ---\r\n");

	    // 激活接收FIFO 0新消息通知，这是开启接收中断的大门
	    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
	    {
	        Error_Handler();
	    }

	    /* 2. 配置发送报文头和数据 */
	    TxHeader.Identifier = 0x001;
	    TxHeader.IdType = FDCAN_STANDARD_ID;
	    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	    TxHeader.FDFormat = FDCAN_FRAME_CLASSIC;
	    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	    TxHeader.MessageMarker = 0;

	    for(int i=0; i<8; i++) {
	        TxData[i] = i;
	    }
}

/*
Data[0] = 0x01: 紧急停止 (Emergency Stop)

Data[0] = 0x02: 步进+ (相对位置正向运动)

Data[0] = 0x03: 步进- (相对位置反向运动)

Data[0] = 0x04: 往复模式 (Reciprocating Mode)

Data[0] = 0x05: 绝对位置模式 (Absolute Position Mode) - 【新增】
 */
// FDCAN接收FIFO 0消息挂起回调函数
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    // 检查是否是“新消息到达”中断
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        // 从Rx FIFO 0中获取消息，存入上面的全局变量中
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
        {
            // --- 在这里添加您自己的数据处理逻辑 ---
            printf("Message Received!\r\n");
            printf("  ID   : 0x%lX\r\n", RxHeader.Identifier);
            printf("  DLC  : %ld bytes\r\n", RxHeader.DataLength);
            printf("  Data : ");
            for(int i=0; i < RxHeader.DataLength; i++)
            {
                printf("0x%02X ", RxData[i]);
            }
            printf("\r\n");

            if((RxHeader.Identifier == MY_NODE_ID ) || (RxHeader.Identifier == BROADCAST_ID ))
            {
            // --- BEGIN CAN PROTOCOL PROCESSING (v2) ---

                        uint8_t command = RxData[0]; // 提取命令字

                        switch(command)
                        {
                            case 0x01: // 命令: 紧急停止
                            {
	                        	DC_Power_OFF;
	                        	if((Motor_mode != MOTOR_OVER_HV_VOLTAGE) || (Motor_mode != MOTOR_OVER_HV_CURRENT) || (Motor_mode != MOTOR_OVER_DC_IN_CURRENT || Motor_mode != MOTOR_ERROR))
	                        	{
	                     		Motor_mode = MOTOR_IDLE;
	                        	}
	                     		Close_output();
	                     		HAL_TIM_Base_Stop(&htim16);//dma printf
                                break;
                            }

                            case 0x02: // 命令: 步进+ (正向相对运动)
                            case 0x03: // 命令: 步进- (反向相对运动)
                            {
                                // 1. 解析参数
                                uint16_t speed_hz = (uint16_t)RxData[1] |
                                                    (uint16_t)(RxData[2] << 8);

                                // 2. 计算并设置电机控制参数
                                position_mode_increment = (uint32_t)(((uint64_t)speed_hz * 0x100000000) / interrupt_freq_hz);

                                // 3. 设置电机模式和目标
                                Motor_mode = MOTOR_OPEN_POSITION;

                                if (command == 0x02) // 正向
                                {
   	                        	 Motor_mode = MOTOR_OPEN_POSITION;
   	                        	 step_move(1, 10);
                                }
                                else // 反向 (command == 0x03)
                                {
   	                        	 Motor_mode = MOTOR_OPEN_POSITION;
   	                        	 step_move(-1, 10);
                                }
                                printf("CMD: Relative Step %c, Speed: %u Hz\r\n", (command == 0x02 ? '+' : '-'), speed_hz);
                                break;
                            }

                            case 0x04: // 命令: 往复模式
                            {
                                // 1. 解析参数
                                uint16_t speed_hz = (uint16_t)RxData[1] |
                                					(uint32_t)(RxData[2] << 8);
                                uint16_t steps_range_A = (uint32_t)RxData[3] |
                                                       (uint32_t)(RxData[4] << 8);

                                uint16_t steps_range_B = (uint32_t)RxData[5] |
                                                       (uint32_t)(RxData[6] << 8);

                                uint8_t count =       (uint32_t)RxData[7];

                                // 2. 计算速度
                                position_mode_increment = (uint32_t)(((uint64_t)speed_hz * 0x100000000) / interrupt_freq_hz);

                                // 3. 设置往复运动参数
                                repeated_pos_A = steps_range_A;
                                repeated_pos_B = steps_range_B;
                                repeated_count_total = count;
                                repeated_count_current = 0;

                                // 4. 启动往复模式
	                        	Motor_mode = MOTOR_OPEN_REPEATED;
	                        	move_repeatedly(repeated_pos_A, repeated_pos_B, repeated_count_total, speed_hz);

                                printf("CMD: Repeated Mode, Range: %d   %d steps, Speed: %u Hz  count:%d \r\n", steps_range_A, steps_range_B, speed_hz , count);
                                break;
                            }

                            case 0x05: // 【新增】命令: 绝对位置模式
                            {
                                // 1. 解析参数
                                // 在此模式下，参数代表的是绝对位置坐标

                                uint16_t speed_hz = (uint16_t)RxData[1] |
                                                    (uint16_t)(RxData[2] << 8);

                                uint32_t absolute_pos = (uint32_t)RxData[3] |
                                                        (uint32_t)(RxData[4] << 8);
                                // 2. 计算速度
                                position_mode_increment = (uint32_t)(((uint64_t)speed_hz * 0x100000000) / interrupt_freq_hz);

                                // 3. 设置电机模式和目标
                                Motor_mode = MOTOR_OPEN_POSITION;
                                // 直接将目标位置设置为指令中的绝对位置
                                target_step_position = absolute_pos;
	                            move_to_position(target_step_position , speed_hz);
                                printf("CMD: Absolute Position, Target: %ld, Speed: %u Hz\r\n", target_step_position, speed_hz);
                                break;
                            }

                            default:
                            {
                                // 收到未知的命令
                                printf("ERR: Unknown Command 0x%02X\r\n", command);
                                break;
                            }
                        }
                        // --- END CAN PROTOCOL PROCESSING (v2) ---
            }
            else
            {
                printf("ID error\r\n");
            }

        }

        // 【非常重要】: 每次处理完中断后，必须重新激活通知，否则中断只会触发一次！
        if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
        {
           Error_Handler();
        }
    }
}









