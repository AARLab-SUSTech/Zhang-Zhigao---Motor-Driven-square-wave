/*
 * command.c
 *
 *  Created on: Sep 2, 2025
 *      Author: Letian
 */
#include <Mid_Command_Can.h>
#include "Bsp_Can.h"
#include "Mid_Control.h"
#include "App_EMA.h"

extern Motor EMA_DATA;

//储存速度计算临时值
float temp_speed_float;
int16_t temp_speed_int16;
float Sync_Motor_Speed=0;
uint32_t Sync_Motor_position=0;
/**
  * @brief  定义队列的存储数组
  * volatile 关键字是必须的，因为它在中断和主循环中被同时访问
  */
extern volatile CanTxMessage_t g_tx_queue[TX_QUEUE_SIZE];
extern volatile CanRxMessage_t g_rx_queue[RX_QUEUE_SIZE];
/**
  * @brief  定义队列的 "头指针" (Head)
  * 'head' 是数据“放入”的位置
  * 只有 "生产者" (中断) 可以修改它
  */
extern volatile uint16_t g_tx_queue_head ;
extern volatile uint16_t g_rx_queue_head ;
/**
  * @brief  定义队列的 "尾指针" (Tail)
  * 'tail' 是数据“取出”的位置
  * 只有 "消费者" (main 循环) 可以修改它
  */
extern volatile uint16_t g_tx_queue_tail ;
extern volatile uint16_t g_rx_queue_tail ;


/*
Data[0] = 0x01: 紧急停止 (Emergency Stop)

Data[0] = 0x02: 高压上电 (Power on)

Data[0] = 0x03: 步进+ (相对位置正向运动)

Data[0] = 0x04: 步进- (相对位置反向运动)

Data[0] = 0x05: 往复模式 (Reciprocating Mode)

Data[0] = 0x06: 绝对位置模式 (Absolute Position Mode) - 【新增】

Data[0] = 0x07: 多电机同步模式 只接收数据

Data[0] = 0x08: 多电机同步模式 同步触发

Data[0] = 0x09: 开环速度模式

 */
void Can_message_process(void)
{
    /*----------------------------------------------------------------*/
        /* --- 消费者 (Consumer) #1: 处理接收队列 (g_rx_queue) --- */
        /*----------------------------------------------------------------*/
        if (g_rx_queue_head != g_rx_queue_tail)
        {
            // 1. 队列不为空，从 "tail" 处取出消息
            volatile CanRxMessage_t* msg = &g_rx_queue[g_rx_queue_tail];

            // (可选) 打印原始报文
//             printf("Rx ID:0x%lX,Data ", msg->Rx_Header.Identifier);
//             for(int i=0; i < msg->Rx_Header.DataLength; i++) { printf("%02X ", msg->Data[i]); }
//             printf("\r\n");

            // 3. 检查ID (这就是您原来的软件过滤器)
            if((msg->Rx_Header.Identifier == MY_NODE_ID ) || (msg->Rx_Header.Identifier == BROADCAST_ID ))
            {
                uint8_t command = msg->Data[0];
                uint8_t status_code = 0x00; // 【新】默认状态码 = 0x00 (OK_Accepted) 0x01无对应指令
                if(msg->Rx_Header.Identifier == MY_NODE_ID ){
//                printf("%d,%2lX\r\n",msg->Data[3],msg->Rx_Header.Identifier);
                }
                // 4. 【安全地】执行所有业务逻辑
                switch(command)
                {
                    case 0x01: // 命令: 紧急停止
                        DC_Power_OFF;
                        HAL_TIM_Base_Stop(&htim16);
                        Close_output();
                        if( (EMA_DATA.Motor_mode != MOTOR_OVER_HV_VOLTAGE) && (EMA_DATA.Motor_mode != MOTOR_OVER_HV_CURRENT) &&
                            (EMA_DATA.Motor_mode != MOTOR_OVER_DC_IN_CURRENT) && (EMA_DATA.Motor_mode != MOTOR_ERROR) )
                        {
                            EMA_DATA.Motor_mode = MOTOR_IDLE;
                        }
                        break;

                    case 0x02: // 命令: 高压上电
                        DC_Power_ON;
                        EMA_DATA.Motor_mode = MOTOR_READY;
                        break;

                    case 0x03: // 命令: 步进+
                    case 0x04: // 命令: 步进-
                        {
                        	temp_speed_int16 = (uint16_t)msg->Data[1] | (uint16_t)(msg->Data[2] << 8);
                        	temp_speed_float = temp_speed_int16 / 100.0f;
                            position_mode_increment = (uint32_t)(((uint64_t)temp_speed_float * 0x100000000) / interrupt_freq_hz);

                            EMA_DATA.Motor_mode = MOTOR_OPEN_POSITION;
                            printf("command:%d,speed %.2f\r\n",command,temp_speed_float);
                            if (command == 0x03) // 正向
                               target_step_position = absolute_step_counter + 1;
                            else // 反向 (command == 0x04)
                               target_step_position = absolute_step_counter - 1;
                        }
                        break;

                    case 0x05: // 命令: 往复模式
                        {
                            EMA_DATA.Motor_mode = MOTOR_OPEN_REPEATED;
                        	temp_speed_int16 = (uint16_t)msg->Data[1] | (uint16_t)(msg->Data[2] << 8);
                        	temp_speed_float = temp_speed_int16 / 100.0f;
                            position_mode_increment = (uint32_t)(((uint64_t)temp_speed_float * 0x100000000) / interrupt_freq_hz);

                            repeated_pos_A = (int32_t)((uint16_t)msg->Data[3] | (uint16_t)(msg->Data[4] << 8));
                            repeated_pos_B = (int32_t)((uint16_t)msg->Data[5] | (uint16_t)(msg->Data[6] << 8));
                            repeated_count_total = (uint32_t)msg->Data[7];
                            if(repeated_count_total == 0)  repeated_count_total = 0xFFFFFFFF;//如果往复次数为0 则为无穷往复
                            repeated_count_current = 0;
                            printf("command:%d,speed_hz %.2f,%ld,%ld\r\n",command,temp_speed_float,repeated_pos_A,repeated_pos_B);
                        }
                        break;

                    case 0x06: // 命令: 绝对位置模式
                        {
                            EMA_DATA.Motor_mode = MOTOR_OPEN_POSITION;
                        	temp_speed_int16 = (uint16_t)msg->Data[1] | (uint16_t)(msg->Data[2] << 8);
                        	temp_speed_float = temp_speed_int16 / 100.0f;
                            position_mode_increment = (uint32_t)(((uint64_t)temp_speed_float * 0x100000000) / interrupt_freq_hz);
                            uint32_t absolute_pos = (uint32_t)msg->Data[3] | (uint32_t)msg->Data[4] << 8;
                            target_step_position = absolute_pos;
                        }
                        break;

                    case 0x07: // 命令: 多电机同步模式
                        {
                        	if(msg->Rx_Header.Identifier == MY_NODE_ID )
                        	{
                        		if((EMA_DATA.Motor_mode != MOTOR_OVER_HV_CURRENT) | (EMA_DATA.Motor_mode != MOTOR_OVER_HV_VOLTAGE) | (EMA_DATA.Motor_mode != MOTOR_OVER_DC_IN_CURRENT))
                        		{
								EMA_DATA.Motor_mode = MOTOR_SYNC_POSITION;
	                        	temp_speed_int16 = (uint16_t)msg->Data[1] | (uint16_t)(msg->Data[2] << 8);
	                        	temp_speed_float = temp_speed_int16 / 100.0f;
								Sync_Motor_Speed = temp_speed_float;
								Sync_Motor_position = (uint32_t)msg->Data[3] | (uint32_t)msg->Data[4] << 8;
                        		}
                        	}
                        }
                        break;

                    case 0x08: // 命令: 多电机同步模式（广播同步执行）
                    	{
                        	if(msg->Rx_Header.Identifier == BROADCAST_ID )
                        	{
                        		if (EMA_DATA.Motor_mode == MOTOR_SYNC_POSITION)
                        		{
                                position_mode_increment = (uint32_t)(((uint64_t)Sync_Motor_Speed * 0x100000000) / interrupt_freq_hz);
                                target_step_position = Sync_Motor_position;
                        		}
                        	}
                        }
                        break;

                    case 0x09: // 命令: 开环速度模式
                    	{
                    			EMA_DATA.Motor_mode = MOTOR_OPEN_VELOCITY;
                            	temp_speed_int16 = (uint16_t)msg->Data[1] | (uint16_t)(msg->Data[2] << 8);
                            	temp_speed_float = temp_speed_int16 / 100.0f;
                            	open_loop_velocity = temp_speed_float;
                                if (open_loop_velocity > 0) { motor_direction = MOTOR_FORWARD; }
                                   else 				  { motor_direction = MOTOR_REVERSE; temp_speed_float = -temp_speed_float;}
                            	velocity_mode_increment = (uint32_t)(((uint64_t)temp_speed_float * 0x100000000) / interrupt_freq_hz);
//                            	printf("%d,%ld\r\n",open_loop_velocity,velocity_mode_increment);
                        }
                        break;

                    default:
                    	status_code = 0x01;
                        printf("ERR: Unknown Command\r\n");
                        break;
                }


	                  // 5. 【请求应答】(只对应答非广播消息)
	                  if (msg->Rx_Header.Identifier != BROADCAST_ID)
	                  {
	                      Queue_Reply_Request(command, status_code);
	                      if(EMA_DATA.Motor_mode == MOTOR_OPEN_REPEATED)  {HAL_TIM_Base_Start_IT(&htim16);}
	                      else {HAL_TIM_Base_Stop(&htim16);}
	                  }
            }
            else
            {
//	                  // ID 不匹配，打印错误
//	                  printf("ID error\r\n");
            }

            // 6. 【关键】处理完毕，移动 Rx 队列的 "tail" 指针
            g_rx_queue_tail = (g_rx_queue_tail + 1) & (RX_QUEUE_SIZE - 1);
        }
}











