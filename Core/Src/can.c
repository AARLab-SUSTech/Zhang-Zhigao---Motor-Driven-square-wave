/*
 * can.c
 *
 *  Created on: Oct 11, 2025
 *      Author: Letian
 */
#include "can.h"
#include "control.h"

extern volatile Motor_mode_t Motor_mode;

FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader; // 用于存储接收报文的头信息
uint8_t RxData[8];              // 用于存储接收报文的数据
uint8_t TxData[8];
uint16_t Sync_Motor_Speed=0;
uint32_t Sync_Motor_position=0;
uint16_t open_loop_velocity=0;

/**
  * @brief  定义队列的存储数组
  * volatile 关键字是必须的，因为它在中断和主循环中被同时访问
  */
volatile CanTxMessage_t g_tx_queue[TX_QUEUE_SIZE];
volatile CanRxMessage_t g_rx_queue[RX_QUEUE_SIZE];
/**
  * @brief  定义队列的 "头指针" (Head)
  * 'head' 是数据“放入”的位置
  * 只有 "生产者" (中断) 可以修改它
  */
volatile uint16_t g_tx_queue_head = 0;
volatile uint16_t g_rx_queue_head = 0;
/**
  * @brief  定义队列的 "尾指针" (Tail)
  * 'tail' 是数据“取出”的位置
  * 只有 "消费者" (main 循环) 可以修改它
  */
volatile uint16_t g_tx_queue_tail = 0;
volatile uint16_t g_rx_queue_tail = 0;

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
        printf("!!! FATAL ERROR: CAN BUS-OFF DETECTED !!!\r\n");
        printf("!!! 检查终端电阻、ACK应答节点、波特率 !!!\r\n");
        Buzzer_ON; // 蜂鸣器报警
}

// ... (您已有的 HAL_TIM_PeriodElapsedCallback) ...

/* USER CODE END 0 */


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


// FDCAN接收FIFO 0消息挂起回调函数
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    // 检查是否是“新消息到达”中断
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        // 从Rx FIFO 0中获取消息，存入上面的全局变量中
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
        {
////             --- 在这里添加您自己的数据处理逻辑 ---
//            printf("Message Received!\r\n");
//            printf("  ID   : 0x%lX\r\n", RxHeader.Identifier);
//            printf("  DLC  : %ld bytes\r\n", RxHeader.DataLength);
//            printf("  Data : ");
//            for(int i=0; i < RxHeader.DataLength; i++)
//            {
//                printf("0x%02X ", RxData[i]);
//            }
//            printf("\r\n");

        	            // 2. 计算 g_rx_queue 队列的下一个 "head" 指针的位置
        	            //    (使用位运算 & (SIZE - 1) 比 % 更高效)
        	            uint16_t next_head = (g_rx_queue_head + 1) & (TX_QUEUE_SIZE - 1);
        	                                    // 注意：这里我们假设 Rx 队列和 Tx 队列一样大

        	            // 3. 检查 Rx 队列是否已满 (如果 head 的下一个位置就是 tail)
        	            if (next_head == g_rx_queue_tail)
        	            {
        	                // 队列已满，本次接收到的报文被丢弃
        	                // (这是“消费者” main() 处理太慢的信号)
        	                // (您可以在这里设置一个错误标志，例如 g_error_rx_overflow = true;)
        	            }
        	            else
        	            {
        	                // 4. 队列未满，获取 "head" 位置的“集装箱”
        	                volatile CanRxMessage_t* msg_to_queue = &g_rx_queue[g_rx_queue_head];

        	                // 5. 将临时数据从 RxHeader/RxData 复制到队列中
        	                msg_to_queue->Rx_Header = RxHeader; // 复制整个报文头 (包含ID, DLC等)

        	                // 复制8字节的数据
        	                for (int i = 0; i < RxHeader.DataLength; i++)
        	                {
        	                    msg_to_queue->Data[i] = RxData[i];
        	                }

        	                // 6. 【原子操作】移动头指针，正式将消息放入队列
        	                //    这个赋值操作是原子的，main 循环现在就能看到新消息了
        	                g_rx_queue_head = next_head;
        	            }
        	        // else
        	        // {
        	        //    // HAL_FDCAN_GetRxMessage 失败 (罕见)
        	        // }s

        }

        // 【非常重要】: 每次处理完中断后，必须重新激活通知，否则中断只会触发一次！
        if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
        {
           Error_Handler();
        }
    }
}



/**
  * @brief  【生产者 #2】: 请求发送一个“应答包” (在 main 循环中调用)
  * @param  command: 原始命令 (用于回显)
  * @param  status: 状态码 (0x00=OK, 0x01=未知命令, 0x02=参数错误)
  * @retval true: 入队成功, false: 队列已满
  */
bool Queue_Reply_Request(uint8_t command, uint8_t status)
{
    // 1. 计算 g_tx_queue 队列的下一个 "head" 指针的位置
    uint16_t next_head = (g_tx_queue_head + 1) & (TX_QUEUE_SIZE - 1);

    // 2. 检查 Tx 队列是否已满
    if (next_head == g_tx_queue_tail)
    {
        // 队列已满，本次应答被丢弃
        return false;
    }

    // 3. 队列未满，获取 "head" 位置的“集装箱”
    volatile CanTxMessage_t* msg_to_queue = &g_tx_queue[g_tx_queue_head];

    // 4. 填充报文头 (Tx_Header)
    msg_to_queue->Tx_Header = TxHeader;      // 复制模板
    msg_to_queue->Tx_Header.Identifier = MESSAGE_ID; // 【使用 MESSAGE_ID 宏 (0x201)】
    msg_to_queue->Tx_Header.DataLength = FDCAN_DLC_BYTES_2; // 【灵活DLC】只发2字节

    // 5. 填充报文数据
    msg_to_queue->Data[0] = command; // Data[0] = 回显收到的命令
    msg_to_queue->Data[1] = status;  // Data[1] = 执行状态码
    // (其余字节自动被忽略)

    // 6. 【原子操作】移动头指针，正式将消息放入队列
    g_tx_queue_head = next_head;

    return true;
}

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
uint8_t r_d[3] = {0,0,0};
void Can_message_process(void)
{
    /*----------------------------------------------------------------*/
        /* --- 消费者 (Consumer) #1: 处理接收队列 (g_rx_queue) --- */
        /*----------------------------------------------------------------*/
        if (g_rx_queue_head != g_rx_queue_tail)
        {
            // 1. 队列不为空，从 "tail" 处取出消息
            volatile CanRxMessage_t* msg = &g_rx_queue[g_rx_queue_tail];


//            if((msg->Rx_Header.Identifier == 0x101 ))
//            {
//            	r_d[0] = msg->Data[3];
//            }
//            else if((msg->Rx_Header.Identifier == 0x102 ))
//            {
//            	r_d[1] = msg->Data[3];
//            }
//            else if((msg->Rx_Header.Identifier == 0x103 ))
//            {
//            	r_d[2] = msg->Data[3];
//            	printf("%d,%d,%d\r\n",r_d[0],r_d[1],r_d[2]);
//            }


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
                        Close_output();
                        if( (Motor_mode != MOTOR_OVER_HV_VOLTAGE) && (Motor_mode != MOTOR_OVER_HV_CURRENT) &&
                            (Motor_mode != MOTOR_OVER_DC_IN_CURRENT) && (Motor_mode != MOTOR_ERROR) )
                        {
                            Motor_mode = MOTOR_IDLE;
                        }
                        break;

                    case 0x02: // 命令: 高压上电
                        DC_Power_ON;
                        Motor_mode = MOTOR_READY;
                        break;

                    case 0x03: // 命令: 步进+
                    case 0x04: // 命令: 步进-
                        {
                            uint16_t speed_hz = (uint16_t)msg->Data[1] | (uint16_t)(msg->Data[2] << 8);
                            uint32_t steps = 1;

                            position_mode_increment = (uint32_t)(((uint64_t)speed_hz * 0x100000000) / interrupt_freq_hz);
                            Motor_mode = MOTOR_OPEN_POSITION;
                            printf("command:%d,speed %d,step%ld\r\n",command,speed_hz,steps);
                            if (command == 0x03) // 正向
                               target_step_position = absolute_step_counter + steps;
                            else // 反向 (command == 0x04)
                               target_step_position = absolute_step_counter - steps;
                        }
                        break;

                    case 0x05: // 命令: 往复模式
                        {
                            uint16_t speed_hz = (uint16_t)msg->Data[1] | (uint16_t)(msg->Data[2] << 8);
                            repeated_pos_A = (int32_t)((uint16_t)msg->Data[3] | (uint16_t)(msg->Data[4] << 8));
                            repeated_pos_B = (int32_t)((uint16_t)msg->Data[5] | (uint16_t)(msg->Data[6] << 8));
                            repeated_count_total = (uint32_t)msg->Data[7];
                            repeated_count_current = 0;
                            printf("command:%d,speed_hz %d,%ld,%ld\r\n",command,speed_hz,repeated_pos_A,repeated_pos_B);
                            position_mode_increment = (uint32_t)(((uint64_t)speed_hz * 0x100000000) / interrupt_freq_hz);
                            Motor_mode = MOTOR_OPEN_REPEATED;
                        }
                        break;

                    case 0x06: // 命令: 绝对位置模式
                        {
                            uint16_t speed_hz = (uint16_t)msg->Data[1] | (uint16_t)(msg->Data[2] << 8);
                            uint32_t absolute_pos = (uint32_t)msg->Data[3] | (uint32_t)msg->Data[4] << 8;

                            position_mode_increment = (uint32_t)(((uint64_t)speed_hz * 0x100000000) / interrupt_freq_hz);
                            Motor_mode = MOTOR_OPEN_POSITION;
                            target_step_position = absolute_pos;
                        }
                        break;

                    case 0x07: // 命令: 多电机同步模式
                        {
                        	if(msg->Rx_Header.Identifier == MY_NODE_ID )
                        	{
                        		if((Motor_mode != MOTOR_OVER_HV_CURRENT) | (Motor_mode != MOTOR_OVER_HV_VOLTAGE) | (Motor_mode != MOTOR_OVER_DC_IN_CURRENT))
                        		{
								Motor_mode = MOTOR_SYNC_POSITION;
								Sync_Motor_Speed = (uint16_t)msg->Data[1] | (uint16_t)(msg->Data[2] << 8);
								Sync_Motor_position = (uint32_t)msg->Data[3] | (uint32_t)msg->Data[4] << 8;
                        		}
                        	}
                        }
                        break;

                    case 0x08: // 命令: 多电机同步模式（广播同步执行）
                    	{
                        	if(msg->Rx_Header.Identifier == BROADCAST_ID )
                        	{
                        		if (Motor_mode == MOTOR_SYNC_POSITION)
                        		{
                                position_mode_increment = (uint32_t)(((uint64_t)Sync_Motor_Speed * 0x100000000) / interrupt_freq_hz);
                                target_step_position = Sync_Motor_position;
                        		}
                        	}
                        }
                        break;

                    case 0x09: // 命令: 开环速度模式
                    	{
                    			Motor_mode = MOTOR_OPEN_VELOCITY;
                    			open_loop_velocity = (uint16_t)msg->Data[1] | (uint16_t)(msg->Data[2] << 8);
                        		velocity_mode_increment = (uint32_t)(((uint64_t)open_loop_velocity * 0x100000000) / interrupt_freq_hz);
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






