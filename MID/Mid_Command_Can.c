/**
 ******************************************************************************
 * @file    Mid_Command_Can.c (原 command.c)
 * @author  Letian
 * @date    Sep 2, 2025
 * @brief   CAN 通信协议解析与业务分发中间件 (MID)
 * @note    负责从 CAN 接收环形队列中提取报文，解析 0x01~0x09 协议指令，
 * 并调度 APP 层电机状态机执行相应动作。
 ******************************************************************************
 */

#include "Mid_Command_Can.h"
#include "Bsp_Can.h"
#include "Mid_Control.h"
#include "App_EMA.h"
#include "App_Fault.h"
#include "Bsp_Control.h"

/* ==========================================
 * 外部依赖声明 (Extern Declarations)
 * ========================================== */
extern Motor EFA_DATA;

/* ==========================================
 * 全局变量定义 (Global Variables)
 * ========================================== */
/* 储存速度计算临时值 */
float    temp_speed_float;
int16_t  temp_speed_int16;

/* 定义一个全局错误计数器，用于替代危险的 printf */
uint32_t g_Can_Tx_Drop_Count = 0;

/* 多电机同步模式参数 */
float    Sync_Motor_Speed = 0;
uint32_t Sync_Motor_position = 0;

/* ==========================================
 * 环形队列外部声明 (Ring Buffer Externs)
 * ========================================== */
/**
 * @brief  定义队列的存储数组
 * @note   volatile 关键字是必须的，因为它在中断和主循环中被同时访问
 */
extern volatile CanTxMessage_t g_Tx_Queue[TX_QUEUE_SIZE];
extern volatile CanRxMessage_t g_Rx_Queue[RX_QUEUE_SIZE];

/**
 * @brief  定义队列的 "头指针" (Head)
 * @note   'head' 是数据“放入”的位置。只有 "生产者" (中断) 可以修改它。
 */
extern volatile uint16_t g_Tx_Queue_Head;
extern volatile uint16_t g_Rx_Queue_Head;

/**
 * @brief  定义队列的 "尾指针" (Tail)
 * @note   'tail' 是数据“取出”的位置。只有 "消费者" (main 循环) 可以修改它。
 */
extern volatile uint16_t g_Tx_Queue_Tail;
extern volatile uint16_t g_Rx_Queue_Tail;


/* ========================================================================== *
 * 【核心逻辑】CAN 报文消费者与分发器
 * ========================================================================== */

/**
 * @brief  处理 CAN 接收队列中的报文 (消费者)
 * @note   支持的指令集 (Data[0]):
 * 0x01: 紧急停止 (Emergency Stop)
 * 0x02: 高压上电 (Power on)
 * 0x03: 步进+ (相对位置正向运动)
 * 0x04: 步进- (相对位置反向运动)
 * 0x05: 往复模式 (Reciprocating Mode)
 * 0x06: 绝对位置模式 (Absolute Position Mode)
 * 0x07: 多电机同步模式 - 仅接收数据
 * 0x08: 多电机同步模式 - 广播同步触发
 * 0x09: 开环速度模式
 */
void Can_message_process(void)
{
    /*----------------------------------------------------------------*/
    /* --- 消费者 (Consumer) #1: 处理接收队列 (g_Rx_Queue) --- */
    /*----------------------------------------------------------------*/

    /* 1. 检查队列是否为空 */
    if (g_Rx_Queue_Head != g_Rx_Queue_Tail)
    {
        /* 2. 队列不为空，从 "tail" 处取出消息 */
        volatile CanRxMessage_t* msg = &g_Rx_Queue[g_Rx_Queue_Tail];

        /* ========================================================== */
        /* [DEBUG 预留] 打印原始报文 */

         printf("Rx ID:0x%lX,Data ", msg->Rx_Header.Identifier);
         for(int i=0; i < msg->Rx_Header.DataLength; i++) { printf("%02X ", msg->Data[i]); }
         printf("\r\n");

        /* ========================================================== */

        /* 3. 检查 ID (软件滤波器) */
        if ((msg->Rx_Header.Identifier == MY_NODE_ID) || (msg->Rx_Header.Identifier == BROADCAST_ID))
        {
            uint8_t command = msg->Data[0];
            uint8_t status_code = 0x00; /* 默认状态码 = 0x00 (OK_Accepted), 0x01 为无对应指令 */

            /* ========================================================== */
            /* [DEBUG 预留] 打印命中本机 ID 的信息 */
            /*
            if(msg->Rx_Header.Identifier == MY_NODE_ID) {
                // printf("%d,%2lX\r\n",msg->Data[3],msg->Rx_Header.Identifier);
            }
            */
            /* ========================================================== */

            /* 4. 【安全地】执行业务逻辑分发 */
            switch (command)
            {
                case 0x01: /* 命令: 紧急停止 */
                    Bsp_Dc_Power_Control(false);
                    HAL_TIM_Base_Stop(&htim16);
                    Bsp_Close_All_Output();
                    if ((EFA_DATA.Fault_Flags != FAULT_NONE) ||
                        (EFA_DATA.Motor_mode != MOTOR_ERROR))
                    {
                        EFA_DATA.Motor_mode = MOTOR_IDLE;
                    }
                    break;

                case 0x02: /* 命令: 高压上电 */
                    Bsp_Dc_Power_Control(true);
                    EFA_DATA.Motor_mode = MOTOR_READY;
                    break;

                case 0x03: /* 命令: 步进+ */
                case 0x04: /* 命令: 步进- */
                {
                    /* 小端模式拼接 16 位速度并转为浮点数 */
                    temp_speed_int16 = (uint16_t)msg->Data[1] | (uint16_t)(msg->Data[2] << 8);
                    temp_speed_float = temp_speed_int16 / 100.0f;

                    position_mode_increment = (uint32_t)(((uint64_t)temp_speed_float * 0x100000000) / interrupt_freq_hz);
                    EFA_DATA.Motor_mode = MOTOR_OPEN_POSITION;

                    printf("command:%d,speed %.2f\r\n", command, temp_speed_float);

                    if (command == 0x03) /* 正向 */
                    {
                        target_step_position = absolute_step_counter + 1;
                    }
                    else /* 反向 (command == 0x04) */
                    {
                        target_step_position = absolute_step_counter - 1;
                    }
                    break;
                }

                case 0x05: /* 命令: 往复模式 */
                {
                    EFA_DATA.Motor_mode = MOTOR_OPEN_REPEATED;

                    temp_speed_int16 = (uint16_t)msg->Data[1] | (uint16_t)(msg->Data[2] << 8);
                    temp_speed_float = temp_speed_int16 / 100.0f;
                    position_mode_increment = (uint32_t)(((uint64_t)temp_speed_float * 0x100000000) / interrupt_freq_hz);

                    repeated_pos_A = (int32_t)((uint16_t)msg->Data[3] | (uint16_t)(msg->Data[4] << 8));
                    repeated_pos_B = (int32_t)((uint16_t)msg->Data[5] | (uint16_t)(msg->Data[6] << 8));
                    repeated_count_total = (uint32_t)msg->Data[7];

                    /* 如果往复次数为 0，则设定为无穷往复 */
                    if(repeated_count_total == 0)
                    {
                        repeated_count_total = 0xFFFFFFFF;
                    }
                    repeated_count_current = 0;

                    printf("command:%d,speed_hz %.2f,%ld,%ld\r\n", command, temp_speed_float, repeated_pos_A, repeated_pos_B);
                    break;
                }

                case 0x06: /* 命令: 绝对位置模式 */
                {
                    EFA_DATA.Motor_mode = MOTOR_OPEN_POSITION;

                    temp_speed_int16 = (uint16_t)msg->Data[1] | (uint16_t)(msg->Data[2] << 8);
                    temp_speed_float = temp_speed_int16 / 100.0f;
                    position_mode_increment = (uint32_t)(((uint64_t)temp_speed_float * 0x100000000) / interrupt_freq_hz);

                    uint32_t absolute_pos = (uint32_t)msg->Data[3] | (uint32_t)msg->Data[4] << 8;
                    target_step_position = absolute_pos;
                    break;
                }

                case 0x07: /* 命令: 多电机同步模式 (参数接收) */
                {
                    if (msg->Rx_Header.Identifier == MY_NODE_ID)
                    {
                            EFA_DATA.Motor_mode = MOTOR_SYNC_POSITION;

                            temp_speed_int16 = (uint16_t)msg->Data[1] | (uint16_t)(msg->Data[2] << 8);
                            temp_speed_float = temp_speed_int16 / 100.0f;
                            Sync_Motor_Speed = temp_speed_float;

                            Sync_Motor_position = (uint32_t)msg->Data[3] | (uint32_t)msg->Data[4] << 8;
                    }
                    break;
                }

                case 0x08: /* 命令: 多电机同步模式 (广播同步执行) */
                {
                    if (msg->Rx_Header.Identifier == BROADCAST_ID)
                    {
                        if (EFA_DATA.Motor_mode == MOTOR_SYNC_POSITION)
                        {
                            position_mode_increment = (uint32_t)(((uint64_t)Sync_Motor_Speed * 0x100000000) / interrupt_freq_hz);
                            target_step_position = Sync_Motor_position;
                        }
                    }
                    break;
                }

                case 0x09: /* 命令: 开环速度模式 */
                {
                    EFA_DATA.Motor_mode = MOTOR_OPEN_VELOCITY;

                    temp_speed_int16 = (uint16_t)msg->Data[1] | (uint16_t)(msg->Data[2] << 8);
                    temp_speed_float = temp_speed_int16 / 100.0f;
                    open_loop_velocity = temp_speed_float;

                    if (open_loop_velocity > 0)
                    {
                        motor_direction = MOTOR_FORWARD;
                    }
                    else
                    {
                        motor_direction = MOTOR_REVERSE;
                        temp_speed_float = -temp_speed_float; /* 取绝对值供后续增量计算 */
                    }

                    velocity_mode_increment = (uint32_t)(((uint64_t)temp_speed_float * 0x100000000) / interrupt_freq_hz);
                    // printf("%d,%ld\r\n", open_loop_velocity, velocity_mode_increment);
                    break;
                }

                default:
                    status_code = 0x01;
                    printf("ERR: Unknown Command\r\n");
                    break;
            }

            /* 5. 【请求应答】(只对应答非广播消息) */
            if (msg->Rx_Header.Identifier != BROADCAST_ID)
            {
                Queue_Reply_Request(command, status_code);

                /* 根据模式启停相关定时器功能 */
                if (EFA_DATA.Motor_mode == MOTOR_OPEN_REPEATED)
                {
                    HAL_TIM_Base_Start_IT(&htim16);
                }
                else
                {
                    HAL_TIM_Base_Stop(&htim16);
                }
            }
        }
        else
        {
            /* ========================================================== */
            /* [DEBUG 预留] ID 不匹配日志 */
            /*
            // printf("ID error\r\n");
            */
            /* ========================================================== */
        }

        /* 6. 【关键】处理完毕，移动 Rx 队列的 "Tail" 指针 */
        /* 使用位运算提升效率，并正式将该条数据从队列中移出 */
        g_Rx_Queue_Tail = (g_Rx_Queue_Tail + 1) & (RX_QUEUE_SIZE - 1);
    }
}

/**
 * @brief  CAN 发送队列清空任务 (MID 层)
 * @note   不断检查软件环形发送队列，如果有数据，则尝试推给 BSP 层发送。
 * 需放置在 main() 函数的 while(1) 主循环中高速轮询。
 */
void Mid_Can_Tx_Task(void)
{
    /* 1. 检查软件环形队列是否为空 */
    if (g_Tx_Queue_Head != g_Tx_Queue_Tail)
    {
        /* 2. 获取队尾 (Tail) 准备发送的数据包指针 */
        CanTxMessage_t* msg_to_send = (CanTxMessage_t*)&g_Tx_Queue[g_Tx_Queue_Tail];

        /* 3. 尝试调用 BSP 层硬件接口发送 */
        if (Bsp_Can_Transmit_Message(msg_to_send) == true)
        {
            /* 4. 【关键】硬件接收成功，【原子操作】移动尾指针，完成出队 */
            g_Tx_Queue_Tail = (g_Tx_Queue_Tail + 1) & (TX_QUEUE_SIZE - 1);
        }
        else
        {
            /* 5. 硬件 FIFO 满碌或离线 (不移动尾指针，下次循环重试) */
            /* 【安全拦截】：这里绝对不能用 printf！我们只做静默计数 */
            g_Can_Tx_Drop_Count++;

            /* 如果确实需要报警，可以通过状态机将故障抛给 APP 层，
             * 比如当 g_Can_Tx_Drop_Count 超过 10000 时，触发总线离线故障。*/
        }
    }
}

