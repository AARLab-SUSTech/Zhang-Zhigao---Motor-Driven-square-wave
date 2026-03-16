/**
 ******************************************************************************
 * @file    command.c
 * @author  Letian
 * @date    Sep 2, 2025
 * @brief   串口通信协议中间件 (MID)
 * @note    负责 UART DMA 发送缓冲管理、IDLE 接收中断处理、
 * 以及核心的通信协议帧解析与业务指令分发 (Command Dispatcher)。
 ******************************************************************************
 */

#include <Mid_Command_Usart.h>
#include <Mid_Control.h>
#include "string.h"
#include "Bsp_Usart.h"
#include "App_EMA.h"

/* 定义 Vofa+ JustFloat 协议的尾部帧 */
const uint8_t JUST_FLOAT_TAIL[4] = {0x00, 0x00, 0x80, 0x7F};

/* ==========================================
 * 外部依赖声明 (Extern Declarations)
 * ========================================== */
extern Motor EMA_DATA;

extern uint8_t rx_buffer[RX_BUFFER_SIZE];      /* 定义在 Bsp_Usart.c，256 字节 */
extern uint8_t process_buffer[RX_BUFFER_SIZE]; /* 定义在 Bsp_Usart.c，256 字节 */

/* --- 串口 DMA 接收打印相关 --- */
extern int dma_print_flag;


/* ==========================================
 * 全局变量定义 (Global Variables)
 * ========================================== */
#define UART_TX_BUFFER_SIZE 512                /* 定义发送缓冲区大小，确保足够长 */
uint8_t g_uart_tx_buffer[UART_TX_BUFFER_SIZE]; /* DMA 发送缓冲区 */

/**
 * @brief DMA 传输状态标志
 * @note  volatile 关键字很重要，因为它在主程序和中断服务程序中同时被访问。
 * 1: 空闲 (Idle), 0: 忙碌 (Busy)
 */
volatile uint8_t g_uart_dma_transfer_complete = 1;


/* ========================================================================== *
 * 【发送处理】DMA 串口发送相关函数
 * ========================================================================== */

/**
 * @brief  UART DMA 发送完成回调函数
 * @param  huart UART 句柄
 * @note   由 HAL 库在 DMA 传输完成中断中自动调用
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    /* 检查是哪个 UART 完成了发送 */
    if (huart->Instance == USART1)
    {
        /* 将 DMA 状态标志恢复为完成 (空闲) */
        g_uart_dma_transfer_complete = 1;
    }
}

/**
 * @brief  使用 DMA 发送一个浮点数数组 (转换为 ASCII 文本)
 * @param  arr   指向浮点数数组的指针
 * @param  count 数组中浮点数的个数
 * @note   格式为: "1.23,4.56,7.89\r\n"
 */
void Mid_Float_Array_Dma_Send(float* arr, int count)
{
    /* 1. 检查 DMA 是否空闲 */
    if (g_uart_dma_transfer_complete == 0)
    {
        Buzzer_ON; /* Error: DMA 发送错误，进入报错模式 */
        return;    /* DMA 忙，直接返回 */
    }

    if (count <= 0)
    {
        return; /* 没有数据需要发送 */
    }

    g_uart_dma_transfer_complete = 0; /* 设置为忙碌状态 */

    /* 2. 动态构建格式化字符串 */
    int current_len = 0;                      /* 当前已写入缓冲区的长度 */
    int remaining_size = UART_TX_BUFFER_SIZE; /* 缓冲区剩余大小 */

    for (int i = 0; i < count; i++)
    {
        int written_len;

        /* 判断是否为最后一个元素，决定是加逗号还是换行符 */
        if (i < count - 1)
        {
            written_len = snprintf((char*)g_uart_tx_buffer + current_len, remaining_size, "%.2f,", arr[i]);
        }
        else
        {
            written_len = snprintf((char*)g_uart_tx_buffer + current_len, remaining_size, "%.2f\r\n", arr[i]);
        }

        /* 检查 snprintf 是否成功，以及是否超出缓冲区 */
        if (written_len <= 0 || written_len >= remaining_size)
        {
            Buzzer_ON; /* Error: 超出缓存区 */
            g_uart_dma_transfer_complete = 1; /* 释放 DMA */
            return;
        }

        /* 更新长度和剩余空间 */
        current_len += written_len;
        remaining_size -= written_len;
    }

    /* 3. 启动 DMA 传输 */
    if (current_len > 0)
    {
        HAL_UART_Transmit_DMA(&huart1, g_uart_tx_buffer, current_len);
    }
    else
    {
        /* 如果因为某种原因没有内容被写入，则直接释放 DMA */
        g_uart_dma_transfer_complete = 1;
    }
}

/**
 * @brief  将一个大数组按指定分组大小格式化，并使用 DMA 一次性发送
 * @param  arr                指向 int16_t 数组的指针
 * @param  total_count        数组中元素的总个数
 * @param  elements_per_group 每组包含多少个元素 (用于插入换行符)
 * @note   警告：此函数混合了二进制数据(int16_t)和ASCII字符(',' '\n')，
 * 如果二进制数据中碰巧包含 0x2C (逗号) 或 0x0A (换行)，可能导致上位机解析混乱。
 */
void Mid_Int16_Groups_Dma_Send(int16_t* arr, int total_count, int elements_per_group)
{
    /* 1. 检查 DMA 是否空闲 */
    if (g_uart_dma_transfer_complete == 0)
    {
        Buzzer_ON; /* Error: DMA 忙碌请检查 */
        return;    /* DMA 忙，丢弃本次任务 */
    }

    if (total_count <= 0 || elements_per_group <= 0)
    {
        return; /* 无效参数 */
    }

    /* --- 缓冲区大小检查 (非常重要!) ---
     * 每个元素占 2字节(int16_t) + 1字节(分隔符) = 3字节
     * 确保定义的 UART_TX_BUFFER_SIZE 足够大
     */
    if (total_count * 3 > UART_TX_BUFFER_SIZE)
    {
        Buzzer_ON; /* Error: 缓存区溢出 */
        return;
    }

    /* 2. 将 DMA 状态标志设置为“忙碌” */
    g_uart_dma_transfer_complete = 0;

    /* 3. 【核心修改】使用 memcpy 和指针高效构建数据流 */
    uint8_t* p_write = g_uart_tx_buffer; /* 创建一个写入指针，指向缓冲区的开头 */

    for (int i = 0; i < total_count; i++)
    {
        /* **步骤 A: 搬运二进制数据** */
        /* 使用 memcpy 将2个字节的 int16_t 原封不动地复制到缓冲区 */
        memcpy(p_write, &arr[i], sizeof(int16_t));
        p_write += sizeof(int16_t); /* 将写入指针向前移动2个字节 */

        /* **步骤 B: 添加分隔符** */
        /* 判断是否为一组的末尾，或者整个大数组的末尾 */
        if (((i + 1) % elements_per_group == 0) || (i == total_count - 1))
        {
            *p_write++ = '\n'; /* 写入1个字节的换行符，并使指针+1 */
        }
        else
        {
            *p_write++ = ',';  /* 不是末尾，写入1个字节的逗号，并使指针+1 */
        }
    }

    /* 计算最终写入的数据总长度 */
    int current_len = p_write - g_uart_tx_buffer;

    /* 4. 启动 DMA 传输 */
    if (current_len > 0)
    {
        HAL_UART_Transmit_DMA(&huart1, g_uart_tx_buffer, current_len);
    }
    else
    {
        g_uart_dma_transfer_complete = 1; /* 异常保护：释放锁 */
    }
}


/* ========================================================================== *
 * 【接收与解析】通信协议核心逻辑
 * ========================================================================== */

/**
 * @brief  UART 接收事件空闲中断回调函数
 * @param  huart UART 句柄指针
 * @param  size  本次 DMA 接收到的实际有效字节数
 * @note   该函数实现了“DMA + IDLE (空闲中断)”的变长数据接收逻辑
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    /* 1. 安全性检查：确保是指令串口 */
    if (huart->Instance == USART1)
    {
        /* 2. 调用 MID 层协议解析器。直接将收到的原始缓冲区和长度交给中层，不在此处做逻辑判断 */
        if (size > 0)
        {
            Mid_Process_Usart_Data(rx_buffer, size);
        }

        /* 3. 重新启动 DMA 接收。ReceiveToIdle 会在收到 IDLE 信号时触发本回调 */
        HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_buffer, RX_BUFFER_SIZE);

        if (status != HAL_OK)
        {
            /* 错误处理：可以在此处添加日志或硬件报错逻辑 */
            // Error_Handler();
        }

        /* 4. 关键：禁用 DMA 半传输中断 (Half-Transfer)。
         * 如果不禁用，当缓冲区填满一半时也会触发回调，导致一帧长指令被拆成两次解析 */
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
}

/**
 * @brief  解析接收到的串口指令帧
 * @param  data 指向接收缓冲区的指针
 * @param  size 本次接收到的实际总字节数
 * @note   通信协议帧格式定义：
 * [0] [1] : 帧头 (0x55 0xAA)
 * [2]     : 载荷数据长度 (从 ID 开始一直到数据段结束)
 * [3]     : 设备 ID (ID = 0x01)
 * [4]     : 指令码 (Command)
 * [n]     : 各种数据 (Index, Data...)
 * [末尾]  : 校验和 (计算从 长度位[2] 到 数据段末尾)
 */
void Mid_Process_Usart_Data(uint8_t* data, uint16_t size)
{
    /* 1. 验证基本帧长 */
    if (size < 6)
    {
        return; /* 帧太短，不可能是完整指令 */
    }

    /* ========================================================== */
    /* [DEBUG 预留] 打印接收到的原始数据 */
    /*
    printf("Received data (size = %d): \r\n", size);
    for (uint16_t i = 0; i < size; i++) {
        printf("%02X ", data[i]);
    }
    printf("\r\n");
    */
    /* ========================================================== */

    /* 2. 验证帧头 */
    if (data[0] == 0x55 && data[1] == 0xAA)
    {
        /* 3. 提取长度并验证完整性 */
        uint8_t data_length = data[2];  /* 载荷数据长度字段 */

        /* 总长度 = 帧头(2) + 长度(1) + 额外位(1) + 载荷长度 + 校验和(1) */
        uint16_t total_length = 2 + 1 + 1 + data_length + 1;

        if (size >= total_length)
        {
            /* 4. 校验和 (Checksum) 计算 */
            uint8_t checksum = 0;
            for (uint16_t i = 2; i < 2 + 1 + 1 + data_length; i++)
            {
                checksum += data[i];  /* 累加除帧头(0,1)之外的所有字节，直到校验和之前 */
            }

            /* 获取发送方传来的校验和 */
            uint8_t received_checksum = data[total_length - 1];

            if (checksum == received_checksum)
            {
                /* ========================================================== */
                /* [DEBUG 预留] 打印解析成功的指令 */
                /*
                printf("ID: %02X, Command: %02X, Index: %02X Data: ", data[3], data[4], data[5]);
                for (uint8_t i = 0; i < data_length-2; i++) {
                    printf("%02X ", data[6 + i]);
                }
                printf("\r\n");
                */
                /* ========================================================== */

                /* 5. 设备 ID 校验 */
                if(data[3] != 0x01) return; /* 不是发给本设备的，直接退出 */

                /* 6. 安全机制：如果电机正处于致命错误状态，拒绝执行任何指令 */
                if((EMA_DATA.Motor_mode == MOTOR_OVER_HV_VOLTAGE) ||
                   (EMA_DATA.Motor_mode == MOTOR_OVER_HV_CURRENT) ||
                   (EMA_DATA.Motor_mode == MOTOR_OVER_DC_IN_CURRENT))
                {
                    return; /* 不处理指令，直接退出 */
                }

                /* 7. 状态预处理：如果是运行指令，则打开电源并置位 Ready */
                if(data[4] != 0x25)
                {
                    DC_Power_ON;
                    EMA_DATA.Motor_mode = MOTOR_READY;
                }

                /* 8. 业务逻辑分发 (Command Dispatcher) */
                switch(data[4])
                {
                    case 0x24: /* 往复运动模式 (Repeat Mode) */
                        EMA_DATA.Motor_mode = MOTOR_OPEN_REPEATED;
                        /* 注意大小端拼接逻辑 */
                        temp_speed_int16 = (int16_t)((data[9] << 8) | data[8]);
                        temp_speed_float = (float)temp_speed_int16;
                        /* 提取各个参数 */
                        uint16_t count_val = ((uint16_t)data[9] << 8) | data[10];

                        move_repeatedly(data[7], data[6] / 2, count_val, temp_speed_float);
                        // printf("min:%d, max:%d, count:%d, speed:%.2f\r\n", data[7], data[6], count_val, temp_speed_float);
                        break;

                    case 0x25: /* 停止模式 (STOP) */
                        DC_Power_OFF;
                        /* 【安全修复】：逻辑非的并列必须用 && */
                        if((EMA_DATA.Motor_mode != MOTOR_OVER_HV_VOLTAGE) &&
                           (EMA_DATA.Motor_mode != MOTOR_OVER_HV_CURRENT) &&
                           (EMA_DATA.Motor_mode != MOTOR_OVER_DC_IN_CURRENT) &&
                           (EMA_DATA.Motor_mode != MOTOR_ERROR))
                        {
                            EMA_DATA.Motor_mode = MOTOR_IDLE;
                        }
                        Close_output();
                        dma_print_flag = 0;
                        break;

                    case 0x27: /* 开环位置模式 (Open Position) */
                        EMA_DATA.Motor_mode = MOTOR_OPEN_POSITION;
                        temp_speed_int16 = (int16_t)((data[8] << 8) | data[7]);
                        temp_speed_float = (float)temp_speed_int16;

                        move_to_position(data[6], temp_speed_float);
                        // printf("speed:%.f step:%d\r\n", temp_speed_float, data[6]);
                        break;

                    case 0x28: /* 正向单步 (STEP+) */
                        EMA_DATA.Motor_mode = MOTOR_OPEN_POSITION;
                        target_step_position = absolute_step_counter + 1;
                        position_mode_increment = (uint32_t)(((uint64_t)10.0f * 0x100000000ULL) / interrupt_freq_hz);
                        break;

                    case 0x29: /* 反向单步 (STEP-) */
                        EMA_DATA.Motor_mode = MOTOR_OPEN_POSITION;
                        target_step_position = absolute_step_counter - 1;
                        position_mode_increment = (uint32_t)(((uint64_t)10.0f * 0x100000000ULL) / interrupt_freq_hz);
                        break;

                    case 0x30: /* 设置速度 (SET SPEED) - 预留功能 */
                        break;

                    case 0x31: /* 开环速度模式 (Open Velocity Mode) */
                        EMA_DATA.Motor_mode = MOTOR_OPEN_VELOCITY;
                        temp_speed_int16 = (int16_t)((data[6] << 8) | data[5]);
                        temp_speed_float = (float)temp_speed_int16;

                        velocity_mode_increment = (uint32_t)(((uint64_t)temp_speed_float * 0x100000000ULL) / interrupt_freq_hz);
                        // printf("Velocity Mode Target: %f\n", temp_speed_float);
                        break;

                    case 0x32: /* 闭环位置模式 (Close Position Mode) */
                        EMA_DATA.Motor_mode = MOTOR_CLOSE_POSITION;
                        // printf("close position\n");
                        break;

                    case 0x33: /* 闭环力矩模式 (Close Force Mode) */
                        EMA_DATA.Motor_mode = MOTOR_CLOSE_FORCE;
                        // printf("close force\n");
                        break;

                    default:
                        /* 未知指令不处理 */
                        break;
                }
            }
            else
            {
                /* 校验和错误日志 (已注释以保证执行效率，调试时可开启) */
                // printf("Checksum error! Calc: %02X, Recv: %02X\n", checksum, received_checksum);
            }
        }
        else
        {
            /* 长度不匹配日志 */
            // printf("Invalid frame length! Recv: %d, Expected: %d\n", size, total_length);
        }
    }
    else
    {
        /* 帧头错误日志 */
        // printf("Invalid header: %02X %02X\n", data[0], data[1]);
    }

    /* 9. 状态联动：根据进入的模式控制打印输出的启停 */
    if((EMA_DATA.Motor_mode == MOTOR_OPEN_POSITION) || (EMA_DATA.Motor_mode == MOTOR_OPEN_REPEATED))
    {
        dma_print_flag = 1;
        // HAL_TIM_Base_Start_IT(&htim6); // dma printf
    }
}


/**
 * @brief  使用 JustFloat 协议通过 USART DMA 发送浮点数组
 * @param  pData 浮点数数组指针
 * @param  count 浮点数的个数
 * @note   通常用于对接 Vofa+ 等上位机波形显示软件
 */
void Mid_Usart_Send_JustFloat(float *pData, uint8_t count)
{
    /* 1. 检查 DMA 是否空闲 */
    if (g_uart_dma_transfer_complete == 0)
    {
        /* DMA 忙碌，直接丢弃本次遥测数据，保证不卡死主循环 */
        return;
    }

    /* 2. 检查缓冲区是否溢出 (浮点数占 4 字节，尾部占 4 字节) */
    uint16_t TotalBytes = (count * 4) + 4;
    if (TotalBytes > UART_TX_BUFFER_SIZE)
    {
        Buzzer_ON; /* 数据超长报警 */
        return;
    }

    /* 3. 锁定 DMA 状态 */
    g_uart_dma_transfer_complete = 0;

    /* 4. 组装数据帧到发送缓冲区 */
    uint8_t *pWrite = g_uart_tx_buffer;

    /* A. 拷贝浮点数据段 */
    memcpy(pWrite, pData, count * 4);
    pWrite += (count * 4);

    /* B. 拷贝 JustFloat 尾部帧 */
    memcpy(pWrite, JUST_FLOAT_TAIL, 4);

    /* 5. 触发底层 DMA 发送 (真正实现与底层 HAL 库解耦) */
    Bsp_Usart_Send_DMA(g_uart_tx_buffer, TotalBytes);
}



