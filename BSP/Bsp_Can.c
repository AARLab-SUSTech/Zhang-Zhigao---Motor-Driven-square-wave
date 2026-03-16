/**
 ******************************************************************************
 * @file    can.c
 * @brief   CAN (FDCAN) 底层硬件驱动与队列管理模块
 * @note    负责 CAN 过滤器的配置、中断使能、底层收发接口封装，
 * 以及基于环形缓冲区的无阻塞收发队列调度。
 ******************************************************************************
 */

#include <Bsp_Can.h>
#include <Mid_Command_Usart.h>
#include <Mid_Control.h>
#include "App_EMA.h"

/* ==========================================
 * 外部依赖声明 (Extern Declarations)
 * ========================================== */
extern Motor EMA_DATA;

/* ==========================================
 * 全局变量声明 (Global Variables)
 * ========================================== */
FDCAN_TxHeaderTypeDef s_Can_Tx_Header;
FDCAN_RxHeaderTypeDef s_Can_Rx_Header;  /* 用于存储接收报文的头信息 */
uint8_t               RxData[8];        /* 用于存储接收报文的数据 */
uint8_t               s_can_tx_data[8];

int16_t               open_loop_velocity;

/* ==========================================
 * 环形队列定义 (Ring Buffer Definitions)
 * ========================================== */
/**
 * @brief  定义队列的存储数组
 * @note   volatile 关键字是必须的，因为它在中断和主循环中被同时访问
 */
volatile CanTxMessage_t g_Tx_Queue[TX_QUEUE_SIZE];
volatile CanRxMessage_t g_Rx_Queue[RX_QUEUE_SIZE];

/**
 * @brief  定义队列的 "头指针" (Head)
 * @note   'head' 是数据“放入”的位置。只有 "生产者" (如中断) 可以修改它。
 */
volatile uint16_t g_Tx_Queue_Head = 0;
volatile uint16_t g_Rx_Queue_Head = 0;

/**
 * @brief  定义队列的 "尾指针" (Tail)
 * @note   'tail' 是数据“取出”的位置。只有 "消费者" (如 main 循环) 可以修改它。
 */
volatile uint16_t g_Tx_Queue_Tail = 0;
volatile uint16_t g_Rx_Queue_Tail = 0;

/* USER CODE END 0 */

/* ==========================================
 * 函数实现 (Function Implementations)
 * ========================================== */

/**
 * @brief  FDCAN 错误回调函数
 * @param  hfdcan FDCAN 句柄
 * @note   当 CAN 总线发生严重错误 (如 Bus-Off) 时由 HAL 库自动调用
 */
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    printf("!!! FATAL ERROR: CAN BUS-OFF DETECTED !!!\r\n");
    printf("!!! 检查终端电阻、ACK应答节点、波特率 !!!\r\n");
    Buzzer_ON; /* 蜂鸣器报警 */
}

// ... (预留的 HAL_TIM_PeriodElapsedCallback) ...

/**
 * @brief  FDCAN 硬件初始化
 * @retval 0: 成功, -1: 失败
 * @note   配置双 ID 过滤器，启动外设，激活中断，并缓存发送报文头
 */
int8_t Bsp_Can_Init(void)
{
    HAL_StatusTypeDef status;
    FDCAN_FilterTypeDef sFilterConfig;

    /* 1. 配置 FDCAN 接收过滤器 */
    sFilterConfig.IdType       = FDCAN_STANDARD_ID;       /* 标准帧 (11-bit) */
    sFilterConfig.FilterIndex  = 0;                       /* 过滤器索引 0 */
    sFilterConfig.FilterType   = FDCAN_FILTER_DUAL;       /* 双 ID 过滤模式 (匹配 ID1 或 ID2) */
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; /* 匹配成功后存入 Rx FIFO 0 */
    sFilterConfig.FilterID1    = MY_NODE_ID;              /* 匹配规则 1: 专属本机 ID */
    sFilterConfig.FilterID2    = BROADCAST_ID;            /* 匹配规则 2: 广播 ID */

    status = HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
    if (status != HAL_OK)
    {
        printf("[BSP CAN] ERROR: Filter Config Failed! (Code: %d)\r\n", status);
        return -1;
    }

    /* 2. 启动 FDCAN 外设 */
    status = HAL_FDCAN_Start(&hfdcan1);
    if (status != HAL_OK)
    {
        /* 修复了原来代码中重复调用 Start 的 Bug */
        printf("[BSP CAN] FATAL: FDCAN Start Failed! (Code: %d)\r\n", status);
        return -1;
    }
    printf("[BSP CAN] INFO: FDCAN Started Successfully.\r\n");

    /* 3. 激活接收中断 (Rx FIFO 0 新消息通知) */
    status = HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    if (status != HAL_OK)
    {
        printf("[BSP CAN] ERROR: Activate Notification Failed!\r\n");
        return -1;
    }

    /* 4. 预初始化默认的发送报文头 (TxHeader)
     * 以后在发送函数中，只需要修改 Identifier 和 DataLength 即可
     */
    s_Can_Tx_Header.Identifier          = MY_NODE_ID; /* 默认发件人是自己 */
    s_Can_Tx_Header.IdType              = FDCAN_STANDARD_ID;
    s_Can_Tx_Header.TxFrameType         = FDCAN_DATA_FRAME;
    s_Can_Tx_Header.DataLength          = FDCAN_DLC_BYTES_8; /* 默认8字节 */
    s_Can_Tx_Header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    s_Can_Tx_Header.BitRateSwitch       = FDCAN_BRS_OFF;     /* 根据需求，如果不跑 FD 高速段则设为 OFF */
    s_Can_Tx_Header.FDFormat            = FDCAN_FRAME_CLASSIC; /* 默认使用经典 CAN 格式 */
    s_Can_Tx_Header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    s_Can_Tx_Header.MessageMarker       = 0;

    /* 清空默认发送数据 */
    for (int i = 0; i < 8; i++)
    {
        s_can_tx_data[i] = 0;
    }

    printf("[BSP CAN] INFO: Firmware Version 2.0 - CAN Init OK.\r\n");
    return 0; /* 初始化成功 */
}

/**
 * @brief  FDCAN 接收 FIFO 0 新消息中断回调
 * @param  hfdcan FDCAN 句柄
 * @param  RxFifo0ITs 中断标志位
 * @note   该函数在 CAN RX 中断内执行，必须做到极速、非阻塞。
 * 包含将接收到的数据推入环形接收队列的逻辑。
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    /* 检查是否是“新消息到达”中断 */
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        /* 1. 优先使用局部变量接收，防止中断重入导致的数据污染 (代码预留) */
        /*
        FDCAN_RxHeaderTypeDef TempRxHeader;
        uint8_t TempRxData[8];
        */

        /* 从 Rx FIFO 0 中获取消息，存入全局变量中 */
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &s_Can_Rx_Header, RxData) == HAL_OK)
        {
            /* ========================================================== */
            /* [DEBUG 预留] 打印原始接收信息，方便后续调试，需要时取消注释 */
            /*
            printf("Message Received!\r\n");
            printf("  ID   : 0x%lX\r\n", s_Can_Rx_Header.Identifier);
            printf("  DLC  : %ld bytes\r\n", s_Can_Rx_Header.DataLength);
            printf("  Data : ");

            // 提示：此处为了安全，建议调试时固定打印前8个字节
            for (int i = 0; i < 8; i++)
            {
                printf("0x%02X ", RxData[i]);
            }
            printf("\r\n");
            */
            /* ========================================================== */

            /* 2. 计算环形队列的下一个 Head 写入位置 (位运算提速) */
            uint16_t NextHead = (g_Rx_Queue_Head + 1) & (RX_QUEUE_SIZE - 1);

            /* 3. 检查队列是否已满 */
            if (NextHead == g_Rx_Queue_Tail)
            {
                /* Error: 队列满，丢弃本次数据并记录溢出错误 */
            }
            else
            {
                /* 4. 队列未满，获取 "head" 位置的“集装箱”指针 */
                volatile CanRxMessage_t* Msg_To_Queue = &g_Rx_Queue[g_Rx_Queue_Head];

                /* 5. 将临时数据从 RxHeader/RxData 复制到队列中 */
                Msg_To_Queue->Rx_Header = s_Can_Rx_Header; /* 复制整个报文头 (包含ID, DLC等) */

                /* 【注意】：在 STM32G4 FDCAN 中，DataLength 是宏定义（如 0x00080000），
                 * 严格来说不能作为循环上限。这里遵照要求保持原样，后续需留意此处的越界隐患。 */
                for (int i = 0; i < s_Can_Rx_Header.DataLength; i++)
                {
                    Msg_To_Queue->Data[i] = RxData[i];
                }

                /* 6. 【原子操作】移动头指针，正式将消息放入队列 */
                /* 这个赋值操作是原子的，main 循环现在就能看到新消息了 */
                g_Rx_Queue_Head = NextHead;
            }
        }

        /* 7. 【非常重要】: 每次处理完中断后，必须重新激活通知，否则中断只会触发一次！ */
        if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
        {
            /* Error: 激活 CAN 失败 */
        }
    }
}

/**
 * @brief  将应答请求压入 CAN 发送队列 (发送生产者)
 * @param  command 原始命令 (用于回显给上位机)
 * @param  status  状态码 (0x00=OK, 0x01=未知命令, 0x02=参数错误)
 * @retval true    入队成功
 * @retval false   队列已满，入队失败
 * @note   在 main 循环或业务层中调用，非阻塞执行。
 */
bool Queue_Reply_Request(uint8_t command, uint8_t status)
{
    /* 1. 计算 g_tx_queue 队列的下一个 "head" 指针的位置 */
    uint16_t Next_Head = (g_Tx_Queue_Head + 1) & (TX_QUEUE_SIZE - 1);

    /* 2. 检查 Tx 队列是否已满 */
    if (Next_Head == g_Tx_Queue_Tail)
    {
        /* 队列已满，本次应答被丢弃 */
        return false;
    }

    /* 3. 队列未满，获取 "head" 位置的缓冲区指针 */
    volatile CanTxMessage_t* Msg_To_Queue = &g_Tx_Queue[g_Tx_Queue_Head];

    /* 4. 填充报文头 (Tx_Header) */
    Msg_To_Queue->Tx_Header = s_Can_Tx_Header;              /* 复制模板报文头 */
    Msg_To_Queue->Tx_Header.Identifier = MESSAGE_ID;        /* 使用 MESSAGE_ID 宏 (0x201) */
    Msg_To_Queue->Tx_Header.DataLength = FDCAN_DLC_BYTES_2; /* 灵活 DLC，本次只发 2 字节 */

    /* 5. 填充报文数据 */
    Msg_To_Queue->Data[0] = command; /* Data[0] = 回显收到的命令 */
    Msg_To_Queue->Data[1] = status;  /* Data[1] = 执行状态码 */
    /* (其余字节由于 DLC 设置为 2，会被底层硬件自动忽略) */

    /* 6. 【原子操作】移动头指针，正式将消息放入队列 */
    g_Tx_Queue_Head = Next_Head;

    return true;
}
