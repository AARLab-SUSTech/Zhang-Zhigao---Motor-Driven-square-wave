/**
 ******************************************************************************
 * @file    App_Can.c
 * @author  letian
 * @date    Mar 16, 2026
 * @brief   CAN 顶层应用通信模块 (APP)
 * @note    负责系统心跳包等顶层 CAN 业务逻辑的组装与发送调度。
 ******************************************************************************
 */

#include <App_EFA.h>
#include "App_Can.h"
#include "Bsp_Can.h"
#include <stdint.h> /* 系统标准库建议使用尖括号 */

/* ==========================================
 * 外部依赖声明 (Extern Declarations)
 * ========================================== */
extern Motor EFA_DATA;

/* ==========================================
 * 函数实现 (Function Implementations)
 * ========================================== */

/**
 * @brief  发送 CAN 心跳包 (Heartbeat)
 * @note   将包含当前电机状态机模式的心跳数据，压入 CAN 发送环形队列。
 * 在定时器任务或主循环中周期性调用。
 */
void App_Can_Heart_Send(void)
{
    /* 1. 计算下一个 "head" 指针的位置
     * (使用位运算 & (TX_QUEUE_SIZE - 1) 比 % 更高效, 因为 TX_QUEUE_SIZE 是 2 的 N 次方)
     */
    uint16_t Next_Head = (g_Tx_Queue_Head + 1) & (TX_QUEUE_SIZE - 1);

    /* 2. 检查队列是否已满 (如果 head 的下一个位置就是 tail，说明满载) */
    if (Next_Head == g_Tx_Queue_Tail)
    {
        /* 队列已满，本次心跳报文直接丢弃，不阻塞主线程 */
    }
    else
    {
        /* 3. 队列未满，获取 "head" 位置的“集装箱”指针
         * (注意：我们总是在 g_Tx_Queue_Head 指向的位置填充数据)
         */
        volatile CanTxMessage_t* msg_to_queue = &g_Tx_Queue[g_Tx_Queue_Head];

        /* 4. 填充报文头 (复制底层初始化好的模板，再修改特定参数) */
        msg_to_queue->Tx_Header            = s_Can_Tx_Header;       /* 复制模板 */
        msg_to_queue->Tx_Header.Identifier = HEART_ID;              /* 【心跳 ID】 */
        msg_to_queue->Tx_Header.DataLength = FDCAN_DLC_BYTES_1;     /* 【灵活 DLC】只需发送 1 字节 */

        /* 5. 填充报文数据 (Payload) */
        msg_to_queue->Data[0] = (uint8_t)EFA_DATA.Motor_mode;       /* 填入当前模式/故障码 */

        /* 清空其余数据位 (良好习惯：防止发送未知的内存残留脏数据) */
        for (int i = 1; i < 8; i++)
        {
            msg_to_queue->Data[i] = 0x00;
        }

        /* 6. 【原子操作】移动头指针，正式将消息放入队列供底层 DMA 发送 */
        g_Tx_Queue_Head = Next_Head;
    }
}

/**
 * @brief  发送 CAN 遥测数据包 (Telemetry Data)
 * @note   将往复运动次数、高压母线电压、电流等运行数据压入发送队列。
 * * 数据帧格式 (8 字节，全大端模式 Big-Endian):
 * [0..3] : 往复次数 (repeated_count_current, 32-bit)
 * [4..5] : 高压电压 (HV_V * 10, 16-bit)
 * [6..7] : 高压电流 (HV_I * 100, 16-bit)
 */
void App_Can_Send_Data(void)
{
    /* 1. 计算下一个 "head" 指针的位置
     * (使用位运算 & (TX_QUEUE_SIZE - 1) 比 % 更高效, 因为 TX_QUEUE_SIZE 是 2 的 N 次方)
     */
    uint16_t NextHead = (g_Tx_Queue_Head + 1) & (TX_QUEUE_SIZE - 1);
    uint16_t TempData;

    /* 2. 检查队列是否已满 (如果 head 的下一个位置就是 tail) */
    if (NextHead == g_Tx_Queue_Tail)
    {
        /* 队列已满，本次遥测数据包直接丢弃，不阻塞主流程 */
    }
    else
    {
        /* 3. 队列未满，获取 "head" 位置的“集装箱”指针 */
        volatile CanTxMessage_t* MsgToQueue = &g_Tx_Queue[g_Tx_Queue_Head];

        /* 4. 填充报文头 (复制模板，再修改特定部分) */
        MsgToQueue->Tx_Header            = s_Can_Tx_Header;       /* 复制模板 */
        MsgToQueue->Tx_Header.Identifier = MESSAGE_ID;            /* 【数据回传 ID】 */
        MsgToQueue->Tx_Header.DataLength = FDCAN_DLC_BYTES_8;     /* 满载 8 字节 */

        /* 5. 填充报文数据 (Payload) */

        /* Data[0..3]: 往复次数 (32-bit, 大端模式拆分) */
        MsgToQueue->Data[0] = (uint8_t)((repeated_count_current >> 24) & 0xFF);
        MsgToQueue->Data[1] = (uint8_t)((repeated_count_current >> 16) & 0xFF);
        MsgToQueue->Data[2] = (uint8_t)((repeated_count_current >> 8) & 0xFF);
        MsgToQueue->Data[3] = (uint8_t)(repeated_count_current & 0xFF);

        /* Data[4..5]: 高压母线电压 (放大 10 倍保留 1 位小数, 16-bit, 大端模式) */
        TempData = (uint16_t)(EFA_DATA.Hv_V_V * 10);
        MsgToQueue->Data[4] = (uint8_t)(TempData >> 8);
        MsgToQueue->Data[5] = (uint8_t)(TempData & 0xFF);

        /* Data[6..7]: 高压母线电流 (放大 100 倍保留 2 位小数, 16-bit, 大端模式) */
        TempData = (uint16_t)(EFA_DATA.Hv_I_mA * 100);
        MsgToQueue->Data[6] = (uint8_t)(TempData >> 8);
        MsgToQueue->Data[7] = (uint8_t)(TempData & 0xFF);

        /* 【安全修复】：移除了原来错误清空 Data[6] 和 Data[7] 的 for 循环 */

        /* 6. 【原子操作】移动头指针，正式将消息放入队列供底层 DMA 发送 */
        g_Tx_Queue_Head = NextHead;
    }
}



