/**
 ******************************************************************************
 * @file    Bsp_Can.h
 * @brief   CAN (FDCAN) 底层硬件驱动与队列管理模块头文件
 * @note    提供 CAN 节点 ID 宏定义、收发消息结构体、环形队列的外部声明
 * 以及初始化和发送请求的函数接口。
 ******************************************************************************
 */

#ifndef _BSP_CAN_H_
#define _BSP_CAN_H_

#include "main.h"

/* ==========================================
 * 节点 ID 与通信宏定义 (Node IDs & Macros)
 * ========================================== */
#define MOTOR_ID        0x03

#define BROADCAST_ID    0x100                 /* <-- 我们协议中定义的广播ID */

/* 【安全规范】：带有算术运算的宏定义，最外层必须加括号，防止展开时优先级错乱 */
#define MY_NODE_ID      (0x100 + MOTOR_ID)    /* <-- 设置本节点的实际ID */
#define MESSAGE_ID      (0x200 + MOTOR_ID)    /* <-- 回信ID */
#define HEART_ID        (0x300 + MOTOR_ID)    /* <-- 心跳ID */

/* ==========================================
 * 环形队列大小定义 (Queue Size)
 * ========================================== */
/**
 * @brief 定义队列的大小 (环形缓冲区)
 * @note  必须是 2 的 N 次方 (例如 2, 4, 8, 16, 32)，以便使用高效的位运算 (&)。
 * 这里选择 16，意味着队列最多可以缓存 16 条待处理的消息。
 */
#define TX_QUEUE_SIZE   16
#define RX_QUEUE_SIZE   16

/* ==========================================
 * 数据结构定义 (Data Structures)
 * ========================================== */

/**
 * @brief  定义一个 CAN 发送消息的结构体
 * @note   它将报文头 (Header) 和数据 (Data) 捆绑在一起，作为队列的元素
 */
typedef struct {
    FDCAN_TxHeaderTypeDef Tx_Header; /* 包含了 ID, DLC (数据长度) 等所有发送信息 */
    uint8_t               Data[8];   /* 报文数据 payload */
} CanTxMessage_t;

/**
 * @brief  定义一个 CAN 接收消息的结构体
 * @note   它将报文头 (Header) 和数据 (Data) 捆绑在一起，作为队列的元素
 */
typedef struct {
    FDCAN_RxHeaderTypeDef Rx_Header; /* 包含了 ID, DLC (数据长度) 等所有接收信息 */
    uint8_t               Data[8];   /* 报文数据 payload */
} CanRxMessage_t;

/* ==========================================
 * 外部全局变量声明 (Extern Variables)
 * ========================================== */

extern FDCAN_TxHeaderTypeDef s_Can_Tx_Header;
extern FDCAN_RxHeaderTypeDef s_Can_Rx_Header; /* 用于存储接收报文的头信息 */
extern uint8_t               TxData[8];

/**
 * @brief  【外部声明】队列的存储数组
 * @note   "extern" 关键字告诉编译器：这个变量是真实存在的，
 * 但它定义在“别处” (另一个 .c 文件)，请在此处引用它。
 */
extern volatile CanTxMessage_t g_Tx_Queue[TX_QUEUE_SIZE];
extern volatile CanRxMessage_t g_Rx_Queue[RX_QUEUE_SIZE];

/**
 * @brief  【外部声明】队列的 "头指针" (Head)
 * @note   'head' 是数据“放入”的位置。
 */
extern volatile uint16_t g_Tx_Queue_Head;
extern volatile uint16_t g_Rx_Queue_Head;

/**
 * @brief  【外部声明】队列的 "尾指针" (Tail)
 * @note   'tail' 是数据“取出”的位置。
 */
extern volatile uint16_t g_Tx_Queue_Tail;
extern volatile uint16_t g_Rx_Queue_Tail;

/* ==========================================
 * 函数接口声明 (Function Prototypes)
 * ========================================== */

/**
 * @brief  FDCAN 硬件初始化
 * @retval 0: 成功, -1: 失败
 */
int8_t Bsp_Can_Init(void);

/**
 * @brief  将应答请求压入 CAN 发送队列 (发送生产者)
 * @param  command 原始命令 (用于回显给上位机)
 * @param  status  状态码 (0x00=OK, 0x01=未知命令, 0x02=参数错误)
 * @retval true    入队成功
 * @retval false   队列已满，入队失败
 */
bool Queue_Reply_Request(uint8_t command, uint8_t status);

/**
 * @brief  尝试将一条报文压入 CAN 硬件发送 FIFO (纯底层逻辑)
 * @param  msg 指向待发送报文结构体的指针
 * @retval bool 发送结果 (true: 成功压入硬件, false: 硬件 FIFO 满或错误)
 */
bool Bsp_Can_Transmit_Message(CanTxMessage_t *msg);

#endif /* _BSP_CAN_H_ */
