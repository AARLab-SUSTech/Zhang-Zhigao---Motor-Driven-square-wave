/*
 * can.h
 *
 *  Created on: Oct 11, 2025
 *      Author: 16964
 */

#ifndef CAN_H_
#define CAN_H_

#include "main.h"

#define MOTOR_ID        0x04

#define BROADCAST_ID    0x100  // <-- 我们协议中定义的广播ID
#define MY_NODE_ID      0x100 + MOTOR_ID  // <-- 设置本节点的实际ID
#define MESSAGE_ID      0x200 + MOTOR_ID  // <-- 回信ID
#define HEART_ID        0x300 + MOTOR_ID  // <-- 心跳ID

extern FDCAN_TxHeaderTypeDef TxHeader;
extern FDCAN_RxHeaderTypeDef RxHeader; // 用于存储接收报文的头信息


/**
  * @brief  定义一个CAN发送消息的结构体
  * 它将报文头(Header)和数据(Data)捆绑在一起
  */
typedef struct{
	FDCAN_TxHeaderTypeDef Tx_Header; // 包含了ID, DLC (数据长度) 等所有信息
	uint8_t Data[8];
}CanTxMessage_t;
typedef struct{
	FDCAN_RxHeaderTypeDef Rx_Header; // 包含了ID, DLC (数据长度) 等所有信息
	uint8_t Data[8];
}CanRxMessage_t;
/**
  * @brief  定义队列的大小 (环形缓冲区)
  * 必须是 2 的N次方 (例如 2, 4, 8, 16, 32)，以便使用高效的位运算
  * 这里我们选择 16，意味着队列最多可以缓存 16 条待发送的消息
  */
#define TX_QUEUE_SIZE 16
#define RX_QUEUE_SIZE 16
/**
  * @brief  【外部声明】队列的存储数组
  * "extern" 关键字告诉编译器：这个变量是真实存在的，
  * 但它定义在“别处”(另一个.c文件)，请你先别报错。
  */
extern volatile CanTxMessage_t g_tx_queue[TX_QUEUE_SIZE];
extern volatile CanRxMessage_t g_rx_queue[RX_QUEUE_SIZE];
/**
  * @brief  【外部声明】队列的 "头指针" (Head)
  */
extern volatile uint16_t g_tx_queue_head;
extern volatile uint16_t g_rx_queue_head;
/**
  * @brief  【外部声明】队列的 "尾指针" (Tail)
  */
extern volatile uint16_t g_tx_queue_tail;
extern volatile uint16_t g_rx_queue_tail;
extern uint8_t TxData[8];

void CAN_init(void);
void Can_message_process(void);
bool Queue_Reply_Request(uint8_t command, uint8_t status);
#endif
