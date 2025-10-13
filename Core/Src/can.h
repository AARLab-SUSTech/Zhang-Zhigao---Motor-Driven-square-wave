/*
 * can.h
 *
 *  Created on: Oct 11, 2025
 *      Author: 16964
 */

#ifndef CAN_H_
#define CAN_H_

#define MY_NODE_ID      0x101  // <-- 【重要】在这里设置本节点的实际ID，例如5号节点
#define BROADCAST_ID    0x100  // <-- 我们协议中定义的广播ID

void CAN_init(void);

#endif
