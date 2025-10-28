/*
 * can.h
 *
 *  Created on: Oct 11, 2025
 *      Author: 16964
 */

#ifndef CAN_H_
#define CAN_H_

#define MY_NODE_ID      0x101  // <-- 设置本节点的实际ID
#define BROADCAST_ID    0x100  // <-- 我们协议中定义的广播ID

void CAN_init(void);

#endif
