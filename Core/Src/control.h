/*
 * control.h
 *
 *  Created on: Sep 2, 2025
 *      Author: 16964
 */

#ifndef INC_CONTROL_H_  // 使用项目约定的保护符格式
#define INC_CONTROL_H_
#include "main.h"


void DC_Power_CTR(bool state);
void set_speed(float speed_hz);
void move_to_position(int32_t target_steps, float speed_hz);
void move_repeatedly(int32_t pos1, int32_t pos2, uint32_t count, float speed_hz);
void start_move_to_position(int32_t target_steps, float speed_hz);
void step_move(int32_t relative_steps, float speed_hz);
void Update_output(uint8_t new_step);
void Close_output(void);
#endif /* SRC_CONTROL_H_ */
