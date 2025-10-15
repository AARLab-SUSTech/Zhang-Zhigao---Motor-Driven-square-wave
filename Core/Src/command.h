/*
 * command.h
 *
 *  Created on: Sep 2, 2025
 *      Author: 16964
 */

#ifndef COMMAND_H_
#define COMMAND_H_

#include "main.h"

void process_received_data(uint8_t* data, uint16_t size);
void send_float_array_dma(float* arr, int count);
void send_int16_groups_dma(int16_t* arr, int total_count, int elements_per_group);
#endif /* SRC_COMMAND_H_ */
