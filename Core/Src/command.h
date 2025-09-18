/*
 * command.h
 *
 *  Created on: Sep 2, 2025
 *      Author: 16964
 */

#ifndef INC_COMMAND_H_
#define INC_COMMAND_H_

#include "main.h"
#include "control.h"

void process_received_data(uint8_t* data, uint16_t size);
void send_float_array_dma(float* arr, int count);
#endif /* SRC_COMMAND_H_ */
