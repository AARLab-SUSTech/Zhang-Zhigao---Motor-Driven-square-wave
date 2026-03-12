/*
 * command.h
 *
 *  Created on: Sep 2, 2025
 *      Author: 16964
 */

#ifndef _MID_COMMAND_USART_H
#define _MID_COMMAND_USART_H

#include "main.h"

extern float temp_speed_float;
extern int16_t temp_speed_int16;

void Mid_Process_Usart_Data(uint8_t* data, uint16_t size);
void Mid_Float_Array_Dma_Send(float* arr, int count);
void Mid_Int16_Groups_Dma_Send(int16_t* arr, int total_count, int elements_per_group);
#endif /* SRC_COMMAND_H_ */
