#include "Bsp_Led.h"
#include "stm32g4xx_hal.h"


void Bsp_Led_On(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
}

void Bsp_Led_Off(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
}



