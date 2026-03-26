#ifndef _APP_DISPLAY_H
#define _APP_DISPLAY_H

#include "stdint.h"

/**
 * @brief  UI 静态框架 (5框模块化, 横屏 280x240) - 仅开机调用一次
 * @note   利用定点像素操作手动绘制 5 个“伪圆角”矩形框。
 */
void App_Display_Init(void);

/**
 * @brief  【辅助函数】画一个“伪圆角”矩形
 * @note   如果你的屏幕驱动库提供了画圆的 API，可以用组合圆和线来做真正的圆角。
 * 这里我们提供一个基于标准画点/画线操作的逻辑切角方案。
 */
void LCD_DrawRoundedRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,uint16_t r, uint16_t color);
/**
 * @brief  UI 动态刷新 (放在 main 循环中, 建议每 100ms 调用一次)
 * @note   绝对坐标对齐，利用背景色 bc 直接覆盖旧字符，不闪屏。
 */
void App_Display_Update(void);





#endif


