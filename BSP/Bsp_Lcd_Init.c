#include <Bsp_Lcd_Init.h>
#include "stm32g4xx.h"

/******************************************************************************
      函数说明：LCD串行数据写入函数
      入口数据：dat  要写入的串行数据
      返回值：  无
******************************************************************************/
//void LCD_Writ_Bus(uint8_t dat)
//{
//	uint8_t i;
//	LCD_CS_Clr();
//	for(i=0;i<8;i++)
//	{
//		LCD_SCLK_Clr();
//		if(dat&0x80)
//		{
//		   LCD_MOSI_Set();
//		}
//		else
//		{
//		   LCD_MOSI_Clr();
//		}
//		LCD_SCLK_Set();
//		dat<<=1;
//	}
//  LCD_CS_Set();
//}
extern SPI_HandleTypeDef hspi1;
/******************************************************************************
      函数说明：LCD串行数据写入函数 (硬件SPI版)
      入口数据：dat  要写入的串行数据
      返回值：  无
******************************************************************************/
void LCD_Writ_Bus(uint8_t dat)
{
	LCD_CS_Clr(); // 拉低片选，选中屏幕

	// 使用 HAL 库的轮询发送函数
	// &dat: 数据的地址
	// 1: 发送 1 个字节
	// 10: 超时时间 10ms
	HAL_SPI_Transmit(&hspi1, &dat, 1, 100);

	LCD_CS_Set(); // 拉高片选，取消选中
}

/******************************************************************************
      函数说明：LCD写入数据
      入口数据：dat 写入的数据
      返回值：  无
******************************************************************************/
void LCD_WR_DATA8(uint8_t dat)
{
	LCD_Writ_Bus(dat);
}


/******************************************************************************
      函数说明：LCD写入数据
      入口数据：dat 写入的数据
      返回值：  无
******************************************************************************/
//void LCD_WR_DATA(uint16_t dat)
//{
//	LCD_Writ_Bus(dat>>8);
//	LCD_Writ_Bus(dat);
//}
/******************************************************************************
      函数说明：LCD写入16位颜色数据 (极速画图专用)
      核心优化：16位数据只拉低一次片选，直接连发两字节！
******************************************************************************/
void LCD_WR_DATA(uint16_t dat){

    LCD_CS_Clr(); // 拉低片选，准备连续发送

    /* 发送高8位 */
    while((SPI1->SR & SPI_SR_TXE) == 0);
    *((__IO uint8_t *)&SPI1->DR) = (dat >> 8);
    /* 发送低8位 (不需要拉高片选，直接连发) */

    while((SPI1->SR & SPI_SR_TXE) == 0);
    *((__IO uint8_t *)&SPI1->DR) = dat;
    /* 等待最后的数据从移位寄存器发完 */

    while((SPI1->SR & SPI_SR_BSY) != 0);

    LCD_CS_Set(); // 传输完毕，拉高片选

}
//void LCD_WR_DATA(uint16_t dat)
//{
//    /* 1. 将 16 位数据拆分成 2 个 8 位字节的数组 (高位在前) */
//    uint8_t tx_buf[2];
//    tx_buf[0] = (uint8_t)(dat >> 8);   // 高 8 位
//    tx_buf[1] = (uint8_t)(dat & 0xFF); // 低 8 位
//
//    /* 2. 拉低片选，选中屏幕 */
//    LCD_CS_Clr();
//
//    /* 3. 使用 HAL 库一次性连续发送 2 个字节
//     * 参数1: SPI句柄
//     * 参数2: 数据数组的首地址
//     * 参数3: 要发送的字节数 (2个)
//     * 参数4: 超时时间 (设为 10ms 足够了)
//     */
//    HAL_SPI_Transmit(&hspi1, tx_buf, 2, 10);
//
//    /* 4. 传输完毕，拉高片选 */
//    LCD_CS_Set();
//}


/******************************************************************************
      函数说明：LCD写入命令
      入口数据：dat 写入的命令
      返回值：  无
******************************************************************************/
void LCD_WR_REG(uint8_t dat)
{
	LCD_DC_Clr();//写命令
	LCD_Writ_Bus(dat);
	LCD_DC_Set();//写数据
}


/******************************************************************************
      函数说明：设置起始和结束地址
      入口数据：x1,x2 设置列的起始和结束地址
                y1,y2 设置行的起始和结束地址
      返回值：  无
******************************************************************************/
void LCD_Address_Set(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)
{
	if(USE_HORIZONTAL==0)
	{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(x1);
		LCD_WR_DATA(x2);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(y1+20);
		LCD_WR_DATA(y2+20);
		LCD_WR_REG(0x2c);//储存器写
	}
	else if(USE_HORIZONTAL==1)
	{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(x1);
		LCD_WR_DATA(x2);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(y1+20);
		LCD_WR_DATA(y2+20);
		LCD_WR_REG(0x2c);//储存器写
	}
	else if(USE_HORIZONTAL==2)
	{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(x1+20);
		LCD_WR_DATA(x2+20);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(y1);
		LCD_WR_DATA(y2);
		LCD_WR_REG(0x2c);//储存器写
	}
	else
	{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(x1+20);
		LCD_WR_DATA(x2+20);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(y1);
		LCD_WR_DATA(y2);
		LCD_WR_REG(0x2c);//储存器写
	}
}

void LCD_Init(void)
{

	//LCD_BLK_Set();//打开背光
	HAL_Delay(100);
	
	//************* Start Initial Sequence **********//
	LCD_WR_REG(0x11); //Sleep out 
	HAL_Delay(120);              //Delay 120ms
	//************* Start Initial Sequence **********// 
	LCD_WR_REG(0x36);
	if(USE_HORIZONTAL==0)LCD_WR_DATA8(0x00);
	else if(USE_HORIZONTAL==1)LCD_WR_DATA8(0xC0);
	else if(USE_HORIZONTAL==2)LCD_WR_DATA8(0x70);
	else LCD_WR_DATA8(0xA0);

	LCD_WR_REG(0x3A);			
	LCD_WR_DATA8(0x05);

	LCD_WR_REG(0xB2);			
	LCD_WR_DATA8(0x0C);
	LCD_WR_DATA8(0x0C); 
	LCD_WR_DATA8(0x00); 
	LCD_WR_DATA8(0x33); 
	LCD_WR_DATA8(0x33); 			

	LCD_WR_REG(0xB7);			
	LCD_WR_DATA8(0x35);

	LCD_WR_REG(0xBB);			
	LCD_WR_DATA8(0x32); //Vcom=1.35V
					
	LCD_WR_REG(0xC2);
	LCD_WR_DATA8(0x01);

	LCD_WR_REG(0xC3);			
	LCD_WR_DATA8(0x15); //GVDD=4.8V  颜色深度
				
	LCD_WR_REG(0xC4);			
	LCD_WR_DATA8(0x20); //VDV, 0x20:0v

	LCD_WR_REG(0xC6);			
	LCD_WR_DATA8(0x0F); //0x0F:60Hz        	

	LCD_WR_REG(0xD0);			
	LCD_WR_DATA8(0xA4);
	LCD_WR_DATA8(0xA1); 

	LCD_WR_REG(0xE0);
	LCD_WR_DATA8(0xD0);   
	LCD_WR_DATA8(0x08);   
	LCD_WR_DATA8(0x0E);   
	LCD_WR_DATA8(0x09);   
	LCD_WR_DATA8(0x09);   
	LCD_WR_DATA8(0x05);   
	LCD_WR_DATA8(0x31);   
	LCD_WR_DATA8(0x33);   
	LCD_WR_DATA8(0x48);   
	LCD_WR_DATA8(0x17);   
	LCD_WR_DATA8(0x14);   
	LCD_WR_DATA8(0x15);   
	LCD_WR_DATA8(0x31);   
	LCD_WR_DATA8(0x34);   

	LCD_WR_REG(0xE1);     
	LCD_WR_DATA8(0xD0);   
	LCD_WR_DATA8(0x08);   
	LCD_WR_DATA8(0x0E);   
	LCD_WR_DATA8(0x09);   
	LCD_WR_DATA8(0x09);   
	LCD_WR_DATA8(0x15);   
	LCD_WR_DATA8(0x31);   
	LCD_WR_DATA8(0x33);   
	LCD_WR_DATA8(0x48);   
	LCD_WR_DATA8(0x17);   
	LCD_WR_DATA8(0x14);   
	LCD_WR_DATA8(0x15);   
	LCD_WR_DATA8(0x31);   
	LCD_WR_DATA8(0x34);
	LCD_WR_REG(0x21); 

	LCD_WR_REG(0x29);
} 








