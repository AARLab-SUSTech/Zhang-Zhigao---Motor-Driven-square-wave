/**
  ******************************************************************************
  * @file    Bsp_Ad7190.h
  * @author  硬石嵌入式开发团队 (经排版规范化整理)
  * @version V1.0
  * @date    2017-03-30
  * @brief   板载串行 Flash / AD7190 称重模块底层驱动头文件
  ******************************************************************************
  * @note    说明：
  * 本例程配套硬石 stm32 开发板 YS-F4Pro 使用。
  * 淘宝：
  * 论坛：http://www.ing10bbs.com
  * 版权归硬石嵌入式开发团队所有，请勿商用。
  ******************************************************************************
  */

#ifndef __BSP_AD7190_H__
#define __BSP_AD7190_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* ==========================================
 * 数据结构定义 (Data Structures)
 * ========================================== */

/**
 * @brief  力传感器 (称重) 数据结构体
 */
typedef struct
{
    int32_t weight_Zero_Data;   /* 去皮零点数据 */
    int32_t weight_proportion;  /* 重量比例系数 */
    int32_t calibrating_weight; /* 标定重量 */
    int32_t RAW_Data;           /* ADC 原始数据 */
    float   weight_g;           /* 计算得出的重量 (克) */
    float   weight_N;           /* 计算得出的力 (牛顿) */
} Force_sensor;

/* ==========================================
 * 硬件引脚与外设宏定义 (Hardware Macros)
 * ========================================== */
#define WEIGHT_SPIx                 SPI1
#define WEIGHT_SPIx_CLK_ENABLE()    __HAL_RCC_SPI1_CLK_ENABLE()
#define WEIGHT_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOB_CLK_ENABLE()

#define WEIGHT_CS_Pin               GPIO_PIN_10
#define WEIGHT_CS_GPIO_Port         GPIOB
#define WEIGHT_SCK_Pin              GPIO_PIN_3
#define WEIGHT_SCK_GPIO_Port        GPIOB
#define WEIGHT_MISO_Pin             GPIO_PIN_4
#define WEIGHT_MISO_GPIO_Port       GPIOB
#define WEIGHT_MOSI_Pin             GPIO_PIN_5
#define WEIGHT_MOSI_GPIO_Port       GPIOB

#define WEIGHT_SPI_AF               GPIO_AF5_SPI1

#define WEIGHT_CS_ENABLE()          HAL_GPIO_WritePin(WEIGHT_CS_GPIO_Port, WEIGHT_CS_Pin, GPIO_PIN_RESET)
#define WEIGHT_CS_DISABLE()         HAL_GPIO_WritePin(WEIGHT_CS_GPIO_Port, WEIGHT_CS_Pin, GPIO_PIN_SET)

/* AD7190 GPIO 状态读取 */
#define AD7190_RDY_STATE            (WEIGHT_MISO_GPIO_Port->IDR & WEIGHT_MISO_Pin)

/* ==========================================
 * AD7190 寄存器映射 (Register Map)
 * ========================================== */
#define AD7190_REG_COMM             0 /* Communications Register (WO, 8-bit) */
#define AD7190_REG_STAT             0 /* Status Register         (RO, 8-bit) */
#define AD7190_REG_MODE             1 /* Mode Register           (RW, 24-bit) */
#define AD7190_REG_CONF             2 /* Configuration Register  (RW, 24-bit) */
#define AD7190_REG_DATA             3 /* Data Register           (RO, 24/32-bit) */
#define AD7190_REG_ID               4 /* ID Register             (RO, 8-bit) */
#define AD7190_REG_GPOCON           5 /* GPOCON Register         (RW, 8-bit) */
#define AD7190_REG_OFFSET           6 /* Offset Register         (RW, 24-bit) */
#define AD7190_REG_FULLSCALE        7 /* Full-Scale Register     (RW, 24-bit) */

/* Communications Register Bit Designations (AD7190_REG_COMM) */
#define AD7190_COMM_WEN             (1 << 7)           /* Write Enable. */
#define AD7190_COMM_WRITE           (0 << 6)           /* Write Operation. */
#define AD7190_COMM_READ            (1 << 6)           /* Read Operation. */
#define AD7190_COMM_ADDR(x)         (((x) & 0x7) << 3) /* Register Address. */
#define AD7190_COMM_CREAD           (1 << 2)           /* Continuous Read of Data Register. */

/* Status Register Bit Designations (AD7190_REG_STAT) */
#define AD7190_STAT_RDY             (1 << 7) /* Ready. */
#define AD7190_STAT_ERR             (1 << 6) /* ADC error bit. */
#define AD7190_STAT_NOREF           (1 << 5) /* Error no external reference. */
#define AD7190_STAT_PARITY          (1 << 4) /* Parity check of the data register. */
#define AD7190_STAT_CH2             (1 << 2) /* Channel 2. */
#define AD7190_STAT_CH1             (1 << 1) /* Channel 1. */
#define AD7190_STAT_CH0             (1 << 0) /* Channel 0. */

/* Mode Register Bit Designations (AD7190_REG_MODE) */
#define AD7190_MODE_SEL(x)          (((x) & 0x7) << 21) /* Operation Mode Select. */
#define AD7190_MODE_DAT_STA         (1 << 20)           /* Status Register transmission. */
#define AD7190_MODE_CLKSRC(x)       (((x) & 0x3) << 18) /* Clock Source Select. */
#define AD7190_MODE_SINC3           (1 << 15)           /* SINC3 Filter Select. */
#define AD7190_MODE_ENPAR           (1 << 13)           /* Parity Enable. */
#define AD7190_MODE_SCYCLE          (1 << 11)           /* Single cycle conversion. */
#define AD7190_MODE_REJ60           (1 << 10)           /* 50/60Hz notch filter. */
#define AD7190_MODE_RATE(x)         ((x) & 0x3FF)       /* Filter Update Rate Select. */

/* Mode Register: AD7190_MODE_SEL(x) options */
#define AD7190_MODE_CONT            0 /* Continuous Conversion Mode. */
#define AD7190_MODE_SINGLE          1 /* Single Conversion Mode. */
#define AD7190_MODE_IDLE            2 /* Idle Mode. */
#define AD7190_MODE_PWRDN           3 /* Power-Down Mode. */
#define AD7190_MODE_CAL_INT_ZERO    4 /* Internal Zero-Scale Calibration. */
#define AD7190_MODE_CAL_INT_FULL    5 /* Internal Full-Scale Calibration. */
#define AD7190_MODE_CAL_SYS_ZERO    6 /* System Zero-Scale Calibration. */
#define AD7190_MODE_CAL_SYS_FULL    7 /* System Full-Scale Calibration. */

/* Mode Register: AD7190_MODE_CLKSRC(x) options */
#define AD7190_CLK_EXT_MCLK1_2      0 /* External crystal connected from MCLK1 to MCLK2. */
#define AD7190_CLK_EXT_MCLK2        1 /* External Clock applied to MCLK2. */
#define AD7190_CLK_INT              2 /* Internal 4.92 MHz clock. Pin MCLK2 is tristated. */
#define AD7190_CLK_INT_CO           3 /* Internal 4.92 MHz clock. Available on MCLK2. */

/* Configuration Register Bit Designations (AD7190_REG_CONF) */
#define AD7190_CONF_CHOP            (1 << 23)           /* CHOP enable. */
#define AD7190_CONF_REFSEL          (1 << 20)           /* REFIN1/REFIN2 Reference Select. */
#define AD7190_CONF_CHAN(x)         (((x) & 0xFF) << 8) /* Channel select. */
#define AD7190_CONF_BURN            (1 << 7)            /* Burnout current enable. */
#define AD7190_CONF_REFDET          (1 << 6)            /* Reference detect enable. */
#define AD7190_CONF_BUF             (1 << 4)            /* Buffered Mode Enable. */
#define AD7190_CONF_UNIPOLAR        (1 << 3)            /* Unipolar/Bipolar Enable. */
#define AD7190_CONF_GAIN(x)         ((x) & 0x7)         /* Gain Select. */

/* Configuration Register: AD7190_CONF_CHAN(x) options */
#define AD7190_CH_AIN1P_AIN2M       0 /* AIN1(+) - AIN2(-) */
#define AD7190_CH_AIN3P_AIN4M       1 /* AIN3(+) - AIN4(-) */
#define AD7190_CH_TEMP_SENSOR       2 /* Temperature sensor */
#define AD7190_CH_AIN2P_AIN2M       3 /* AIN2(+) - AIN2(-) */
#define AD7190_CH_AIN1P_AINCOM      4 /* AIN1(+) - AINCOM */
#define AD7190_CH_AIN2P_AINCOM      5 /* AIN2(+) - AINCOM */
#define AD7190_CH_AIN3P_AINCOM      6 /* AIN3(+) - AINCOM */
#define AD7190_CH_AIN4P_AINCOM      7 /* AIN4(+) - AINCOM */

/* Configuration Register: AD7190_CONF_GAIN(x) options */
/* ADC Input Range (5 V Reference) */
#define AD7190_CONF_GAIN_1          0 /* Gain 1    +-5 V */
#define AD7190_CONF_GAIN_8          3 /* Gain 8    +-625 mV */
#define AD7190_CONF_GAIN_16         4 /* Gain 16   +-312.5 mV */
#define AD7190_CONF_GAIN_32         5 /* Gain 32   +-156.2 mV */
#define AD7190_CONF_GAIN_64         6 /* Gain 64   +-78.125 mV */
#define AD7190_CONF_GAIN_128        7 /* Gain 128  +-39.06 mV */

/* ID Register Bit Designations (AD7190_REG_ID) */
#define ID_AD7190                   0x4
#define AD7190_ID_MASK              0x0F

/* GPOCON Register Bit Designations (AD7190_REG_GPOCON) */
#define AD7190_GPOCON_BPDSW         (1 << 6) /* Bridge power-down switch enable */
#define AD7190_GPOCON_GP32EN        (1 << 5) /* Digital Output P3 and P2 enable */
#define AD7190_GPOCON_GP10EN        (1 << 4) /* Digital Output P1 and P0 enable */
#define AD7190_GPOCON_P3DAT         (1 << 3) /* P3 state */
#define AD7190_GPOCON_P2DAT         (1 << 2) /* P2 state */
#define AD7190_GPOCON_P1DAT         (1 << 1) /* P1 state */
#define AD7190_GPOCON_P0DAT         (1 << 0) /* P0 state */

/* ==========================================
 * 外部全局变量声明 (Extern Variables)
 * ========================================== */
extern SPI_HandleTypeDef hspi_weight;

/* ==========================================
 * 函数接口声明 (Function Prototypes)
 * ========================================== */
unsigned char AD7190_Init(void);
void          AD7190_SetPower(unsigned char pwrMode);
void          AD7190_ChannelSelect(unsigned short channel);
void          AD7190_Calibrate(unsigned char mode, unsigned char channel);
void          AD7190_RangeSetup(unsigned char polarity, unsigned char range);
unsigned int  AD7190_SingleConversion(void);
unsigned int  AD7190_ContinuousReadAvg(unsigned char sampleNumber);
unsigned int  AD7190_TemperatureRead(void);

void          weight_ad7190_conf(void);
unsigned int  weight_ad7190_ReadAvg(unsigned char sampleNumber);
void          Force_sensor_init(void);

#endif /* __BSP_AD7190_H__ */

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
