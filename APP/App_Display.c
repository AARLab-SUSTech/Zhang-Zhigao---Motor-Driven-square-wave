#include <Bsp_Lcd.h>
#include "App_EFA.h"
#include "App_Fault.h"
#include "App_Display.h"
#include "Bsp_Control.h"
#include "Mid_Command_Usart.h"
extern Motor EFA_DATA;

/* ==================================================== *
 * 完美还原 UI 设计图的专属颜色库 (RGB565 格式)
 * ==================================================== */
// 1. 马卡龙背景色 (极浅)
#define UI_BG_GREEN     0xE7FC  // 极浅绿
#define UI_BG_BLUE      0xD71F  // 极浅蓝
#define UI_BG_PURPLE    0xE71F  // 极浅紫
#define UI_BG_DARK      0x2965  // 底部深空灰 (炭黑色)

// 2. 数据文字颜色 (鲜艳)
#define UI_TXT_GREEN    0x2E68  // 深草绿
#define UI_TXT_BLUE     0x4C7D  // 鲜亮蓝
#define UI_TXT_PURPLE   0x913A  // 亮紫色

uint16_t color_H = 0xF800;
uint16_t color_L = BLACK;

/**
 * @brief  UI 静态框架 (严格按照马卡龙设计图布局 280x240)
 */
void App_Display_Init(void)
{
    /* 1. 刷全屏纯白底色 (这层白底将作为框与框之间的分割线) */
    LCD_Fill(0, 0, 280, 240, WHITE);

    /* 2. 填充 5 个静态彩色方框 (预留 4px 的白色间隙) */
    // 顶部行高度: 0 ~ 145
    LCD_Fill(0,   0,  75,  145, UI_BG_GREEN);   /* 左上框: DC (中轴线 X=38) */
    LCD_Fill(79,  0,  201, 145, UI_BG_BLUE);    /* 中上框: HV(V) (中轴线 X=140) */
    LCD_Fill(205, 0,  280, 145, UI_BG_PURPLE);  /* 右上框: HV(A) (中轴线 X=242) */

    // 底部行高度: 149 ~ 240
    LCD_Fill(0,   149, 138, 240, UI_BG_DARK);   /* 左下框: MODE (中轴线 X=69) */
    LCD_Fill(142, 149, 280, 240, UI_BG_DARK);   /* 右下框: STATE (中轴线 X=211) */

    /* 3. 在框内顶部【居中】打上静态标签 (Size 16, 上边距统一为 15) */
    // 顶部黑字
    LCD_ShowString(18,  15, (uint8_t *)"DC(A)", BLACK, UI_BG_GREEN,  16, 0);
    LCD_ShowString(120, 15, (uint8_t *)"HV(V)", BLACK, UI_BG_BLUE,   16, 0);
    LCD_ShowString(222, 15, (uint8_t *)"HV(mA)", BLACK, UI_BG_PURPLE, 16, 0);

    // 底部白字 (反白设计，更具现代感)
    LCD_ShowString(53,  155, (uint8_t *)"MODE",  WHITE, UI_BG_DARK,  24, 0);
    LCD_ShowString(191, 155, (uint8_t *)"STATE", WHITE, UI_BG_DARK,  24, 0);

        // 【新增】把不会变的单位移到静态初始化里，终生只画一次！
        LCD_ShowString(42, 116, (uint8_t *)"Hz", UI_TXT_GREEN, UI_BG_GREEN, 16, 0);
        LCD_ShowString(260, 116, (uint8_t *)"W", RED, UI_BG_PURPLE, 16, 0);

}

///**
// * @brief  UI 动态刷新 (100ms 调用)
// */
//void App_Display_Update(void)
//{
//    static uint32_t last_fault_flags = 0xFFFFFFFF;
//    static uint8_t  last_motor_mode  = 0xFF;
//
//	uint16_t u_color = color_L;
//    uint16_t v_color = color_L;
//    uint16_t w_color = color_L;
//
//    if(Bsp_Get_Dc_Power_State() == 1){//若高压模块没有上电，则输出状态默认是黑色
//    // 根据你的六步换向代码真值表映射颜色
//    switch (EFA_DATA.Step) {
//        case 0: u_color = color_H; v_color = color_L; w_color = color_H; break;
//        case 1: u_color = color_H; v_color = color_L; w_color = color_L; break;
//        case 2: u_color = color_H; v_color = color_H; w_color = color_L; break;
//        case 3: u_color = color_L; v_color = color_H; w_color = color_L; break;
//        case 4: u_color = color_L; v_color = color_H; w_color = color_H; break;
//        case 5: u_color = color_L; v_color = color_L; w_color = color_H; break;
//        default: u_color = color_L; v_color = color_L; w_color = color_L;break;
//    					  }
//    }
//    else{
//    	u_color = color_L; v_color = color_L; w_color = color_L;
//    }
//    // 1. 绘制前先用局部背景色覆盖一下这一小块区域 (防闪屏残影)
//    // 坐标确保在中央浅蓝框的底部
//    //LCD_Fill(100, 105, 180, 140, UI_BG_BLUE);
//
//    // 2. 画三个实心点 (半径 r=5)，这里正式用到了 u_color, v_color, w_color！
//    LCD_Draw_SolidDot(110, 115, 5, u_color, UI_BG_BLUE);
//    LCD_Draw_SolidDot(140, 115, 5, v_color, UI_BG_BLUE);
//    LCD_Draw_SolidDot(170, 115, 5, w_color, UI_BG_BLUE);
//
//    // 3. 在点下方打上极简的 U V W 标签 (如果你只有16号字，就用16号)
//    LCD_ShowString(106, 125, (uint8_t *)"U", UI_TXT_BLUE, UI_BG_BLUE, 16, 0);
//    LCD_ShowString(136, 125, (uint8_t *)"V", UI_TXT_BLUE, UI_BG_BLUE, 16, 0);
//    LCD_ShowString(166, 125, (uint8_t *)"W", UI_TXT_BLUE, UI_BG_BLUE, 16, 0);
//
//    /* ==================================================== *
//     * 1. 顶部三大物理量 (大字号24，Y轴统一为 70，X轴绝对居中)
//     * ==================================================== */
//    /* 左侧 - 低压电流 (len=4, 占48px, 中轴38 -> 起点 X=14) */
//    LCD_ShowFloatNum1(14,  70, EFA_DATA.Dc_I_A,  4, UI_TXT_GREEN,  UI_BG_GREEN,  24);
//
//    /* 左下侧 - 电频率 */
//    LCD_ShowIntNum(14, 116, (int)Mid_Get_Fre_Ele() ,3 , UI_TXT_GREEN, UI_BG_GREEN, 16);
//    //LCD_ShowString(42, 116, (uint8_t *)"Hz", UI_TXT_GREEN, UI_BG_GREEN, 16, 0);
//
//    /* 中央 - 高压电压 (len=5, 占60px, 中轴140 -> 起点 X=110) */
//    //LCD_ShowFloatNum1(100, 70, EFA_DATA.Hv_V_V,  6, UI_TXT_BLUE,   UI_BG_BLUE,   24);
//    LCD_ShowIntNum(100, 70, (int)EFA_DATA.Hv_V_V,  4, UI_TXT_BLUE,   UI_BG_BLUE,   24);
//
//    /* 右侧 - 高压电流 (len=4, 占48px, 中轴242 -> 起点 X=218) */
//    uint16_t hv_i_color = (EFA_DATA.Hv_I_mA > 10.0f) ? RED : UI_TXT_PURPLE;
//    LCD_ShowFloatNum1(218, 70, EFA_DATA.Hv_I_mA, 4, hv_i_color,  UI_BG_PURPLE, 24);
//
//    /* 右下侧 - 高压输出功率 (len=4, 占48px, 中轴242 -> 起点 X=218) */
//    float Hv_Output_Power = EFA_DATA.Hv_I_mA * EFA_DATA.Hv_V_V * 0.001f;
//    LCD_ShowFloatNum1(218, 116, Hv_Output_Power, 4, RED,  UI_BG_PURPLE, 16);
//    //LCD_ShowString(260, 116, (uint8_t *)"W", RED, UI_BG_PURPLE, 16, 0);
//    /* ==================================================== *
//     * 2. 左下框 - 控制模式 (白字，深灰底，X轴绝对居中)
//     * 字符串统一长度为 8, 占96px, 中轴69 -> 起点 X=21
//     * ==================================================== */
//    if (EFA_DATA.Motor_mode == MOTOR_IDLE) {
//        LCD_ShowString(21, 190, (uint8_t *)"IDLE    ", WHITE, UI_BG_DARK, 24, 0);
//    } else if (EFA_DATA.Motor_mode == MOTOR_READY) {
//        LCD_ShowString(21, 190, (uint8_t *)"Ready   ", WHITE, UI_BG_DARK, 24, 0);
//    } else if (EFA_DATA.Motor_mode == MOTOR_ERROR) {
//        LCD_ShowString(21, 190, (uint8_t *)"Error   ", WHITE, UI_BG_DARK, 24, 0);
//    } else if (EFA_DATA.Motor_mode == MOTOR_OPEN_SPEED) {
//        LCD_ShowString(21, 190, (uint8_t *)"Speed   ", WHITE, UI_BG_DARK, 24, 0);
//    } else if (EFA_DATA.Motor_mode == MOTOR_OPEN_POSITION) {
//        LCD_ShowString(21, 190, (uint8_t *)"Position", WHITE, UI_BG_DARK, 24, 0);
//    } else if (EFA_DATA.Motor_mode == MOTOR_OPEN_REPEATED) {
//        LCD_ShowString(21, 190, (uint8_t *)"Repeated", WHITE, UI_BG_DARK, 24, 0);
//    } else if (EFA_DATA.Motor_mode == MOTOR_OPEN_VELOCITY) {
//        LCD_ShowString(21, 190, (uint8_t *)"Velocity", WHITE, UI_BG_DARK, 24, 0);
//    } else if (EFA_DATA.Motor_mode == MOTOR_CLOSE_POSITION) {
//        LCD_ShowString(21, 190, (uint8_t *)"Position", WHITE, UI_BG_DARK, 24, 0);
//    } else if (EFA_DATA.Motor_mode == MOTOR_CLOSE_FORCE) {
//        LCD_ShowString(21, 190, (uint8_t *)"Force   ", WHITE, UI_BG_DARK, 24, 0);
//    } else if (EFA_DATA.Motor_mode == MOTOR_CLOSE_VELOCITY) {
//        LCD_ShowString(21, 190, (uint8_t *)"Velocity", WHITE, UI_BG_DARK, 24, 0);
//    } else if (EFA_DATA.Motor_mode == MOTOR_SYNC_POSITION) {
//        LCD_ShowString(21, 190, (uint8_t *)"Sync    ", WHITE, UI_BG_DARK, 24, 0);
//    } else {
//        LCD_ShowString(21, 190, (uint8_t *)"        ", WHITE,  UI_BG_DARK, 24, 0);
//    }
//
//    /* ==================================================== *
//     * 3. 右下框 - 系统状态 (文字统一白色，根据报错级别动态改变状态框颜色)
//     * 字符串统一长度为 8, 占96px, 中轴211 -> 起点 X=163
//     * ==================================================== */
//    if ((last_fault_flags != EFA_DATA.Fault_Flags) || (last_motor_mode != EFA_DATA.Motor_mode))
//    {
//        last_fault_flags = EFA_DATA.Fault_Flags;
//        last_motor_mode  = EFA_DATA.Motor_mode;
//
//        if (EFA_DATA.Fault_Flags == FAULT_NONE)
//        {
//            if (EFA_DATA.Motor_mode == MOTOR_IDLE) {
//                /* 待机：深灰底，白字 */
//                LCD_Fill(142, 149, 280, 240, UI_BG_DARK);
//                LCD_ShowString(191, 155, (uint8_t *)"STATE", WHITE, UI_BG_DARK, 16, 0);
//                LCD_ShowString(163, 190, (uint8_t *)"IDLE    ", WHITE, UI_BG_DARK, 24, 0);
//            } else {
//                /* 运行：绿底，白字 */
//                LCD_Fill(142, 149, 280, 240, GREEN);
//                LCD_ShowString(191, 155, (uint8_t *)"STATE", WHITE, GREEN, 16, 0);
//                LCD_ShowString(163, 190, (uint8_t *)"RUNNING ", WHITE, GREEN, 24, 0);
//            }
//        }
//        else
//        {
//            /* 故障：红底，白字！ */
//            LCD_Fill(142, 149, 280, 240, RED);
//            LCD_ShowString(191, 155, (uint8_t *)"ALARM", WHITE, RED, 16, 0);
//
//            if (EFA_DATA.Fault_Flags & FAULT_HV_OVER_CURRENT) {
//                LCD_ShowString(163, 190, (uint8_t *)"Over Cur", WHITE, RED, 24, 0);
//            } else if (EFA_DATA.Fault_Flags & FAULT_HV_OVER_VOLTAGE) {
//                LCD_ShowString(163, 190, (uint8_t *)"Over Vol", WHITE, RED, 24, 0);
//            } else {
//                LCD_ShowString(163, 190, (uint8_t *)"SYS ERR ", WHITE, RED, 24, 0);
//            }
//        }
//    }
//}

///**
// * @brief  UI 动态刷新 (100ms 调用)
// */
//void App_Display_Update(void)
//{
//    /* 静态记忆变量：阻挡不必要的重复刷屏 */
//    static uint32_t last_fault_flags = 0xFFFFFFFF;
//    static uint8_t  last_motor_mode  = 0xFF;
//    static uint8_t  last_step        = 0xFF;
//    static uint8_t  last_dc_state    = 0xFF;
//
//    /* ==================================================== *
//     * 1. 动态指示灯：仅在“换向节拍”或“上电状态”变化时才重绘
//     * ==================================================== */
//    uint8_t current_dc_state = Bsp_Get_Dc_Power_State();
//
//    if (last_step != EFA_DATA.Step || last_dc_state != current_dc_state)
//    {
//        last_step = EFA_DATA.Step;
//        last_dc_state = current_dc_state;
//
//        uint16_t u_color = color_L;
//        uint16_t v_color = color_L;
//        uint16_t w_color = color_L;
//
//        if(current_dc_state == 1) {
//            switch (EFA_DATA.Step) {
//                case 0: u_color = color_H; v_color = color_L; w_color = color_H; break;
//                case 1: u_color = color_H; v_color = color_L; w_color = color_L; break;
//                case 2: u_color = color_H; v_color = color_H; w_color = color_L; break;
//                case 3: u_color = color_L; v_color = color_H; w_color = color_L; break;
//                case 4: u_color = color_L; v_color = color_H; w_color = color_H; break;
//                case 5: u_color = color_L; v_color = color_L; w_color = color_H; break;
//                default: u_color = color_L; v_color = color_L; w_color = color_L; break;
//            }
//        }
//
//        // 更新点与字母
//        LCD_Draw_SolidDot(110, 115, 5, u_color, UI_BG_BLUE);
//        LCD_Draw_SolidDot(140, 115, 5, v_color, UI_BG_BLUE);
//        LCD_Draw_SolidDot(170, 115, 5, w_color, UI_BG_BLUE);
//
//        LCD_ShowString(106, 125, (uint8_t *)"U", UI_TXT_BLUE, UI_BG_BLUE, 16, 0);
//        LCD_ShowString(136, 125, (uint8_t *)"V", UI_TXT_BLUE, UI_BG_BLUE, 16, 0);
//        LCD_ShowString(166, 125, (uint8_t *)"W", UI_TXT_BLUE, UI_BG_BLUE, 16, 0);
//    }
//
//    /* ==================================================== *
//     * 2. 实时物理量：这些数据一直在高频变动，允许每次都刷
//     * ==================================================== */
//    LCD_ShowFloatNum1(14,  70, EFA_DATA.Dc_I_A,  4, UI_TXT_GREEN,  UI_BG_GREEN,  24);
//    LCD_ShowIntNum(14, 116, (int)Mid_Get_Fre_Ele() ,3 , UI_TXT_GREEN, UI_BG_GREEN, 16);
//
//    LCD_ShowIntNum(100, 70, (int)EFA_DATA.Hv_V_V,  4, UI_TXT_BLUE,   UI_BG_BLUE,   24);
//
//    uint16_t hv_i_color = (EFA_DATA.Hv_I_mA > 10.0f) ? RED : UI_TXT_PURPLE;
//    LCD_ShowFloatNum1(218, 70, EFA_DATA.Hv_I_mA, 4, hv_i_color,  UI_BG_PURPLE, 24);
//
//    float Hv_Output_Power = EFA_DATA.Hv_I_mA * EFA_DATA.Hv_V_V * 0.001f;
//    LCD_ShowFloatNum1(218, 116, Hv_Output_Power, 4, RED,  UI_BG_PURPLE, 16);
//
//    /* ==================================================== *
//     * 3. 逻辑状态区：仅在“报错”或“模式切换”时才重绘
//     * 把原本毫无保护的 MODE 文本显示也搬进来了！
//     * ==================================================== */
//    if ((last_fault_flags != EFA_DATA.Fault_Flags) || (last_motor_mode != EFA_DATA.Motor_mode))
//    {
//        last_fault_flags = EFA_DATA.Fault_Flags;
//        last_motor_mode  = EFA_DATA.Motor_mode;
//
//        /* --- 刷新左下角 MODE --- */
//        if (EFA_DATA.Motor_mode == MOTOR_IDLE) {
//            LCD_ShowString(21, 190, (uint8_t *)"IDLE    ", WHITE, UI_BG_DARK, 24, 0);
//        } else if (EFA_DATA.Motor_mode == MOTOR_READY) {
//            LCD_ShowString(21, 190, (uint8_t *)"Ready   ", WHITE, UI_BG_DARK, 24, 0);
//        } else if (EFA_DATA.Motor_mode == MOTOR_ERROR) {
//            LCD_ShowString(21, 190, (uint8_t *)"Error   ", WHITE, UI_BG_DARK, 24, 0);
//        } else if (EFA_DATA.Motor_mode == MOTOR_OPEN_SPEED) {
//            LCD_ShowString(21, 190, (uint8_t *)"Speed   ", WHITE, UI_BG_DARK, 24, 0);
//        } else if (EFA_DATA.Motor_mode == MOTOR_OPEN_POSITION) {
//            LCD_ShowString(21, 190, (uint8_t *)"Position", WHITE, UI_BG_DARK, 24, 0);
//        } else if (EFA_DATA.Motor_mode == MOTOR_OPEN_REPEATED) {
//            LCD_ShowString(21, 190, (uint8_t *)"Repeated", WHITE, UI_BG_DARK, 24, 0);
//        } else if (EFA_DATA.Motor_mode == MOTOR_OPEN_VELOCITY) {
//            LCD_ShowString(21, 190, (uint8_t *)"Velocity", WHITE, UI_BG_DARK, 24, 0);
//        } else if (EFA_DATA.Motor_mode == MOTOR_CLOSE_POSITION) {
//            LCD_ShowString(21, 190, (uint8_t *)"Position", WHITE, UI_BG_DARK, 24, 0);
//        } else if (EFA_DATA.Motor_mode == MOTOR_CLOSE_FORCE) {
//            LCD_ShowString(21, 190, (uint8_t *)"Force   ", WHITE, UI_BG_DARK, 24, 0);
//        } else if (EFA_DATA.Motor_mode == MOTOR_CLOSE_VELOCITY) {
//            LCD_ShowString(21, 190, (uint8_t *)"Velocity", WHITE, UI_BG_DARK, 24, 0);
//        } else if (EFA_DATA.Motor_mode == MOTOR_SYNC_POSITION) {
//            LCD_ShowString(21, 190, (uint8_t *)"Sync    ", WHITE, UI_BG_DARK, 24, 0);
//        } else {
//            LCD_ShowString(21, 190, (uint8_t *)"        ", WHITE, UI_BG_DARK, 24, 0);
//        }
//
//        /* --- 刷新右下角 STATE --- */
//        if (EFA_DATA.Fault_Flags == FAULT_NONE)
//        {
//            if (EFA_DATA.Motor_mode == MOTOR_IDLE) {
//                LCD_Fill(142, 149, 280, 240, UI_BG_DARK);
//                LCD_ShowString(191, 155, (uint8_t *)"STATE", WHITE, UI_BG_DARK, 16, 0);
//                LCD_ShowString(163, 190, (uint8_t *)"IDLE    ", WHITE, UI_BG_DARK, 24, 0);
//            } else {
//                LCD_Fill(142, 149, 280, 240, GREEN);
//                LCD_ShowString(191, 155, (uint8_t *)"STATE", WHITE, GREEN, 16, 0);
//                LCD_ShowString(163, 190, (uint8_t *)"RUNNING ", WHITE, GREEN, 24, 0);
//            }
//        }
//        else
//        {
//            LCD_Fill(142, 149, 280, 240, RED);
//            LCD_ShowString(191, 155, (uint8_t *)"ALARM", WHITE, RED, 16, 0);
//
//            if (EFA_DATA.Fault_Flags & FAULT_HV_OVER_CURRENT) {
//                LCD_ShowString(163, 190, (uint8_t *)"Over Cur", WHITE, RED, 24, 0);
//            } else if (EFA_DATA.Fault_Flags & FAULT_HV_OVER_VOLTAGE) {
//                LCD_ShowString(163, 190, (uint8_t *)"Over Vol", WHITE, RED, 24, 0);
//            } else {
//                LCD_ShowString(163, 190, (uint8_t *)"SYS ERR ", WHITE, RED, 24, 0);
//            }
//        }
//    }
//}

/**
 * @brief  UI 动态刷新 (100ms 调用)
 */
void App_Display_Update(void)
{
    /* ==================================================== *
     * 静态记忆变量：阻挡不必要的重复刷屏 (核心防卡顿秘诀)
     * ==================================================== */
    static uint32_t last_fault_flags = 0xFFFFFFFF;
    static uint8_t  last_motor_mode  = 0xFF;
    static uint8_t  last_step        = 0xFF;
    static uint8_t  last_dc_state    = 0xFF;

    /* 物理量专属记忆锁 */
    static int last_dc_i   = -999;
    static int last_fre    = -999;
    static int last_hv_v   = -999;
    static int last_hv_i   = -999;
    static int last_power  = -999;

    /* ==================================================== *
     * 1. 动态指示灯：仅在“换向节拍”或“上电状态”变化时才重绘
     * ==================================================== */
    uint8_t current_dc_state = Bsp_Get_Dc_Power_State();

    if (last_step != EFA_DATA.Step || last_dc_state != current_dc_state)
    {
        last_step = EFA_DATA.Step;
        last_dc_state = current_dc_state;

        uint16_t u_color = color_L;
        uint16_t v_color = color_L;
        uint16_t w_color = color_L;

        if(current_dc_state == 1) {
            switch (EFA_DATA.Step) {
                case 0: u_color = color_H; v_color = color_L; w_color = color_H; break;
                case 1: u_color = color_H; v_color = color_L; w_color = color_L; break;
                case 2: u_color = color_H; v_color = color_H; w_color = color_L; break;
                case 3: u_color = color_L; v_color = color_H; w_color = color_L; break;
                case 4: u_color = color_L; v_color = color_H; w_color = color_H; break;
                case 5: u_color = color_L; v_color = color_L; w_color = color_H; break;
                default: u_color = color_L; v_color = color_L; w_color = color_L; break;
            }
        }

        // 更新点与字母 (包含底色覆盖防闪)
        LCD_Draw_SolidDot(110, 115, 5, u_color, UI_BG_BLUE);
        LCD_Draw_SolidDot(140, 115, 5, v_color, UI_BG_BLUE);
        LCD_Draw_SolidDot(170, 115, 5, w_color, UI_BG_BLUE);

        LCD_ShowString(106, 125, (uint8_t *)"U", UI_TXT_BLUE, UI_BG_BLUE, 16, 0);
        LCD_ShowString(136, 125, (uint8_t *)"V", UI_TXT_BLUE, UI_BG_BLUE, 16, 0);
        LCD_ShowString(166, 125, (uint8_t *)"W", UI_TXT_BLUE, UI_BG_BLUE, 16, 0);
    }

    /* ==================================================== *
     * 2. 实时物理量：如果数值不变化，坚决不重画！(突破 20ms 的关键)
     * ==================================================== */
    /* 2.1 低压电流 (放大10倍比对) */
    int curr_dc_i = (int)(EFA_DATA.Dc_I_A * 10.0f);
    if (curr_dc_i != last_dc_i) {
        last_dc_i = curr_dc_i;
        LCD_ShowFloatNum1(14,  70, EFA_DATA.Dc_I_A,  4, UI_TXT_GREEN,  UI_BG_GREEN,  24);
    }

    /* 2.2 电频率 (整数比对) */
    int curr_fre = (int)Mid_Get_Fre_Ele();
    if (curr_fre != last_fre) {
        last_fre = curr_fre;
        LCD_ShowIntNum(14, 116, curr_fre, 3, UI_TXT_GREEN, UI_BG_GREEN, 16);
    }

    /* 2.3 高压电压 (整数比对) */
    int curr_hv_v = (int)EFA_DATA.Hv_V_V;
    if (curr_hv_v != last_hv_v) {
        last_hv_v = curr_hv_v;
        LCD_ShowIntNum(100, 70, curr_hv_v,  4, UI_TXT_BLUE,   UI_BG_BLUE,   24);
    }

    /* 2.4 高压电流 (放大10倍比对) */
    int curr_hv_i = (int)(EFA_DATA.Hv_I_mA * 10.0f);
    if (curr_hv_i != last_hv_i) {
        last_hv_i = curr_hv_i;
        uint16_t hv_i_color = (EFA_DATA.Hv_I_mA > 10.0f) ? RED : UI_TXT_PURPLE;
        LCD_ShowFloatNum1(218, 70, EFA_DATA.Hv_I_mA, 4, hv_i_color,  UI_BG_PURPLE, 24);
    }

    /* 2.5 功率 (放大10倍比对) */
    float Hv_Output_Power = EFA_DATA.Hv_I_mA * EFA_DATA.Hv_V_V * 0.001f;
    int curr_power = (int)(Hv_Output_Power * 10.0f);
    if (curr_power != last_power) {
        last_power = curr_power;
        LCD_ShowFloatNum1(218, 116, Hv_Output_Power, 4, RED,  UI_BG_PURPLE, 16);
    }

    /* ==================================================== *
     * 3. 逻辑状态区：仅在“报错”或“模式切换”时才重绘
     * ==================================================== */
    if ((last_fault_flags != EFA_DATA.Fault_Flags) || (last_motor_mode != EFA_DATA.Motor_mode))
    {
        last_fault_flags = EFA_DATA.Fault_Flags;
        last_motor_mode  = EFA_DATA.Motor_mode;

        /* --- 刷新左下角 MODE --- */
        if (EFA_DATA.Motor_mode == MOTOR_IDLE) {
            LCD_ShowString(21, 190, (uint8_t *)"IDLE    ", WHITE, UI_BG_DARK, 24, 0);
        } else if (EFA_DATA.Motor_mode == MOTOR_READY) {
            LCD_ShowString(21, 190, (uint8_t *)"Ready   ", WHITE, UI_BG_DARK, 24, 0);
        } else if (EFA_DATA.Motor_mode == MOTOR_ERROR) {
            LCD_ShowString(21, 190, (uint8_t *)"Error   ", WHITE, UI_BG_DARK, 24, 0);
        } else if (EFA_DATA.Motor_mode == MOTOR_OPEN_SPEED) {
            LCD_ShowString(21, 190, (uint8_t *)"Speed   ", WHITE, UI_BG_DARK, 24, 0);
        } else if (EFA_DATA.Motor_mode == MOTOR_OPEN_POSITION) {
            LCD_ShowString(21, 190, (uint8_t *)"Position", WHITE, UI_BG_DARK, 24, 0);
        } else if (EFA_DATA.Motor_mode == MOTOR_OPEN_REPEATED) {
            LCD_ShowString(21, 190, (uint8_t *)"Repeated", WHITE, UI_BG_DARK, 24, 0);
        } else if (EFA_DATA.Motor_mode == MOTOR_OPEN_VELOCITY) {
            LCD_ShowString(21, 190, (uint8_t *)"Velocity", WHITE, UI_BG_DARK, 24, 0);
        } else if (EFA_DATA.Motor_mode == MOTOR_CLOSE_POSITION) {
            LCD_ShowString(21, 190, (uint8_t *)"Position", WHITE, UI_BG_DARK, 24, 0);
        } else if (EFA_DATA.Motor_mode == MOTOR_CLOSE_FORCE) {
            LCD_ShowString(21, 190, (uint8_t *)"Force   ", WHITE, UI_BG_DARK, 24, 0);
        } else if (EFA_DATA.Motor_mode == MOTOR_CLOSE_VELOCITY) {
            LCD_ShowString(21, 190, (uint8_t *)"Velocity", WHITE, UI_BG_DARK, 24, 0);
        } else if (EFA_DATA.Motor_mode == MOTOR_SYNC_POSITION) {
            LCD_ShowString(21, 190, (uint8_t *)"Sync    ", WHITE, UI_BG_DARK, 24, 0);
        } else {
            LCD_ShowString(21, 190, (uint8_t *)"        ", WHITE,  UI_BG_DARK, 24, 0);
        }

        /* --- 刷新右下角 STATE --- */
        if (EFA_DATA.Fault_Flags == FAULT_NONE)
        {
            if (EFA_DATA.Motor_mode == MOTOR_IDLE) {
                LCD_Fill(142, 149, 280, 240, UI_BG_DARK);
                LCD_ShowString(191, 155, (uint8_t *)"STATE", WHITE, UI_BG_DARK, 24, 0);
                LCD_ShowString(163, 190, (uint8_t *)"IDLE    ", WHITE, UI_BG_DARK, 24, 0);
            } else {
                LCD_Fill(142, 149, 280, 240, GREEN);
                LCD_ShowString(191, 155, (uint8_t *)"STATE", WHITE, GREEN, 24, 0);
                LCD_ShowString(163, 190, (uint8_t *)"RUNNING ", WHITE, GREEN, 24, 0);
            }
        }
        else
        {
            LCD_Fill(142, 149, 280, 240, RED);
            LCD_ShowString(191, 155, (uint8_t *)"ALARM", WHITE, RED, 16, 0);

            if (EFA_DATA.Fault_Flags & FAULT_HV_OVER_CURRENT) {
                LCD_ShowString(163, 190, (uint8_t *)"Over Cur", WHITE, RED, 24, 0);
            } else if (EFA_DATA.Fault_Flags & FAULT_HV_OVER_VOLTAGE) {
                LCD_ShowString(163, 190, (uint8_t *)"Over Vol", WHITE, RED, 24, 0);
            } else {
                LCD_ShowString(163, 190, (uint8_t *)"SYS ERR ", WHITE, RED, 24, 0);
            }
        }
    }
}
