/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  * n
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "control.h"
#include "command.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint16_t duty_TIM1 = 999;

volatile int32_t repeated_pos_A = 0;      // 往复运动点A
volatile int32_t repeated_pos_B = 0;      // 往复运动点B
volatile uint32_t repeated_count_total = 0; // 需要往复的总次数
volatile uint32_t repeated_count_current = 0; // 当前已完成的次数
volatile float repeated_speed_hz = 0.0f;    // 往复运动时使用的速度
volatile uint32_t phase_accumulator = 0;
volatile uint32_t phase_increment = 0;
volatile uint16_t pwm_duty_value = 0;
volatile uint8_t step = 0;
float interrupt_freq_hz = 20000.0f; // 您TIM6中断的频率 (1 / 0.00005s)

volatile MotorDirection_t motor_direction = MOTOR_FORWARD; // 默认为正

volatile Motor_mode_t Motor_mode = MOTOR_IDLE;

// --- 新增：用于累计步数的全局变量 ---
// 使用 signed 32-bit 整数，可以记录正反转，且范围足够大
// 使用 volatile 关键字，确保在中断和主循环中安全访问
volatile int32_t absolute_step_counter = 0;
// 目标步数位置
volatile int32_t target_step_position = 0;
// 在位置模式下，电机移动到目标点时使用的速度
volatile uint32_t position_mode_increment = 10;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

bool DC_ON_State = false;

uint16_t ARR;
uint32_t ADC1_RAW_data[2];
uint32_t ADC2_RAW_data[3];
float rad_omega;
float HV_V,HV_I,DC_I;
float MAX_HV_voltage=1300;
float MAX_HV_current=12;
float MAX_DC_current=2;

// --- 功率计算相关变量 ---
// 直流输入电压 (输入为20V)
const float DC_INPUT_VOLTAGE = 20.0f;

// 瞬时功率
float dc_input_power = 0.0f;
float hv_instantaneous_power = 0.0f;

/*
 *
 */

// 周期平均功率相关
float hv_power_accumulator = 0.0f; // 用于累加一个电周期内的瞬时功率
uint32_t hv_power_sample_count = 0;   // 用于计算一个电周期内的采样点数
float hv_average_power_cycle = 0.0f;  // 存储每个电周期计算出的平均功率
float last_hv_average_power_cycle = 0.0f;  // 存储上个电周期计算出的平均功率
// 每秒平均功率相关
float hv_power_accumulator_1s = 0.0f;   // 用于累加一秒内的瞬时功率
uint32_t hv_power_sample_count_1s = 0;  // 用于计算一秒内的采样点数
float hv_average_power_1s = 0.0f;       // 存储每秒计算出的平均功率
float last_hv_average_power_1s = 0.0f;  // 存储上1s计算出的平均功率

float dma_float_data[6];
/* USER CODE END PV */

uint8_t loop_count;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

COMP_HandleTypeDef hcomp1;
COMP_HandleTypeDef hcomp2;

DAC_HandleTypeDef hdac3;

FDCAN_HandleTypeDef hfdcan1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
// 1. 算法参数 (您可以根据实际情况调整)
const float  SPIKE_CURRENT_THRESHOLD   = 16.0f;    // 定义尖峰电流的阈值 (单位: mA)
const uint16_t CHECK_WINDOW_SAMPLES    = 60;      // 定义检查窗口的大小 (N个采样点)。200个点 @ 20kHz = 1ms
const uint16_t MAX_SPIKES_IN_WINDOW    = 50;      // 定义在一个窗口期内，允许出现的最大尖峰次数

// 2. 算法工作变量
uint16_t window_sample_counter = 0;   // 用于在窗口内计数的采样点计数器 (从0数到CHECK_WINDOW_SAMPLES)
uint16_t spike_count_in_window = 0;   // 用于累计一个窗口期内的尖峰次数
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM7_Init(void);
static void MX_COMP1_Init(void);
static void MX_COMP2_Init(void);
static void MX_DAC3_Init(void);
static void MX_TIM6_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__									//串口重定向
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart1 , (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int float_current_counter=0;
float current_1ms=0;
float current_hvi;
float dc_current_1ms=0;
float current_dc;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim16)
	{
		dma_float_data[0] = last_hv_average_power_cycle;
		dma_float_data[1] = last_hv_average_power_1s;
		dma_float_data[2] = dc_current_1ms * 20.0f;
		dma_float_data[3] = absolute_step_counter;
		dma_float_data[4] = (float)spike_count_in_window;
		dma_float_data[5] = current_1ms;
		send_float_array_dma(dma_float_data, 6);
	}
	if(htim == &htim6)
	{
    	int temp_data[2];
    	temp_data[0] = (2048 - ADC1_RAW_data[0]) * 2;
    	temp_data[1] = (2048 - ADC1_RAW_data[1]) * 2;
//    	temp_data[0] = (ADC1_RAW_data[0] - 2048) * 2;
//    	temp_data[1] = (ADC1_RAW_data[1] - 2048) * 2;
    	//V  mA
    	HV_V = temp_data[0] * 1.00909423828125f;//HV_V = (temp_data[0] / 4096) * 3.3f * 2 * 501 / 0.4f
        HV_I = (temp_data[1] * 0.01007080078125);//HV_I = (temp_data[1] / 4096) * 3.3 / 8 / 10 * 1000
    	DC_I = ADC2_RAW_data[0] * 0.0008056640625f;//(ADC2_RAW_data[0] / 4096) * 3.3 / 200 / 0.005

        // 1. 检查当前电流是否形成了一次尖峰
        if (HV_I > SPIKE_CURRENT_THRESHOLD)
        {
        	spike_count_in_window++;
        }

        // 2. 采样点计数器自增
        window_sample_counter++;

        // 3. 判断一个检查窗口是否已经结束
        if (window_sample_counter >= CHECK_WINDOW_SAMPLES)
        {
            // 窗口结束，进行判断
            if (spike_count_in_window > MAX_SPIKES_IN_WINDOW)
            {
                // 在过去的N个采样点中，尖峰次数过多，判定为故障！
                Motor_mode = MOTOR_OVER_HV_CURRENT; // 设置故障状态
                Close_output();
                DC_Power_CTR(false);
                Buzzer_ON;
                // 您现有的安全检查层会捕捉到这个状态并执行保护
            }

            // 4. 重置计数器，为下一个检查窗口做准备
            window_sample_counter = 0;
            spike_count_in_window = 0;
        }

        float_current_counter++;
        current_hvi += HV_I;
        current_dc += DC_I;

        if(float_current_counter == 20)
        {
        	current_1ms = current_hvi/20.0f;
        	current_hvi = 0;
        	dc_current_1ms = current_dc/20.0f;
        	current_dc = 0;
        	float_current_counter = 0;
        }

		// 1. 计算直流输入瞬时功率 (P_dc = V_dc * I_dc)
		dc_input_power = DC_INPUT_VOLTAGE * DC_I;
		// 2. 计算高压输出瞬时功率 (P_hv = V_hv * I_hv)
		hv_instantaneous_power = HV_V * HV_I * 0.001f;
		// --- B. 进行周期平均和每秒平均功率的累加 ---

		// 3. 为“每周期平均功率”累加
		hv_power_accumulator += hv_instantaneous_power;
		hv_power_sample_count++;

		// 4. 为“每秒平均功率”累加
		hv_power_accumulator_1s += hv_instantaneous_power;
		hv_power_sample_count_1s++;

		// 中断频率是20kHz, 所以200次中断就是0.01秒
		if (hv_power_sample_count_1s >= 200)
		{
			last_hv_average_power_1s = hv_power_accumulator_1s / hv_power_sample_count_1s;// 计算并更新上一秒的平均功率
			hv_power_sample_count_1s = 0;   //清零计数
			hv_power_accumulator_1s = 0.0f;// 重置，为下一秒准备
		}

//    	loop_count++;
//    	if(loop_count == 100)
//    	{
//    			loop_count = 0;
//       	    send_float_array_dma(&HV_V, 1);
//    	}

    		if(HV_V > MAX_HV_voltage)  { Motor_mode = MOTOR_OVER_HV_VOLTAGE;    Close_output();DC_Power_CTR(false);Buzzer_ON;}
//   		    if(HV_I > MAX_HV_current)  { Motor_mode = MOTOR_OVER_HV_CURRENT;    Close_output();DC_Power_CTR(false);Buzzer_ON;}
    		if(DC_I > MAX_DC_current)  { Motor_mode = MOTOR_OVER_DC_IN_CURRENT; Close_output();DC_Power_CTR(false);Buzzer_ON;}

        // --- 1. 安全检查层：处理最高优先级的 IDLE 和 ERROR 状态 ---
        if (Motor_mode == MOTOR_IDLE || Motor_mode == MOTOR_ERROR || Motor_mode == MOTOR_OVER_HV_CURRENT || Motor_mode == MOTOR_OVER_HV_VOLTAGE || Motor_mode == MOTOR_OVER_DC_IN_CURRENT)
        {
        	DC_Power_OFF;
            phase_increment = 0; // 强制速度为0，确保电机停止
        }
        else
        {
        	DC_Power_ON;
        		// --- 1. 状态决策层：根据当前模式决定电机的目标和速度 ---
				if (Motor_mode == MOTOR_OPEN_REPEATED)
				{
					// 在往复模式下，检查上一个移动是否已完成 (表现为电机已停止)
					if (phase_increment == 0)
					{
						// 检查总次数是否已完成
						if (repeated_count_current >= repeated_count_total)
						{
							Motor_mode = MOTOR_OPEN_SPEED; // 任务完成，切换到速度模式并保持静止
						}
						else
						{
							// 任务未完成，决定下一个目标点
							if (absolute_step_counter == repeated_pos_A)
							{
								// 当前在A点，下一个目标是B点
								target_step_position = repeated_pos_B;
							}
							else // 当前在B点 (或者初始位置)
							{
								// 下一个目标是A点
								target_step_position = repeated_pos_A;
								// 完成一次 B->A 的移动，才算一个完整的往复周期
								if (absolute_step_counter == repeated_pos_B)
								{
									repeated_count_current++;
								}
							}
						}
					}
				}
			// --- 2. 运动执行层：根据目标驱动电机 ---
				 // 如果是任何需要定位的模式 (单次或往复)
				 if (Motor_mode == MOTOR_OPEN_POSITION || Motor_mode == MOTOR_OPEN_REPEATED)
				 {
					 if (absolute_step_counter < target_step_position)
					 {
						 motor_direction = MOTOR_FORWARD;
						 phase_increment = position_mode_increment;
					 }
					 else if (absolute_step_counter > target_step_position)
					 {
						 motor_direction = MOTOR_REVERSE;
						 phase_increment = position_mode_increment;
					 }
					 else // 已到达目标
					 {
						 phase_increment = 0;
					 }
				 }
				 // 注意: 在 MOTOR_OPEN_SPEED 模式下，上面两个if块都不会进入，
				 // phase_increment 会保持由 set_speed() 函数设定的值，电机将持续旋转。
        }

        // 1. 累加/累减相位 (无论何种模式，都基于最终计算出的 phase_increment)
        // 根据方向执行累加或累减
        if (motor_direction == MOTOR_FORWARD) { phase_accumulator += phase_increment; }
           else 							  { phase_accumulator -= phase_increment; }

		// 2. 计算扇区 (0-5)
		// 思想：将 0x00000000 ~ 0xFFFFFFFF 的范围等分成 6 份
		// 方法：(phase_accumulator * 6) / 2^32
		// 使用64位乘法和右移32位来实现，非常高效
		uint8_t new_step = (uint8_t)(((uint64_t)phase_accumulator * 6) >> 32);

        if (new_step > 5) new_step = 5; // 防止浮点数误差导致越界

        if(step != new_step)
        {

			// 【修正】同时判断正转(5->0)和反转(0->5)的周期结束点
			if ( (step == 5 && new_step == 0) || (step == 0 && new_step == 5) )
			{
				// 周期结束的计算逻辑本身是正确的
				if (hv_power_sample_count > 0)
				{
					// 计算并更新上个周期的平均功率
					last_hv_average_power_cycle = hv_power_accumulator / hv_power_sample_count;
				}
				// 重置，为下个周期准备
				hv_power_accumulator = 0.0f;
				hv_power_sample_count = 0;
			}
            if (motor_direction == MOTOR_FORWARD) { absolute_step_counter++; }// 正转，计数值加1
                else                              { absolute_step_counter--; }// 反转，计数值减1

            step = new_step; // 更新当前步骤

            Update_output(step);
        }


	}
}

#define MY_NODE_ID      0x101  // <-- 【重要】在这里设置本节点的实际ID，例如5号节点
#define BROADCAST_ID    0x100  // <-- 我们协议中定义的广播ID

FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader; // 用于存储接收报文的头信息
uint8_t RxData[8];              // 用于存储接收报文的数据
uint8_t TxData[8];
/*
Data[0] = 0x01: 紧急停止 (Emergency Stop)

Data[0] = 0x02: 步进+ (相对位置正向运动)

Data[0] = 0x03: 步进- (相对位置反向运动)

Data[0] = 0x04: 往复模式 (Reciprocating Mode)

Data[0] = 0x05: 绝对位置模式 (Absolute Position Mode) - 【新增】
 */
// FDCAN接收FIFO 0消息挂起回调函数
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    // 检查是否是“新消息到达”中断
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        // 从Rx FIFO 0中获取消息，存入上面的全局变量中
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
        {
            // --- 在这里添加您自己的数据处理逻辑 ---
            printf("Message Received!\r\n");
            printf("  ID   : 0x%lX\r\n", RxHeader.Identifier);
            printf("  DLC  : %ld bytes\r\n", RxHeader.DataLength);
            printf("  Data : ");
            for(int i=0; i < RxHeader.DataLength; i++)
            {
                printf("0x%02X ", RxData[i]);
            }
            printf("\r\n");

            if((RxHeader.Identifier == MY_NODE_ID ) || (RxHeader.Identifier == BROADCAST_ID ))
            {
            // --- BEGIN CAN PROTOCOL PROCESSING (v2) ---

                        uint8_t command = RxData[0]; // 提取命令字

                        switch(command)
                        {
                            case 0x01: // 命令: 紧急停止
                            {
	                        	DC_Power_OFF;
	                        	if((Motor_mode != MOTOR_OVER_HV_VOLTAGE) || (Motor_mode != MOTOR_OVER_HV_CURRENT) || (Motor_mode != MOTOR_OVER_DC_IN_CURRENT || Motor_mode != MOTOR_ERROR))
	                        	{
	                     		Motor_mode = MOTOR_IDLE;
	                        	}
	                     		Close_output();
	                     		HAL_TIM_Base_Stop(&htim16);//dma printf
                                break;
                            }

                            case 0x02: // 命令: 步进+ (正向相对运动)
                            case 0x03: // 命令: 步进- (反向相对运动)
                            {
                                // 1. 解析参数
                                uint16_t speed_hz = (uint16_t)RxData[1] |
                                                    (uint16_t)(RxData[2] << 8);

                                // 2. 计算并设置电机控制参数
                                position_mode_increment = (uint32_t)(((uint64_t)speed_hz * 0x100000000) / interrupt_freq_hz);

                                // 3. 设置电机模式和目标
                                Motor_mode = MOTOR_OPEN_POSITION;

                                if (command == 0x02) // 正向
                                {
   	                        	 Motor_mode = MOTOR_OPEN_POSITION;
   	                        	 step_move(1, 10);
                                }
                                else // 反向 (command == 0x03)
                                {
   	                        	 Motor_mode = MOTOR_OPEN_POSITION;
   	                        	 step_move(-1, 10);
                                }
                                printf("CMD: Relative Step %c, Speed: %u Hz\r\n", (command == 0x02 ? '+' : '-'), speed_hz);
                                break;
                            }

                            case 0x04: // 命令: 往复模式
                            {
                                // 1. 解析参数
                                uint16_t speed_hz = (uint16_t)RxData[1] |
                                					(uint32_t)(RxData[2] << 8);
                                uint16_t steps_range_A = (uint32_t)RxData[3] |
                                                       (uint32_t)(RxData[4] << 8);

                                uint16_t steps_range_B = (uint32_t)RxData[5] |
                                                       (uint32_t)(RxData[6] << 8);

                                uint8_t count =       (uint32_t)RxData[7];

                                // 2. 计算速度
                                position_mode_increment = (uint32_t)(((uint64_t)speed_hz * 0x100000000) / interrupt_freq_hz);

                                // 3. 设置往复运动参数
                                repeated_pos_A = steps_range_A;
                                repeated_pos_B = steps_range_B;
                                repeated_count_total = count;
                                repeated_count_current = 0;

                                // 4. 启动往复模式
	                        	Motor_mode = MOTOR_OPEN_REPEATED;
	                        	move_repeatedly(repeated_pos_A, repeated_pos_B, repeated_count_total, speed_hz);

                                printf("CMD: Repeated Mode, Range: %d   %d steps, Speed: %u Hz  count:%d \r\n", steps_range_A, steps_range_B, speed_hz , count);
                                break;
                            }

                            case 0x05: // 【新增】命令: 绝对位置模式
                            {
                                // 1. 解析参数
                                // 在此模式下，参数代表的是绝对位置坐标

                                uint16_t speed_hz = (uint16_t)RxData[1] |
                                                    (uint16_t)(RxData[2] << 8);

                                uint32_t absolute_pos = (uint32_t)RxData[3] |
                                                        (uint32_t)(RxData[4] << 8);
                                // 2. 计算速度
                                position_mode_increment = (uint32_t)(((uint64_t)speed_hz * 0x100000000) / interrupt_freq_hz);

                                // 3. 设置电机模式和目标
                                Motor_mode = MOTOR_OPEN_POSITION;
                                // 直接将目标位置设置为指令中的绝对位置
                                target_step_position = absolute_pos;
	                            move_to_position(target_step_position , speed_hz);
                                printf("CMD: Absolute Position, Target: %ld, Speed: %u Hz\r\n", target_step_position, speed_hz);
                                break;
                            }

                            default:
                            {
                                // 收到未知的命令
                                printf("ERR: Unknown Command 0x%02X\r\n", command);
                                break;
                            }
                        }
                        // --- END CAN PROTOCOL PROCESSING (v2) ---
            }
            else
            {
                printf("ID error\r\n");
            }

        }

        // 【非常重要】: 每次处理完中断后，必须重新激活通知，否则中断只会触发一次！
        if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
        {
           Error_Handler();
        }
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM7_Init();
  MX_COMP1_Init();
  MX_COMP2_Init();
  MX_DAC3_Init();
  MX_TIM6_Init();
  MX_FDCAN1_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

//  Close_output();//关闭所有输出

  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer, RX_BUFFER_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_DIFFERENTIAL_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

  HAL_ADC_Start_DMA(&hadc1, ADC1_RAW_data, 2);// HV_V HV_I
  HAL_ADC_Start_DMA(&hadc2, ADC2_RAW_data, 3);//DC_I SIN COS
  HAL_TIM_Base_Start_IT(&htim7);//trigger

  //（ 10M + 20K )/20K = 501  ---- AMC1350 * 0.4   1/（（1/501）*0.4 ） = 501/0.4 = 1252.5   2400/1252.5 = 1.916167664670659V
//  HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 460);//HVOUTPUT set MAX V  2500V  --  4.99001996007984V    5*0.4 = 2V  报警电压1.4 +- 1 V = 2.4V/0.4V
//  HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 3000);//10ma max
//  HAL_DAC_Start(&hdac3, DAC_CHANNEL_1);
//  HAL_DAC_Start(&hdac3, DAC_CHANNEL_2);

//  HAL_COMP_Start(&hcomp1);
//  HAL_COMP_Start(&hcomp2);

  HAL_Delay(1000);

  rad_omega = 10;
  // 公式： phase_increment = (电机频率 / 中断频率) * 2^32
  // 我们使用 64 位整数来计算以避免溢出
  phase_increment = (uint32_t)(((uint64_t)rad_omega * 0x100000000) / interrupt_freq_hz);
  HAL_TIM_Base_Start_IT(&htim6);//换向代码

  FDCAN_FilterTypeDef sFilterConfig;

  sFilterConfig.IdType = FDCAN_STANDARD_ID;       // ID类型：标准ID
  sFilterConfig.FilterIndex = 0;                  // 过滤器索引，0-27
  sFilterConfig.FilterType = FDCAN_FILTER_DUAL;   // 过滤器类型：
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 匹配成功后存入Rx FIFO 0
  sFilterConfig.FilterID1 = MY_NODE_ID;                // 要匹配的ID
  sFilterConfig.FilterID2 = BROADCAST_ID;              // 广播ID

  // 应用此过滤器配置
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
  {
  	printf("rxerror\r\n");
      Error_Handler();
  }
    /* 1. 启动 FDCAN 外设 */
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
      // 如果HAL_FDCAN_Start失败，会在这里打印信息然后卡死
      printf("FATAL: HAL_FDCAN_Start() FAILED%d!\r\n",HAL_FDCAN_Start(&hfdcan1));
      Error_Handler();
    }
    printf("INFO: HAL_FDCAN_Start() OK.\r\n");
    printf("--- FIRMWARE VERSION 2.0 -- FILTER TEST ---\r\n");

    // 激活接收FIFO 0新消息通知，这是开启接收中断的大门
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }

    /* 2. 配置发送报文头和数据 */
    TxHeader.Identifier = 0x001;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_FRAME_CLASSIC;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    for(int i=0; i<8; i++) {
        TxData[i] = i;
    }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(Motor_mode == MOTOR_OVER_HV_VOLTAGE)
	  {
		  printf("MOTOR_OVER_HV_VOLTAGE\r\n");
			HAL_Delay(1000);
	  }
	  else if(Motor_mode == MOTOR_OVER_HV_CURRENT)
	  {
		  printf("MOTOR_OVER_HV_CURRENT\r\n");
			HAL_Delay(1000);
	  }
	  else if(Motor_mode == MOTOR_OVER_DC_IN_CURRENT)
	  {
		  printf("MOTOR_OVER_DC_IN_CURRENT\r\n");
			HAL_Delay(1000);
	  }
	  else
	  {
//		  printf("%.3f,%.3f,%.3f,%ld\r\n",last_hv_average_power_1s,last_hv_average_power_cycle,dc_input_power,absolute_step_counter);
//		  printf("%.3f,%.3f,%.3f,%ld\r\n",HV_V,HV_I,DC_I,absolute_step_counter);
	  }
		HAL_Delay(1000);
	    HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
//	    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0)
//	    {
//	      if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK)
//	      {
//	        Error_Handler();
//	      }
//	    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T7_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_256;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_8;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 3;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T7_TRGO;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = ENABLE;
  hadc2.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_64;
  hadc2.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_6;
  hadc2.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc2.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_17;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief COMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP1_Init(void)
{

  /* USER CODE BEGIN COMP1_Init 0 */

  /* USER CODE END COMP1_Init 0 */

  /* USER CODE BEGIN COMP1_Init 1 */

  /* USER CODE END COMP1_Init 1 */
  hcomp1.Instance = COMP1;
  hcomp1.Init.InputPlus = COMP_INPUT_PLUS_IO1;
  hcomp1.Init.InputMinus = COMP_INPUT_MINUS_DAC3_CH1;
  hcomp1.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp1.Init.Hysteresis = COMP_HYSTERESIS_HIGH;
  hcomp1.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING;
  if (HAL_COMP_Init(&hcomp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP1_Init 2 */

  /* USER CODE END COMP1_Init 2 */

}

/**
  * @brief COMP2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP2_Init(void)
{

  /* USER CODE BEGIN COMP2_Init 0 */

  /* USER CODE END COMP2_Init 0 */

  /* USER CODE BEGIN COMP2_Init 1 */

  /* USER CODE END COMP2_Init 1 */
  hcomp2.Instance = COMP2;
  hcomp2.Init.InputPlus = COMP_INPUT_PLUS_IO2;
  hcomp2.Init.InputMinus = COMP_INPUT_MINUS_DAC3_CH2;
  hcomp2.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp2.Init.Hysteresis = COMP_HYSTERESIS_HIGH;
  hcomp2.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp2.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING;
  if (HAL_COMP_Init(&hcomp2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP2_Init 2 */

  /* USER CODE END COMP2_Init 2 */

}

/**
  * @brief DAC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC3_Init(void)
{

  /* USER CODE BEGIN DAC3_Init 0 */

  /* USER CODE END DAC3_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC3_Init 1 */

  /* USER CODE END DAC3_Init 1 */

  /** DAC Initialization
  */
  hdac3.Instance = DAC3;
  if (HAL_DAC_Init(&hdac3) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac3, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac3, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC3_Init 2 */

  /* USER CODE END DAC3_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 17;
  hfdcan1.Init.NominalSyncJumpWidth = 2;
  hfdcan1.Init.NominalTimeSeg1 = 7;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 1;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 16;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 499;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIMEx_EnableDeadTimePreload(&htim1);
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 199;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 169;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 249;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 124;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 16;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 499;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 1699;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 9999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 169;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 2000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Power_ON_Pin|CH1__CTR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PF1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : Power_ON_Pin */
  GPIO_InitStruct.Pin = Power_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Power_ON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CH1__CTR_Pin */
  GPIO_InitStruct.Pin = CH1__CTR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CH1__CTR_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
