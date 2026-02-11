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
//TEST
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "control.h"
#include "command.h"
#include "ad7190.h"
#include "foc.h"
#include "can.h"
volatile Force_sensor Force_Sensor1;
P_Controller Motor_Position_Controller;// 定义控制器实例
P_Controller Motor_Force_Controller;// 定义控制器实例
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint16_t duty_TIM1 = 1000;

volatile int32_t repeated_pos_A = 0;      // 往复运动点A
volatile int32_t repeated_pos_B = 0;      // 往复运动点B
volatile uint32_t repeated_count_total = 0; // 需要往复的总次数
volatile uint32_t repeated_count_current = 0; // 当前已完成的次数
volatile float repeated_speed_hz = 0.0f;    // 往复运动时使用的速度
volatile uint32_t phase_accumulator = 0;
volatile uint32_t phase_increment = 0;
volatile uint16_t pwm_duty_value = 0;
volatile uint8_t step = 0;
float interrupt_freq_hz = 10000.0f; // 您TIM6中断的频率 (1 / 0.00005s)

volatile MotorDirection_t motor_direction = MOTOR_FORWARD; // 默认为正
volatile Motor_mode_t Motor_mode = MOTOR_IDLE;

bool Sin_Velocity_Flag = false;
float Omega_Sin_Velocity,Max_Velocity;

int dma_print_flag = 0;
// --- 新增：用于累计步数的全局变量 ---
// 使用 signed 32-bit 整数，可以记录正反转，且范围足够大
// 使用 volatile 关键字，确保在中断和主循环中安全访问
volatile int32_t absolute_step_counter = 0;
// 目标步数位置
volatile int32_t target_step_position = 0;
// 在位置模式下，电机移动到目标点时使用的速度
volatile uint32_t position_mode_increment = 10;
// 在速度模式下，电机移动速度
volatile uint32_t velocity_mode_increment = 0;

#define AD7190_Force_sensor 0
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t rx_buffer[RX_BUFFER_SIZE];        // 在这里为 rx_buffer 分配了 256 字节
uint8_t process_buffer[RX_BUFFER_SIZE]; // 在这里为 process_buffer 分配了 256 字节
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

bool DC_ON_State = false;

uint16_t ARR;
uint32_t ADC1_RAW_data[2];
uint32_t ADC2_RAW_data[3];

int16_t dma_int16_data[100];
/* USER CODE END PV */

uint8_t loop_count;
Motor EMA_DATA;

int last_time_ms,time_gap;
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

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */


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
static void MX_SPI1_Init(void);
static void MX_TIM17_Init(void);
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

#define CHANNEL_COUNT 6 // 您有6个通道

// 定义JustFloat的数据帧结构
typedef struct __attribute__((packed)) {
    float fdata[CHANNEL_COUNT]; // 6个float数据
    uint8_t tail[4];            // 4字节的帧尾
} JustFloatFrame_t;

// 创建一个静态的发送包实例
static JustFloatFrame_t Tx_A_buffer_20k;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t Flag_Abuffer0_Bbuffer1 = 0;//A buffer0-----B buffer1
volatile uint8_t Tx_sample_count = 0;
volatile float weight_g_temp=0;

float rad_omega;
float HV_V,HV_I,DC_I;
float MAX_HV_voltage=2600;
float MAX_HV_current=12;
float MAX_DC_current=4;

int float_current_counter=0;
float current_1ms=0;
float current_hvi;
float dc_current_1ms=0;
float current_dc;

// --- 功率计算相关变量 ---
// 直流输入电压 (输入为20V)
const float DC_INPUT_VOLTAGE = 20.0f;

// 瞬时功率
float dc_input_power = 0.0f;
float hv_instantaneous_power = 0.0f;

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

// 1. 算法参数 (您可以根据实际情况调整)
const float  SPIKE_CURRENT_THRESHOLD   = 16.0f;    // 定义尖峰电流的阈值 (单位: mA)
const uint16_t CHECK_WINDOW_SAMPLES    = 60;      // 定义检查窗口的大小 (N个采样点)。200个点 @ 20kHz = 1ms
const uint16_t MAX_SPIKES_IN_WINDOW    = 50;      // 定义在一个窗口期内，允许出现的最大尖峰次数

// 2. 算法工作变量
uint16_t window_sample_counter = 0;   // 用于在窗口内计数的采样点计数器 (从0数到CHECK_WINDOW_SAMPLES)
uint16_t spike_count_in_window = 0;   // 用于累计一个窗口期内的尖峰次数

extern volatile uint8_t g_uart_dma_transfer_complete;
float speed_test;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim17)
	{
		//CAN heart
		// 1. 计算下一个 "head" 指针的位置   (使用位运算 & (TX_QUEUE_SIZE - 1) 比 % 更高效, 因为 TX_QUEUE_SIZE = 16)
					uint16_t next_head = (g_tx_queue_head + 1) & (TX_QUEUE_SIZE - 1);
		// 2. 检查队列是否已满 (如果 head 的下一个位置就是 tail)
		            if (next_head == g_tx_queue_tail)
		            {
		                // 队列已满，本次心跳被丢弃
		            }
		            else
		             {
		                // 3. 队列未满，获取 "head" 位置的“集装箱”  //    (注意：我们总是在 g_tx_queue_head 指向的位置填充)
		                volatile CanTxMessage_t* msg_to_queue = &g_tx_queue[g_tx_queue_head];

		                // 4. 填充报文头 (复制模板，再修改特定部分)
		                msg_to_queue->Tx_Header = TxHeader;         			 // 复制模板
		                msg_to_queue->Tx_Header.Identifier = HEART_ID;           // 【心跳ID】
		                msg_to_queue->Tx_Header.DataLength = FDCAN_DLC_BYTES_1;  // 【灵活DLC】

		                // 5. 填充报文数据
		                msg_to_queue->Data[0] = (uint8_t)Motor_mode;    // 填入当前模式/故障码

		                for (int i = 1; i < 8; i++) { msg_to_queue->Data[i] = 0x00; }

		                 // 6. 【原子操作】移动头指针，正式将消息放入队列
		                 g_tx_queue_head = next_head;
		             }
	}

	if(htim == &htim16)//用于往复模式下打印往复次数以及电流电压等
	{
		//CAN heart
		// 1. 计算下一个 "head" 指针的位置   (使用位运算 & (TX_QUEUE_SIZE - 1) 比 % 更高效, 因为 TX_QUEUE_SIZE = 16)
					uint16_t next_head = (g_tx_queue_head + 1) & (TX_QUEUE_SIZE - 1);
					uint16_t temp_data;
		// 2. 检查队列是否已满 (如果 head 的下一个位置就是 tail)
		            if (next_head == g_tx_queue_tail)
		            {
		                // 队列已满，本次心跳被丢弃
		            }
		            else
		             {
		                // 3. 队列未满，获取 "head" 位置的“集装箱”  //    (注意：我们总是在 g_tx_queue_head 指向的位置填充)
		                volatile CanTxMessage_t* msg_to_queue = &g_tx_queue[g_tx_queue_head];

		                // 4. 填充报文头 (复制模板，再修改特定部分)
		                msg_to_queue->Tx_Header = TxHeader;         			 // 复制模板
		                msg_to_queue->Tx_Header.Identifier = MESSAGE_ID;           // 【心跳ID】
		                msg_to_queue->Tx_Header.DataLength = FDCAN_DLC_BYTES_8;  // 【灵活DLC】

		                // 5. 填充报文数据
		                msg_to_queue->Data[0] = (uint8_t)((repeated_count_current >> 24) & 0xFF);
		                msg_to_queue->Data[1] = (uint8_t)((repeated_count_current >> 16) & 0xFF);
		                msg_to_queue->Data[2] = (uint8_t)((repeated_count_current >> 8) & 0xFF);
		                msg_to_queue->Data[3] = (uint8_t)(repeated_count_current & 0xFF);
		                temp_data = (uint16_t)(HV_V*10);
		                msg_to_queue->Data[4] = temp_data >> 8;
		                msg_to_queue->Data[5] = temp_data & 0xFF;
		                temp_data = (uint16_t)(HV_I*100);
		                msg_to_queue->Data[6] = temp_data >> 8;
		                msg_to_queue->Data[7] = temp_data & 0xFF;

		                for (int i = 6; i < 8; i++) { msg_to_queue->Data[i] = 0x00; }

		                 // 6. 【原子操作】移动头指针，正式将消息放入队列
		                 g_tx_queue_head = next_head;
		             }
	}
	if(htim == &htim6)
	{
			/*高压电流 电压 低压DC电流 ADC DMA数据处理与单位转换*/
		  int temp_data[2];
		  temp_data[0] = (2048 - ADC1_RAW_data[0]) * 2;
		  temp_data[1] = (2048 - ADC1_RAW_data[1]) * 2;

		  HV_V = temp_data[0] * 1.00909423828125f;		//HV_V = (temp_data[0] / 4096) * 3.3f * 2 * 501 / 0.4f
		  HV_I = (temp_data[1] * 0.01007080078125);		//HV_I = (temp_data[1] / 4096) * 3.3 / 8 / 10 * 1000
		  DC_I = ADC2_RAW_data[0] * 0.0008056640625f;		//(ADC2_RAW_data[0] / 4096) * 3.3 / 200 / 0.005

		  EMA_DATA.sin_V = (ADC2_RAW_data[1]*0.0008056640625f);// 3.3/4096  磁栅尺位移传感器ADC采样与转换
		  EMA_DATA.cos_V = (ADC2_RAW_data[2]*0.0008056640625f);// 3.3/4096

		  float current_theta_degrees; // 存储当前计算出的角度
		  EMA_DATA.theta_radians = atan2f(EMA_DATA.sin_V - EMA_DATA.sin_offset, EMA_DATA.cos_V - EMA_DATA.cos_offset);
		  current_theta_degrees = EMA_DATA.theta_radians * (180.0f / M_PI);
		  EMA_DATA.theta_degrees = current_theta_degrees; // 更新结构体中的当前角度

		  if (EMA_DATA.theta_degrees < 0) {
			  EMA_DATA.theta_degrees += 360.0f;
		  }

		  // 角度回绕检测和圈数累计 // 需要一个阈值来判断是否发生了回绕，例如180度。 如果角度变化超过180度，则认为发生了一次回绕。
		  float delta_angle = EMA_DATA.theta_degrees - EMA_DATA.previous_theta_degrees;

 		  if (!EMA_DATA.first_calculation) { // 只有在不是第一次计算时才进行回绕检测
				  if (delta_angle > 180.0f) 		  { EMA_DATA.cycle_count--;}// 例如从 350 -> 10，实际是正转，但差值 < -180
					  else if (delta_angle < -180.0f) { EMA_DATA.cycle_count++;}// 反向回绕 (例如从 10 度跳到 350 度)
		  }
 		  else {
			  EMA_DATA.first_calculation = 0; // 清除首次计算标志
	      }


		  EMA_DATA.previous_theta_degrees = EMA_DATA.theta_degrees;// 更新上一时刻的角度
		  EMA_DATA.current_displacement_within_cycle_mm = (EMA_DATA.theta_degrees / 360.0f) * 2.0f;// 计算周期内位移 (0 到 2mm)

		  if(EMA_DATA.Min_Max_cal_status == false)// 计算累计位置
		  {
			  EMA_DATA.position_mm = (float)EMA_DATA.cycle_count * 2.0f + EMA_DATA.current_displacement_within_cycle_mm;
		  }
		  else if(EMA_DATA.Min_Max_cal_status == true)
		  {
			  EMA_DATA.position_mm = (float)EMA_DATA.cycle_count * 2.0f + EMA_DATA.current_displacement_within_cycle_mm - EMA_DATA.min_position_mm + 1.5f;
		  }

		  EMA_DATA.Loop_count++;
		  if(EMA_DATA.Loop_count == 10)//低速环路
		  {
			  EMA_DATA.speed = (EMA_DATA.position_mm - EMA_DATA.Last_position_mm)/0.001f;//计算速度 1khz循环执行周期
			  EMA_DATA.speed = 0.8f*EMA_DATA.speed + 0.2*EMA_DATA.Last_speed;
			  EMA_DATA.Last_speed = EMA_DATA.speed;
			  EMA_DATA.Last_position_mm = EMA_DATA.position_mm;//更新位置
		  }


        if (HV_I > SPIKE_CURRENT_THRESHOLD) { spike_count_in_window++; }// 1. 检查当前电流是否形成了一次尖峰
        window_sample_counter++;// 2. 采样点计数器自增

        // 3. 判断一个检查窗口是否已经结束
        if (window_sample_counter >= CHECK_WINDOW_SAMPLES)
        {
            if (spike_count_in_window > MAX_SPIKES_IN_WINDOW)// 窗口结束，进行判断
            {
                // 在过去的N个采样点中，尖峰次数过多，判定为故障！
                Motor_mode = MOTOR_OVER_HV_CURRENT; // 设置故障状态 报警代码2
                Close_output();
                DC_Power_CTR(false);
                Buzzer_ON;
            }
            window_sample_counter = 0;// 4. 重置计数器，为下一个检查窗口做准备
            spike_count_in_window = 0;
        }

//			Tx_A_buffer_20k.fdata[0] = HV_V;//V
//			Tx_A_buffer_20k.fdata[1] = HV_I;//mA
//			Tx_A_buffer_20k.fdata[2] = absolute_step_counter;//
//			Tx_A_buffer_20k.fdata[3] = EMA_DATA.position_mm;//mm
//			Tx_A_buffer_20k.fdata[4] = Motor_Position_Controller.TargetPos;//g
//			Tx_A_buffer_20k.fdata[5] = DC_I;//A
//
//			Tx_A_buffer_20k.tail[0] = 0x00;
//			Tx_A_buffer_20k.tail[1] = 0x00;
//			Tx_A_buffer_20k.tail[2] = 0x80;
//			Tx_A_buffer_20k.tail[3] = 0x7F;

//			if(dma_print_flag == 1)
//			{
//				if(g_uart_dma_transfer_complete == 1)
//				{
//					g_uart_dma_transfer_complete = 0;//设置为发送模式
//					HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&Tx_A_buffer_20k, sizeof(JustFloatFrame_t));
//				}
//			else
//			{
//				Buzzer_ON;
//			    printf("dma data error\r\n");
//			}
//			}
             //报警代码1
    		if(HV_V > MAX_HV_voltage)  { Motor_mode = MOTOR_OVER_HV_VOLTAGE;    Close_output();DC_Power_CTR(false);Buzzer_ON;}
   		    if(HV_I > MAX_HV_current * 3)  { Motor_mode = MOTOR_OVER_HV_CURRENT;    Close_output();DC_Power_CTR(false);Buzzer_ON;}
    		if(DC_I > MAX_DC_current)  { Motor_mode = MOTOR_OVER_DC_IN_CURRENT; printf("%.3f\r\n",DC_I);Close_output();DC_Power_CTR(false);Buzzer_ON;}

//    		if(Sin_Velocity_Flag == true)
//    		{
//    			Motor_mode = MOTOR_OPEN_VELOCITY;
//    			float temp_time = HAL_GetTick() * Omega_Sin_Velocity;
//    			speed_test = Max_Velocity * sin(temp_time);
//    			set_speed(speed_test);
//    		}

        // --- 1. 安全检查层：处理最高优先级的 IDLE 和 ERROR 状态 ---
        if (Motor_mode == MOTOR_IDLE || Motor_mode == MOTOR_ERROR || Motor_mode == MOTOR_OVER_HV_CURRENT || Motor_mode == MOTOR_OVER_HV_VOLTAGE || Motor_mode == MOTOR_OVER_DC_IN_CURRENT)
        {
            phase_increment = 0; // 强制速度为0，确保电机停止
        }
        else
        {
				if (Motor_mode == MOTOR_OPEN_REPEATED)// --- 1. 状态决策层：根据当前模式决定电机的目标和速度 ---
				{
					if (phase_increment == 0)// 在往复模式下，检查上一个移动是否已完成 (表现为电机已停止)
					{
						if (repeated_count_current >= repeated_count_total)// 检查总次数是否已完成
						{
							Motor_mode = MOTOR_IDLE; // 任务完成，切换到速度模式并保持静止
							HAL_TIM_Base_Stop(&htim16);
							Queue_Reply_Request(0x06, 0x10);//发送完成指令
						}
						else
						{
							if (absolute_step_counter == repeated_pos_A)// 任务未完成，决定下一个目标点
							{
								target_step_position = repeated_pos_B;// 当前在A点，下一个目标是B点
							}
							else // 当前在B点 (或者初始位置)
							{
								target_step_position = repeated_pos_A;// 下一个目标是A点
								if (absolute_step_counter == repeated_pos_B)// 完成一次 B->A 的移动，才算一个完整的往复周期
								{
									repeated_count_current++;
								}
							}
						}
					}
				}

			     // --- 2. 运动执行层：根据目标驱动电机 ---
				 if (Motor_mode == MOTOR_OPEN_POSITION || Motor_mode == MOTOR_OPEN_REPEATED || Motor_mode == MOTOR_SYNC_POSITION)
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
				 else if(Motor_mode == MOTOR_OPEN_VELOCITY)
				 {
					 phase_increment = velocity_mode_increment;
				 }
				 else if(Motor_mode == MOTOR_CLOSE_POSITION)
				 {
					 float output = P_Control_Compute(&Motor_Position_Controller, Motor_Position_Controller.TargetPos, EMA_DATA.position_mm);
					    if (output >= 0)
					    {
					        motor_direction = MOTOR_REVERSE;
					        float increment_f = (output / interrupt_freq_hz) * PHASE_2_32;
					        phase_increment = (uint32_t)increment_f;
					    }
					    else
					    {
					        motor_direction = MOTOR_FORWARD;
					        float increment_f = (-output / interrupt_freq_hz) * PHASE_2_32;
					        phase_increment = (uint32_t)increment_f;
					    }
				 }
				 else if(Motor_mode == MOTOR_CLOSE_FORCE)
				 {
					 float output = P_Control_Compute(&Motor_Force_Controller, Motor_Force_Controller.TargetPos, Force_Sensor1.weight_g);
					    if (output >= 0)
					    {
					        motor_direction = MOTOR_REVERSE;
					        float increment_f = (output / interrupt_freq_hz) * PHASE_2_32;
					        phase_increment = (uint32_t)increment_f;
					    }
					    else
					    {
					        motor_direction = MOTOR_FORWARD;
					        float increment_f = (-output / interrupt_freq_hz) * PHASE_2_32;
					        phase_increment = (uint32_t)increment_f;
					    }
				 }


        }

        // 1. 累加/累减相位 (无论何种模式，都基于最终计算出的 phase_increment)  // 根据方向执行累加或累减
        if (motor_direction == MOTOR_FORWARD) { phase_accumulator += phase_increment; }
           else 							  { phase_accumulator -= phase_increment; }

		// 2. 计算扇区 (0-5)  //将 0x00000000 ~ 0xFFFFFFFF 的范围等分成 6 份  // 方法：(phase_accumulator * 6) / 2^32  // 使用64位乘法和右移32位来实现，非常高效
		uint8_t new_step = (uint8_t)(((uint64_t)phase_accumulator * 6) >> 32);

        if (new_step > 5) new_step = 5; // 防止浮点数误差导致越界

        if(step != new_step)
        {
        	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

			if ( (step == 5 && new_step == 0) || (step == 0 && new_step == 5) )  // 【修正】同时判断正转(5->0)和反转(0->5)的周期结束点
			{
				if (hv_power_sample_count > 0)// 周期结束的计算逻辑本身是正确的
				{
					last_hv_average_power_cycle = hv_power_accumulator / hv_power_sample_count;// 计算并更新上个周期的平均功率
				}
				hv_power_accumulator = 0.0f;// 重置，为下个周期准备
				hv_power_sample_count = 0;
			}
            if (motor_direction == MOTOR_FORWARD) { absolute_step_counter++; }// 正转，计数值加1
                else                              { absolute_step_counter--; }// 反转，计数值减1

            step = new_step; // 更新当前步骤

            Update_output(step);
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
  MX_SPI1_Init();
  MX_TIM17_Init();
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

    	  Force_Sensor1.weight_proportion=86742;  // 电压值与重量变换比例，这个需要实际测试计算才能得到
    	  Force_Sensor1.weight_Zero_Data=0;   // 零值

  //  	    Force_sensor_init();
  //  	    weight_ad7190_conf();
  //
  //  	    HAL_Delay(500);
  //  	    Force_Sensor1.weight_Zero_Data = weight_ad7190_ReadAvg(6);
  ////  	    printf("zero:%ld\n",Force_Sensor1.weight_Zero_Data);
  //
  //  	  Force_Sensor1.RAW_Data=weight_ad7190_ReadAvg(1);
  //  	  Force_Sensor1.weight_g=(Force_Sensor1.RAW_Data-Force_Sensor1.weight_Zero_Data)*1000/Force_Sensor1.weight_proportion;

    	CAN_init();

      EMA_DATA.sin_offset = 1.65f;
      EMA_DATA.cos_offset = 1.65f;

      Motor_Position_Controller.Kp = 100;
      Motor_Position_Controller.MaxSpeed = 100;
      Omega_Sin_Velocity = 1 * 2 * M_PI * 0.001f;

      HAL_TIM_Base_Start_IT(&htim6);

      HAL_TIM_Base_Start_IT(&htim17);//CAN Heart

//      Motor_mode = MOTOR_OPEN_VELOCITY;
//      velocity_mode_increment = (uint32_t)(((uint64_t)100.0f * 0x100000000) / interrupt_freq_hz);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //阶跃信号
//	  if(HAL_GetTick() - last_time_ms <= 1000)
//	  {
//		  Motor_Position_Controller.TargetPos = 0.0f;
//	  }
//	  else if((HAL_GetTick() - last_time_ms <= 2000) && (HAL_GetTick() - last_time_ms >= 1000))
//	  {
//		  Motor_Position_Controller.TargetPos = 6.0f;
//	  }
//	  else
//	  {
//		  last_time_ms = HAL_GetTick();
//	  }
	  //正弦位置
//	  	  Motor_Position_Controller.TargetPos = (5 * sin(HAL_GetTick() * Omega_Sin_Velocity)) + 5;
//	  	  time_gap = HAL_GetTick() - last_time_ms;
//	  	  if(time_gap <= 500)
//	  	  {
//	  		  Motor_Position_Controller.TargetPos = time_gap * 0.02f;
//	  	  }
//	  	  else if((time_gap <= 1000) && (time_gap >= 500))
//	  	  {
//	  		  Motor_Position_Controller.TargetPos = 20 - (time_gap * 0.02f);
//	  	  }
//	  	  else
//	  	  {
//	  		  last_time_ms = HAL_GetTick();
//	  	  }
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
//		  printf("%.3f\r\n",Force_Sensor1.weight_g);
	  }


	      if (g_tx_queue_head != g_tx_queue_tail) // 1. 检查队列是否为空 (head 和 tail 是否相等)
	      {
	          if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0)// 2. 检查CAN硬件Tx FIFO是否空闲 (有空余等级)
	          {
	              // 注意：我们总是从 g_tx_queue[g_tx_queue_tail] 处取出消息 我们需要强制转换(cast)，因为 g_tx_queue 被声明为 volatile
	              FDCAN_TxHeaderTypeDef* tx_header = (FDCAN_TxHeaderTypeDef*)&g_tx_queue[g_tx_queue_tail].Tx_Header;
	              uint8_t* tx_data = (uint8_t*)g_tx_queue[g_tx_queue_tail].Data;

	              if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, tx_header, tx_data) == HAL_OK)// 4. 调用HAL库函数，将消息放入硬件发送FIFO
	              {
	                  g_tx_queue_tail = (g_tx_queue_tail + 1) & (TX_QUEUE_SIZE - 1);// 5. 【关键】发送成功，【原子操作】移动尾指针，完成出队
	              }
	              else
	              {
	                  printf("Can_Tx_error\r\n");
	              }
	          }
	           else
	          {
	              printf("Can fifo error\r\n");
	          }
	      }

	      Can_message_process();

//	      if(Motor_mode == MOTOR_CLOSE_POSITION)
//	      {
//	    	  printf("%.2f,%ld,%.2f,%.2f,%ld\r\n",speed_test,absolute_step_counter,EMA_DATA.position_mm,Motor_Position_Controller.TargetPos,HAL_GetTick() - last_time_ms);
//	      }
//	      else if(Motor_mode == MOTOR_CLOSE_FORCE)
//	      {
//	    	  printf("%.2f,%ld,%.2f,%.2f,%.2f\r\n",speed_test,absolute_step_counter,EMA_DATA.position_mm,Motor_Force_Controller.TargetPos,Force_Sensor1.weight_g);
//	      }
//	      else
//	      {
////              printf("step:%ld\r\n",absolute_step_counter);
//	      }

//    printf("%ld\r\n",absolute_step_counter);



//	  Force_Sensor1.RAW_Data=weight_ad7190_ReadAvg(1);
//	  Force_Sensor1.weight_g=(Force_Sensor1.RAW_Data-Force_Sensor1.weight_Zero_Data)*1000/Force_Sensor1.weight_proportion;
//	  printf("%.3f\r\n",Force_Sensor1.weight_g);
//	  printf("%ld,%ld,%.3f,%.3f\r\n",ADC2_RAW_data[1],ADC2_RAW_data[2],EMA_DATA.theta_degrees,EMA_DATA.position_mm);
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
  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim6.Init.Period = 999;
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
  htim7.Init.Prescaler = 84;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 99;
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
  htim16.Init.Period = 4999;
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
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 16999;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 9999;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

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
  huart1.Init.BaudRate = 115200;
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
  HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin|Power_ON_Pin|CH1__CTR_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : SPI1_CS_Pin CH1__CTR_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin|CH1__CTR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Power_ON_Pin */
  GPIO_InitStruct.Pin = Power_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Power_ON_GPIO_Port, &GPIO_InitStruct);

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
