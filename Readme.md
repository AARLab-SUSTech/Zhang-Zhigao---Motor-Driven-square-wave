STM32G4 Pulse Motor Control System 

##  项目简介 (Overview)
本项目是一个基于STM32G431微控制器的高性能、高压静电薄膜电机驱动与控制系统。
系统采用了（APP - MID - BSP)架构，实现了业务逻辑、中间件算法与底层硬件的分离。

## 硬件平台 (Hardware Platform)
* **MCU:** STMicroelectronics STM32G431CBTX
* **外设依赖:** 高级定时器 (PWM 六步换向)、CAN总线、USART DMA、ADC、外部 ADC (AD7190 用于力传感器)
* **开发工具链:** STM32CubeIDE

## 软件架构设计 (Software Architecture)
项目严格遵循垂直单向依赖原则：`APP` -> `MID` -> `BSP`，严禁越级穿透。

### 1. 顶层应用层 (APP Layer)
负责顶层业务逻辑调度、状态机流转与系统级保护，完全脱离硬件。
* `App_EMA` / `App_Control`: 电机运动规划与核心状态机 (IDLE, READY, ERROR 等)。
* `App_Fault`: **全局故障管理中心**。采用位域独立管理过压、过流、通信掉线等异常，提供统一的硬件急停与上报机制。
* `App_Led`: 非阻塞式 LED 状态机，通过时序闪烁编码指示当前系统运行状态或故障码。
* `App_Can` / `App_Usart`: 顶层通信业务调度。

### 2. 中间件层 (MID Layer)
负责数据处理、协议解析与控制算法封装，承上启下。
* `Mid_Command_Can` / `Mid_Command_Usart`: 通信协议解析、CRC校验与数据封包。
* `Mid_Control`: 闭环 PID 控制器与核心算法计算。
* `Mid_Position_Sensor` / `Mid_Adc`: 传感器原始数据滤波、标定与物理量转换 (转换为真实的角度、电流值)。

### 3. 板级支持包 (BSP Layer)
最底层的硬件驱动层，直接操作 STM32 寄存器或 HAL 库，对外提供极简的 API 接口。
* `Bsp_Control`: 电源时序控制、PWM 六步换向底层驱动与紧急封锁 (Close Output)。
* `Bsp_Can` / `Bsp_Usart`: CAN/串口底层收发接口与中断回调包装。
* `Bsp_Ad7190` / `Bsp_Force_Sensor`: 外部高精度传感器 SPI通信底层时序。
* `Bsp_Led`: 极简的 GPIO 亮灭控制。

## 🛡️ 核心安全机制 (Safety Features)
1.  * **IWDG (独立看门狗):** 部署于主循环，提供系统级死机兜底重启。
2.  **极速过流急停:** 中断级硬件状态监控，一旦触发阈值，微秒级切断高压 DCDC 与 PWM 桥臂输出。

---
*Maintained by Letian && Gemini.*



























































