# YDrive CubeMX 配置与复刻计划

本文记录当前 YDrive 工程的 CubeMX 选择，以及后续从零复刻 ODrive 思路的实现顺序。目标是让后人可以按同一条路线重新生成工程、编译、下载、调试，并逐步补齐 FOC、电机校准、编码器、闭环控制和通信。

## 1. 工程目标

YDrive 是一个基于 STM32F405 的单电机 ODrive 风格控制器实验工程。

当前阶段目标：

- 使用 STM32CubeMX 6.16.0 生成基础工程
- 使用 CMake + VSCode 编译调试
- 使用 FreeRTOS 运行低速任务
- 使用 USB CDC 和 UART4 做调试通信
- 先只做 M0 单电机，暂不启用 TIM8/M1
- 高速 FOC 后续放在 TIM1/ADC 同步时序中，不放进普通 FreeRTOS 任务

硬件配置以原版 ODrive v3 工程为参考：

```text
C:\Users\mrg\Desktop\PMSMFocLib\odrive代码分析合集\odrive_原代码注释\Odrive_vscode\Firmware\Board\v3\Odrive.ioc
```

软件编写顺序以 `readme.md` 中的 YDrive 开发计划为主。

## 2. 当前 CubeMX 基础选择

### 2.1 MCU 与工程

- MCU：`STM32F405RGTx`
- Package：`LQFP64`
- CubeMX：`6.16.0`
- Firmware Package：`STM32Cube FW_F4 V1.28.3`
- Toolchain：`CMake`
- Debug：`Serial Wire`
- SYS Timebase：`TIM14`
- RTOS：`FreeRTOS CMSIS_V2`

### 2.2 时钟

- HSE：外部晶振 8 MHz
- SYSCLK：168 MHz
- HCLK：168 MHz
- APB1：42 MHz
- APB1 Timer Clock：84 MHz
- APB2：84 MHz
- APB2 Timer Clock：168 MHz
- USB 48 MHz：PLLQ = 7，输出 48 MHz

这个配置保证 TIM1 运行在 168 MHz，并且 USB FS CDC 能正常枚举。

## 3. 当前引脚配置

### 3.1 M0 三相 PWM

M0 使用 `TIM1`：

| 信号 | 引脚 | 定时器功能 |
|---|---|---|
| M0_AH | PA8 | TIM1_CH1 |
| M0_BH | PA9 | TIM1_CH2 |
| M0_CH | PA10 | TIM1_CH3 |
| M0_AL | PB13 | TIM1_CH1N |
| M0_BL | PB14 | TIM1_CH2N |
| M0_CL | PB15 | TIM1_CH3N |

当前只配置 TIM1，不配置 TIM8。TIM8 是原版 ODrive M1 第二电机用的，YDrive 第一阶段先不启用。

### 3.2 ADC 电流与母线采样

当前 M0 使用三个 ADC injected 通道：

| 信号 | 引脚 | ADC |
|---|---|---|
| VBUS_S | PA6 | ADC1_CHANNEL_6 |
| M0_IB | PC0 | ADC2_CHANNEL_10 |
| M0_IC | PC1 | ADC3_CHANNEL_11 |

Injected 触发配置：

```text
ADC1 injected: TIM1_TRGO + Rising
ADC2 injected: TIM1_TRGO + Rising
ADC3 injected: TIM1_TRGO + Rising
```

注意：`Rising` 指的是 ADC 检测 `TIM1_TRGO` 触发脉冲的上升沿，不是 TIM1 只能在向上计数时采样。只要 TIM1 产生真正的 Update Event/TRGO，ADC 就会被触发。

不建议改成上下沿触发。FOC 电流采样需要固定、可预测的采样点；上下沿会让采样点变多且时刻不同，更容易采到开关噪声。

### 3.3 TIM1 PWM 与 TRGO

TIM1 当前关键配置：

```text
Counter Mode       = Center Aligned 3
Period             = 3500
RepetitionCounter  = 2
PWM Mode           = PWM2
Pulse CH1/2/3      = 1750
DeadTime           = 20
TRGO               = Update Event
OffStateRunMode    = Enable
OffStateIDLEMode   = Enable
AutomaticOutput    = Disable
```

时序理解：

```text
TIM1 中心对齐计数：
0 -> ARR -> 0 -> ARR -> 0

原始 update 点大约出现在 ARR 顶部和 0 底部。
RepetitionCounter = 2 表示每 3 个 update 请求才产生一次真正 Update Event。
TIM1_TRGO = Update Event。
ADC injected 在 TIM1_TRGO 的 Rising edge 开始采样。
```

所以 PWM 可以连续运行，ADC/控制计算按较低的同步频率运行。CPU 更新的是 TIM1 的 CCR 占空比寄存器，PWM 硬件会继续输出，不会因为 CPU 没进入中断就停一下。

### 3.4 安全与状态 GPIO

| 信号 | 引脚 | 配置 |
|---|---|---|
| EN_GATE | PB12 | GPIO Output，默认 LOW |
| nFAULT | PD2 | GPIO Input，Pull-up |

启动阶段必须保持：

```text
EN_GATE = LOW
```

只有完成电流零偏校准、驱动状态确认、PWM 安全初始化后，后续 `arm()` 流程才允许打开 EN_GATE。

### 3.5 通信

当前通信配置：

| 功能 | 引脚/外设 |
|---|---|
| UART4_TX | PA0 |
| UART4_RX | PA1 |
| USB FS DM | PA11 |
| USB FS DP | PA12 |
| USB_DEVICE | CDC |

UART4 使用 115200 8N1，作为早期启动日志和调试串口。

USB CDC 在 `StartDefaultTask()` 里调用 `MX_USB_DEVICE_Init()` 初始化。USB 能枚举，说明 FreeRTOS 调度器和 defaultTask 已经跑起来。

### 3.6 SPI3 预留

当前 SPI3 已配置，后续用于磁编码器：

| 信号 | 引脚 |
|---|---|
| SPI3_SCK | PB3 |
| SPI3_MISO | PC11 |
| SPI3_MOSI | PC12 |

后续可继续补充 M0 编码器 CS、AS5047P/MT6701 等驱动。

## 4. 当前已完成的启动验证代码

当前 `Core/Src/freertos.c` 的 `StartDefaultTask()` 已完成最小启动验证：

- 调用 `MX_USB_DEVICE_Init()`
- UART4 打印 `YDrive boot`
- 每 1 秒读取一次 `nFAULT`
- 打印 `nFAULT=1 OK` 或 `nFAULT=0 FAULT`
- 循环中持续保持 `EN_GATE = LOW`

这一阶段不要启动 PWM 输出，不要拉高 EN_GATE，不要接电机做功率实验。

## 5. 推荐复刻步骤

### Step 1：基础工程跑通

目标：

- CMake 能编译
- ST-Link 能下载
- 调试能进 `main()`
- FreeRTOS defaultTask 跑起来
- UART4 能打印 `YDrive boot`
- USB CDC 能枚举
- 能读取 `nFAULT`
- `EN_GATE` 始终保持 LOW

验证通过后再进入下一步。

### Step 2：TIM1 与 ADC injected 空载验证

目标：

- 启动 ADC injected
- 启动 TIM1 基准计数和 TRGO
- 暂不启动功率输出
- 观察 ADC 是否被 TIM1_TRGO 触发
- 读取 `VBUS_S / M0_IB / M0_IC`
- 统计采样频率是否符合预期
- 检查 ADC 原始值稳定性

此阶段仍然保持 `EN_GATE = LOW`。

### Step 3：电流零偏校准

目标：

- 在无电流状态下采集多次 `M0_IB / M0_IC`
- 计算 ADC offset
- 保存到 RAM 中的 motor/current sense 结构体
- 检查 offset 是否稳定

通过后，才允许进入低压开环测试。

### Step 4：FOC 数学基础

实现：

- Clarke 变换
- Park 变换
- inverse Park
- SVPWM
- duty 限幅
- TIM1 CCR 更新函数

先用固定角度和固定 `Vdq` 做离线计算验证，再接入 TIM1。

### Step 5：开环电压/速度控制

目标：

- 小电压 `Vq` 开环转动
- 验证三相 PWM 输出顺序
- 验证相序
- 验证母线电压缩放
- 验证 `EN_GATE` arm/disarm 流程

这一阶段必须限流、低压、空载，先不做闭环电流。

### Step 6：电阻、电感、相序校准

目标：

- 测量相电阻
- 测量相电感
- 判断相序
- 建立电机参数结构体
- 出错时立即 disarm

### Step 7：编码器驱动与 offset 校准

目标：

- SPI 读取磁编码器
- 实现位置 unwrap
- 实现 PLL 速度估计
- 校准 encoder offset
- 建立电角度与机械角度映射

### Step 8：电流闭环

目标：

- 使用 ADC 实测电流
- 实现 Id/Iq PI 控制
- 输出 Vd/Vq
- 接入 SVPWM
- 实现电流限幅和过流保护

高频电流环必须与 TIM1/ADC 同步，不放进普通 FreeRTOS task。

### Step 9：速度环和位置环

目标：

- 速度 PI
- 位置 P/PI
- 限速、限流、限位置
- trap trajectory 轨迹规划
- axis 状态机

### Step 10：通信与参数保存

目标：

- USB CDC 命令协议
- UART 调试命令
- 参数读写
- Flash 保存配置
- 错误码查询与清除

### Step 11：保护和工程化

目标：

- nFAULT 处理
- 过流、过压、欠压、过温保护
- watchdog
- 状态机错误恢复
- 单元测试或离线数学测试
- 文档补齐

## 6. 当前注意事项

- 不要把 `ADC injected edge` 改成 both edges，保持 Rising。
- 不要在验证前拉高 `EN_GATE`。
- 不要让 FreeRTOS 普通任务跑 16 kHz 电流环。
- CubeMX 重新生成后，确认 `Core/Src/freertos.c` 中用户代码区仍保留启动验证逻辑。
- 如果 GCC 链接脚本报 `READONLY` 相关错误，检查 `STM32F405XX_FLASH.ld` 中 `.ARM.extab/.ARM/.init_array` 等段属性。
- `YDrive/build/` 是本地构建产物，不上传到 git。

