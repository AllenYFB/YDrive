# YDrive

YDrive 是一个从零复刻 ODrive 控制思路的 STM32F405 电机控制工程。目标不是逐行照搬 ODrive 官方 C++ 固件，而是以 ODrive v3 的硬件时序为参考，用 C 语言、STM32CubeMX、CMake、VSCode 和 FreeRTOS 一步一步实现单电机 FOC 控制器。

当前阶段先做 M0 单电机，不启用 TIM8/M1。高速 PWM、ADC 采样和后续 FOC 电流环围绕 TIM1/ADC injected 同步时序实现；FreeRTOS 只负责低速任务，例如 USB、状态机、参数管理和调试输出。

## 目录

```text
YDrive/
  Board/                         STM32CubeMX/CMake 工程目录
  Docs/                          文档
  .vscode/                       VSCode 任务配置
  README.md                      项目首页说明
```

真正的 CMake 工程在：

```powershell
C:\Users\mrg\Desktop\PMSMFocLib\YDrive\Board
```

不要在仓库根目录直接执行 `cmake --preset Debug`，否则 CMake 找不到 `CMakePresets.json`。

## 当前硬件配置基准

硬件配置参考原版 ODrive v3 CubeMX 工程：

```text
C:\Users\mrg\Desktop\PMSMFocLib\odrive代码分析合集\odrive_原代码注释\Odrive_vscode\Firmware\Board\v3\Odrive.ioc
```

YDrive 当前使用：

- MCU：`STM32F405RGTx`
- CubeMX：`6.16.0`
- Toolchain：`CMake + Ninja + arm-none-eabi-gcc`
- RTOS：`FreeRTOS CMSIS_V2`
- USB：`USB CDC`
- PWM：`TIM1` 控制 M0 三相桥
- ADC：`ADC1/ADC2/ADC3 injected`，由 `TIM1_TRGO` 触发
- 安全脚：`PB12 EN_GATE` 默认保持 LOW
- 故障脚：`PD2 nFAULT` 输入上拉

## 当前 TIM1 与 ADC 时序

M0 三相 PWM 使用 `TIM1`：

| 信号 | 引脚 | 功能 |
|---|---|---|
| M0_AH | PA8 | TIM1_CH1 |
| M0_BH | PA9 | TIM1_CH2 |
| M0_CH | PA10 | TIM1_CH3 |
| M0_AL | PB13 | TIM1_CH1N |
| M0_BL | PB14 | TIM1_CH2N |
| M0_CL | PB15 | TIM1_CH3N |

TIM1 关键配置：

```text
Counter Mode      = Center Aligned 3
Period            = 3500
RepetitionCounter = 2
PWM Mode          = PWM2
TRGO              = Update Event
DeadTime          = 20
```

ADC injected 配置：

| 信号 | 引脚 | ADC | 触发 |
|---|---|---|---|
| VBUS_S | PA6 | ADC1_CHANNEL_6 | TIM1_TRGO Rising |
| M0_IB | PC0 | ADC2_CHANNEL_10 | TIM1_TRGO Rising |
| M0_IC | PC1 | ADC3_CHANNEL_11 | TIM1_TRGO Rising |

`Rising` 指 ADC 检测 `TIM1_TRGO` 触发脉冲的上升沿，不是 TIM1 只能在向上计数时采样。不要改成 both edges，电流采样需要固定、可预测的采样点。

## 编译

环境配置和详细编译说明见：

[Docs/environment_and_build.md](Docs/environment_and_build.md)

最常用命令：

```powershell
cd C:\Users\mrg\Desktop\PMSMFocLib\YDrive\Board
cmake --fresh --preset Debug
cmake --build --preset Debug
```

编译输出：

```text
Board/build/Debug/YDrive.elf
Board/build/Debug/YDrive.bin
Board/build/Debug/YDrive.hex
Board/build/Debug/YDrive.map
```

## 当前已完成

当前固件已经完成最小启动验证代码：

- 工程可编译
- 自动生成 `.elf/.bin/.hex`
- FreeRTOS defaultTask 创建
- `StartDefaultTask()` 中初始化 USB CDC
- USB CDC 输出 `YDrive boot`
- 每 1 秒读取一次 `nFAULT`
- 根据 `nFAULT` 打印 `nFAULT=1 OK` 或 `nFAULT=0 FAULT`
- 循环中持续保持 `EN_GATE = LOW`

这一步只验证基础系统，不启动 PWM 功率输出，不拉高 `EN_GATE`。

## 文档

- [Docs/environment_and_build.md](Docs/environment_and_build.md)：环境配置与编译说明
- [Docs/ydrive_cubemx_and_plan.md](Docs/ydrive_cubemx_and_plan.md)：CubeMX 配置与复刻计划

## 后续开发计划

### Step 1：板级启动验证

目标：

- 下载进板子
- 调试能进入 `main()`
- FreeRTOS defaultTask 跑起来
- USB CDC 能看到 `YDrive boot`
- USB CDC 能枚举
- `nFAULT` 读取正常
- `EN_GATE` 始终保持 LOW

### Step 2：TIM1 与 ADC injected 空载验证

目标：

- 启动 ADC injected
- 启动 TIM1 基准计数和 TRGO
- 暂不启动功率输出
- 验证 ADC 是否被 `TIM1_TRGO` 触发
- 读取 `VBUS_S / M0_IB / M0_IC`
- 统计采样频率和原始 ADC 稳定性

### Step 3：电流零偏校准

目标：

- 无电流状态下多次采集 `M0_IB / M0_IC`
- 计算 ADC offset
- 建立 current sense 数据结构
- 检查零偏稳定性

### Step 4：FOC 数学模块

实现：

- Clarke
- Park
- inverse Park
- SVPWM
- duty 限幅
- TIM1 CCR 更新

### Step 5：开环电压/速度控制

目标：

- 低压、小电流、空载开环转动
- 验证三相 PWM 顺序
- 验证相序和母线电压缩放
- 建立 `arm()` / `disarm()` 安全流程

### Step 6：电机参数校准

目标：

- 测量相电阻
- 测量相电感
- 判断相序
- 出错时立即关闭 PWM 和 `EN_GATE`

### Step 7：编码器

目标：

- SPI 读取磁编码器
- 位置 unwrap
- PLL 速度估计
- encoder offset calibration
- 建立机械角度到电角度映射

### Step 8：电流闭环

目标：

- ADC 实测相电流
- Id/Iq PI 控制
- 输出 Vd/Vq
- 接入 SVPWM
- 实现电流限幅和保护

### Step 9：速度环、位置环与通信

目标：

- 速度环
- 位置环
- trap trajectory
- USB 命令协议，后续可选 UART 命令协议
- 参数保存
- 错误码管理

## 当前下一步

下一步先做 Step 1 的真实板级验证：

1. 下载 `Board/build/Debug/YDrive.elf` 或 `YDrive.bin`
2. 调试确认进入 `main()`
3. 确认进入 `StartDefaultTask()`
4. USB CDC 串口确认 `YDrive boot`
5. Windows 设备管理器确认 USB CDC 枚举
6. 测量或观察 `PB12 EN_GATE` 保持 LOW
7. 改变/观察 `PD2 nFAULT` 状态，确认 USB CDC 输出跟随变化

Step 1 完成后，再开始 Step 2：只启动 TIM1_TRGO 和 ADC injected，仍然不打开功率输出。

