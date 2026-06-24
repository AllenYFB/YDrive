# YDrive 开发计划

YDrive 是一个从零开始复刻 ODrive 控制思想的 C 语言电机控制工程。目标不是逐行照搬 ODrive 官方 C++ 固件，而是以 ODrive v3 的 CubeMX 硬件配置为基准，在 VSCode 中编译、下载、调试，并使用 FreeRTOS 管理低速任务，手写 FOC、电机校准、编码器、闭环控制、参数保存和通信协议。

硬件配置唯一基准：

- `C:\Users\mrg\Desktop\PMSMFocLib\odrive代码分析合集\odrive_原代码注释\Odrive_vscode\Firmware\Board\v3\Odrive.ioc`

本 README 中的阶段计划只规定 YDrive 的软件编写顺序；CubeMX 外设、引脚、时钟、DMA、NVIC 等硬件配置均以 `Board\v3\Odrive.ioc` 为准。

参考工程：

- `ODrive移植keil/2_点亮led`
- `ODrive移植keil/3_USB转串口和快速正弦余弦运算`
- `ODrive移植keil/4_TIM1和ADC电流采样`
- `ODrive移植keil/5_开环速度控制`
- `ODrive移植keil/6_测量电阻电感`
- `ODrive移植keil/7_插值算法和偏置校准`
- `ODrive移植keil/8_闭环控制`
- `ODrive移植keil/9_抗齿槽效应算法`

## 1. 总体思路

CubeMX 工程不重新自由选择外设，而是迁移或复刻 `Board\v3\Odrive.ioc` 的硬件配置。软件代码在此硬件基准上分层实现：

硬件层来自 `Odrive.ioc`：

- STM32F405RGTx / LQFP64
- RCC 时钟树
- GPIO 引脚定义
- TIM1 / TIM8 双电机三相 PWM
- ADC1 / ADC2 / ADC3 电流、母线电压、温度采样
- DMA
- SPI3 编码器通信
- TIM3 / TIM4 增量编码器接口
- TIM5 GPIO 捕获
- UART4 调试串口
- USB CDC 虚拟串口
- CAN1 通信
- FreeRTOS

YDrive 手写业务代码：

- `board`：硬件抽象层
- `foc`：Clarke/Park/SVPWM/电流环
- `motor`：电机参数、校准、电流限制、错误处理
- `encoder`：编码器读取、PLL 插值、偏置校准
- `controller`：位置环、速度环、力矩环、输入模式
- `axis`：状态机
- `comm`：USB/UART/CAN 命令协议
- `storage`：Flash 参数保存
- `anticogging`：抗齿槽补偿
- `utils`：数学工具和通用函数

核心原则：

- 高频 FOC 必须和 TIM1/ADC 同步，放在定时器或 ADC 中断回调里。
- FreeRTOS 不直接跑电流环，只跑通信、状态机、监控、参数保存等低速任务。
- 控制算法和硬件驱动分层，后续换板子或换编码器时不影响核心算法。

## 2. 推荐目录结构

```text
YDrive/
  README.md
  YDrive.ioc
  Core/
  Drivers/
  Middlewares/
  App/
    board/
      board_pwm.c
      board_pwm.h
      board_adc.c
      board_adc.h
      board_spi.c
      board_spi.h
      board_uart.c
      board_uart.h
      board_usb.c
      board_usb.h
      board_flash.c
      board_flash.h
    foc/
      foc.c
      foc.h
      svm.c
      svm.h
    motor/
      motor.c
      motor.h
    encoder/
      encoder.c
      encoder.h
      encoder_as5047p.c
      encoder_mt6701.c
    controller/
      controller.c
      controller.h
      trap_traj.c
      trap_traj.h
    axis/
      axis.c
      axis.h
    comm/
      command.c
      command.h
      usb_comm.c
      uart_comm.c
      can_comm.c
    storage/
      config.c
      config.h
      flash_storage.c
      flash_storage.h
    anticogging/
      anticogging.c
      anticogging.h
    utils/
      ydrive_math.c
      ydrive_math.h
      ydrive_types.h
      error.h
```

## 3. 硬件配置基准

CubeMX 使用 `v6.16.0`。打开或迁移工程时，以 ODrive 原工程的 `.ioc` 为准：

```text
C:\Users\mrg\Desktop\PMSMFocLib\odrive代码分析合集\odrive_原代码注释\Odrive_vscode\Firmware\Board\v3\Odrive.ioc
```

不要按本 README 重新自由选择引脚；本节只把 `.ioc` 的关键硬件配置摘出来，方便核对。

### 3.1 MCU 和工程

- MCU：`STM32F405RGTx`
- Package：`LQFP64`
- Toolchain：`Makefile`
- Debug：`Serial Wire`
- HSE：`8 MHz`
- SYSCLK：`168 MHz`
- HCLK：`168 MHz`
- APB1：`42 MHz`
- APB1 Timer：`84 MHz`
- APB2：`84 MHz`
- 原 `.ioc` 使用的 CubeMX 较旧，CubeMX 6.16.0 打开时如果提示迁移，先备份再迁移。

### 3.2 用户常量

`.ioc` 中已有这些常量，软件代码应沿用：

```text
TIM_1_8_CLOCK_HZ        = 168000000
TIM_1_8_PERIOD_CLOCKS   = 3500
TIM_1_8_DEADTIME_CLOCKS = 20
TIM_APB1_CLOCK_HZ       = 84000000
TIM_APB1_PERIOD_CLOCKS  = 4096
TIM_APB1_DEADTIME_CLOCKS= 40
TIM_1_8_RCR             = 2
```

### 3.3 PWM

M0 使用 `TIM1`：

- `PA8`：`M0_AH` / TIM1_CH1
- `PA9`：`M0_BH` / TIM1_CH2
- `PA10`：`M0_CH` / TIM1_CH3
- `PB13`：`M0_AL` / TIM1_CH1N
- `PB14`：`M0_BL` / TIM1_CH2N
- `PB15`：`M0_CL` / TIM1_CH3N
- Counter：`CENTERALIGNED3`
- PWM mode：`PWM2`
- Period：`TIM_1_8_PERIOD_CLOCKS`
- DeadTime：`TIM_1_8_DEADTIME_CLOCKS`
- RepetitionCounter：`TIM_1_8_RCR`
- TRGO：`UPDATE`

M1 使用 `TIM8`：

- `PC6`：`M1_AH` / TIM8_CH1
- `PC7`：`M1_BH` / TIM8_CH2
- `PC8`：`M1_CH` / TIM8_CH3
- `PA7`：`M1_AL` / TIM8_CH1N
- `PB0`：`M1_BL` / TIM8_CH2N
- `PB1`：`M1_CL` / TIM8_CH3N
- 其余 PWM 关键参数与 TIM1 对齐。

### 3.4 ADC 和电流采样

`.ioc` 配置了 ADC1/ADC2/ADC3，注入组由 `TIM1_TRGO` 触发：

- `PA6`：`VBUS_S` / ADC_CHANNEL_6
- `PC0`：`M0_IB` / ADC_CHANNEL_10
- `PC1`：`M0_IC` / ADC_CHANNEL_11
- `PC3`：`M1_IB` / ADC_CHANNEL_13
- `PC2`：`M1_IC` / ADC_CHANNEL_12
- `PA4`：`M1_TEMP`
- `PA5`：`AUX_TEMP`
- `PC5`：`M0_TEMP`
- ADC 分频：`PCLK / 4`
- Resolution：`12 bit`
- SamplingTime：`3 cycles`
- ADC1 DMA：`DMA2_Stream0`，circular，halfword

### 3.5 编码器和通信

- SPI3 磁编码器：`PC10 SCK`、`PC11 MISO`、`PC12 MOSI`
- `PC13`：`M0_nCS`
- `PC14`：`M1_nCS`
- SPI3：Master、16 bit、MSB first、prescaler 16、phase 2edge
- TIM3 增量编码器：`PB4 M0_ENC_A`、`PB5 M0_ENC_B`
- TIM4 增量编码器：`PB6 M1_ENC_A`、`PB7 M1_ENC_B`
- `PC9`：`M0_ENC_Z`
- `PC15`：`M1_ENC_Z`
- USB FS CDC：`PA11 DM`、`PA12 DP`
- CAN1 已启用，500 kbit/s 配置来自 `.ioc`
- UART4 已启用，用作调试串口，不是 USART2

### 3.6 GPIO 和安全引脚

- `PB12`：`EN_GATE`，栅极驱动使能输出
- `PD2`：`nFAULT`，故障输入，上拉
- `PA0` 到 `PA3`、`PB2`、`PB3`、`PC4`、`PA15` 等保留为 GPIO/辅助功能，按 `.ioc` 标签使用。

### 3.7 PWM 和 ADC 关键要求

PWM 和 ADC 是整个工程的地基：

- 高频控制以 TIM1/TIM8 与 ADC 注入采样同步为核心。
- ADC 采样点要避开开关噪声，保持 `.ioc` 的触发关系。
- PWM 默认不输出，只有 `arm()` 后才打开 `EN_GATE` 和 PWM。
- 出错时立即 `disarm()`，关闭 PWM 输出和 `EN_GATE`。
- 先实现 M0 单轴，跑通后再扩展 M1。

## 4. FreeRTOS 任务划分

高频控制不做成普通任务：

```text
TIM1/ADC ISR:
  读取 M0 ADC 电流
  电流零点校准
  FOC current loop
  更新 TIM1 CCR
```

FreeRTOS 任务建议：

```text
AxisTask 1kHz:
  run_state_machine_loop()
  处理校准流程
  处理闭环状态

CommTask:
  USB/UART/CAN 收包
  命令解析
  参数读写

MonitorTask:
  vbus 检查
  错误检查
  温度检查
  心跳状态

StorageTask:
  Flash 保存
  Flash 擦除
  参数 CRC 校验

LedTask:
  运行状态指示
```

## 5. 分阶段实现计划

### 阶段 0：工程骨架

目标：基于 `Board\v3\Odrive.ioc` 建立 YDrive 工程，确认 CubeMX 6.16.0、VSCode、Makefile 工具链可用。

任务：

- [ ] 复制或另存 `Board\v3\Odrive.ioc` 为 `YDrive.ioc`
- [ ] 用 CubeMX 6.16.0 打开并迁移 `.ioc`
- [ ] 核对 MCU、时钟、TIM1/TIM8、ADC1/2/3、SPI3、USB、CAN1、UART4、FreeRTOS 配置没有被迁移过程改乱
- [ ] 生成 Makefile 工程
- [ ] 配置 VSCode 编译任务
- [ ] 配置 VSCode 下载/调试
- [ ] 建立 `App/` 目录
- [ ] 确认 `build / flash / debug` 全流程可用

验收：

- [ ] 可以成功编译
- [ ] 可以下载到板子
- [ ] 可以打断点单步调试
- [ ] CubeMX 再生成一次代码后，用户代码不丢失

### 阶段 1：启动、时间基准、UART4

参考：`2_点亮led`

目标：跑通最小系统。ODrive v3 `.ioc` 中 `PD2` 是 `nFAULT` 输入，不按普通 LED 输出使用。

任务：

- [ ] `micros()` / `millis()` 时间基准
- [ ] UART4 printf
- [ ] FreeRTOS 默认任务运行
- [ ] 读取 `nFAULT`
- [ ] 控制 `EN_GATE` 默认关闭
- [ ] 启动信息打印

验收：

- [ ] UART4 打印启动信息
- [ ] FreeRTOS tick 正常
- [ ] `nFAULT` 输入状态可读
- [ ] 上电默认 `EN_GATE` 关闭

### 阶段 2：USB CDC、命令框架、数学工具

参考：`3_USB转串口和快速正弦余弦运算`

目标：建立通信入口和基础数学函数。

任务：

- [ ] USB CDC 虚拟串口
- [ ] UART4 接收
- [ ] 环形缓冲区
- [ ] 命令解析框架
- [ ] `sin/cos` 快速计算
- [ ] `clamp`
- [ ] `wrap_pm_pi`
- [ ] `wrap_2pi`
- [ ] `SVM` 基础函数

验收：

- [ ] USB CDC 能收发
- [ ] UART4 能收发
- [ ] 发送 `help` 可返回命令列表
- [ ] 数学函数可单元测试或串口打印验证

### 阶段 3：M0 TIM1 PWM 和 ADC 电流采样

参考：`4_TIM1和ADC电流采样`

目标：先跑通 M0 单轴三相 PWM 和同步 ADC 采样。M1/TIM8 暂时只保持初始化正确，不先参与控制。

任务：

- [ ] TIM1 中心对齐 PWM，使用 `.ioc` 中 M0 引脚
- [ ] TIM1 互补输出
- [ ] TIM1 死区配置
- [ ] PWM enable/disable 接口
- [ ] `EN_GATE` 控制接口
- [ ] ADC 采样 M0 相电流
- [ ] ADC 采样 `VBUS_S`
- [ ] 电流零点校准
- [ ] ADC 回调里读取采样结果
- [ ] 串口/USB 打印 ADC 原始值

验收：

- [ ] 示波器看到 M0 正确三相 PWM
- [ ] 示波器看到 M0 互补 PWM 死区
- [ ] 母线电压读数合理
- [ ] 空载电流零点稳定
- [ ] 未 `arm()` 时 PWM 和 `EN_GATE` 均保持安全关闭

### 阶段 4：开环速度控制

参考：`5_开环速度控制`

目标：不依赖编码器，让电机小电压开环转动。

任务：

- [ ] 实现 `FOC_voltage()`
- [ ] 实现 SVPWM 占空比更新
- [ ] 实现开环电角度累加
- [ ] 实现 `open_loop_controller_update()`
- [ ] 串口命令设置开环速度
- [ ] 串口命令设置开环电压

验收：

- [ ] 电机可低速平稳开环转动
- [ ] 改变速度命令后方向和速度变化正确
- [ ] 电压限制有效
- [ ] 出错后能关闭 PWM

### 阶段 5：电阻、电感测量和电流环参数

参考：`6_测量电阻电感`

目标：实现 ODrive 风格 motor calibration。

任务：

- [ ] `motor_config`
- [ ] 相电阻测量
- [ ] 相电感测量
- [ ] calibration current 配置
- [ ] resistance calib max voltage 配置
- [ ] 电流环 PI 参数计算
- [ ] 错误码机制
- [ ] `AXIS_STATE_MOTOR_CALIBRATION`

验收：

- [ ] 能测出合理相电阻
- [ ] 能测出合理相电感
- [ ] 电流环参数不发散
- [ ] 异常电机/未接电机能报错

### 阶段 6：编码器读取、PLL 插值、偏置校准

参考：`7_插值算法和偏置校准`

目标：建立可靠的位置和速度估算。

任务：

- [ ] SPI 编码器驱动框架
- [ ] AS5047P 支持
- [ ] MT6701 支持
- [ ] CPR 配置
- [ ] 绝对位置读取
- [ ] `pos_estimate`
- [ ] `vel_estimate`
- [ ] PLL 插值
- [ ] 编码器方向识别
- [ ] 编码器 offset calibration
- [ ] `AXIS_STATE_ENCODER_OFFSET_CALIBRATION`

验收：

- [ ] 手转电机时位置连续变化
- [ ] 速度估算方向正确
- [ ] offset calibration 后电角度正确
- [ ] 正反转方向识别正确

### 阶段 7：闭环控制

参考：`8_闭环控制`

目标：完成力矩、速度、位置闭环。

任务：

- [ ] `axis.c`
- [ ] `motor.c`
- [ ] `encoder.c`
- [ ] `controller.c`
- [ ] `foc.c`
- [ ] `trap_traj.c`
- [ ] 电流环闭环
- [ ] 力矩控制
- [ ] 速度控制
- [ ] 位置控制
- [ ] 梯形轨迹
- [ ] 闭环进入/退出命令

控制模式：

- [ ] `CONTROL_MODE_VOLTAGE_CONTROL`
- [ ] `CONTROL_MODE_TORQUE_CONTROL`
- [ ] `CONTROL_MODE_VELOCITY_CONTROL`
- [ ] `CONTROL_MODE_POSITION_CONTROL`

输入模式：

- [ ] `INPUT_MODE_INACTIVE`
- [ ] `INPUT_MODE_PASSTHROUGH`
- [ ] `INPUT_MODE_VEL_RAMP`
- [ ] `INPUT_MODE_TORQUE_RAMP`
- [ ] `INPUT_MODE_POS_FILTER`
- [ ] `INPUT_MODE_TRAP_TRAJ`

验收：

- [ ] 能进入 closed loop
- [ ] 力矩命令有效
- [ ] 速度命令有效
- [ ] 位置命令有效
- [ ] 梯形轨迹平滑
- [ ] 超速、过流等错误能退出闭环

### 阶段 8：参数保存和启动配置

目标：支持像 ODrive 一样保存校准和启动行为。

任务：

- [ ] Flash 参数结构体
- [ ] 默认参数
- [ ] 参数版本号
- [ ] CRC 校验
- [ ] 保存参数
- [ ] 擦除参数
- [ ] 启动读取参数
- [ ] `pre_calibrated`
- [ ] `startup_closed_loop_control`
- [ ] `startup_encoder_offset_calibration`

验收：

- [ ] 断电重启后参数仍然存在
- [ ] CRC 错误时恢复默认参数
- [ ] 可以配置上电自动闭环

### 阶段 9：通信协议

目标：建立调试和上位机控制入口。

任务：

- [ ] USB CDC 命令
- [ ] UART 命令
- [ ] CAN 命令
- [ ] 查询状态
- [ ] 查询错误
- [ ] 设置控制模式
- [ ] 设置输入模式
- [ ] 设置位置/速度/力矩
- [ ] 执行校准
- [ ] 保存/擦除参数

基础命令建议：

```text
help
status
error
clear_errors
calib_motor
calib_encoder
closed_loop
idle
set_pos
set_vel
set_torque
save_config
erase_config
reboot
```

验收：

- [ ] USB 和 UART 都能控制电机
- [ ] 命令不会阻塞高频控制
- [ ] 错误信息可读
- [ ] 参数可读写

### 阶段 10：抗齿槽效应

参考：`9_抗齿槽效应算法`

目标：实现位置控制下的齿槽转矩补偿。

任务：

- [ ] `anticogging_config`
- [ ] cogging map 数组
- [ ] 抗齿槽校准状态
- [ ] 缓慢扫描一圈
- [ ] 记录补偿 torque
- [ ] 运行时查表补偿
- [ ] 保存抗齿槽数据
- [ ] 启用/禁用抗齿槽

验收：

- [ ] 能完成一圈抗齿槽校准
- [ ] 校准数据能保存
- [ ] 低速位置控制抖动减小
- [ ] 禁用后补偿不再生效

## 6. 核心数据结构规划

### 6.1 Axis

```c
typedef enum {
    AXIS_STATE_IDLE = 0,
    AXIS_STATE_MOTOR_CALIBRATION,
    AXIS_STATE_ENCODER_INDEX_SEARCH,
    AXIS_STATE_ENCODER_DIR_FIND,
    AXIS_STATE_ENCODER_OFFSET_CALIBRATION,
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_UNDEFINED,
} axis_state_t;
```

### 6.2 Motor

```c
typedef struct {
    bool pre_calibrated;
    int32_t pole_pairs;
    float calibration_current;
    float resistance_calib_max_voltage;
    float phase_resistance;
    float phase_inductance;
    float current_lim;
    float current_control_bandwidth;
    int32_t motor_type;
} motor_config_t;
```

### 6.3 Encoder

```c
typedef struct {
    bool pre_calibrated;
    int32_t mode;
    int32_t cpr;
    int32_t offset;
    int32_t direction;
    float bandwidth;
} encoder_config_t;
```

### 6.4 Controller

```c
typedef struct {
    int32_t control_mode;
    int32_t input_mode;
    float pos_gain;
    float vel_gain;
    float vel_integrator_gain;
    float vel_limit;
    float torque_ramp_rate;
    float vel_ramp_rate;
} controller_config_t;
```

## 7. 错误码规划

```text
ERROR_NONE
ERROR_MODULATION_MAGNITUDE
ERROR_CURRENT_SENSE_SATURATION
ERROR_CURRENT_LIMIT_VIOLATION
ERROR_PHASE_RESISTANCE_OUT_OF_RANGE
ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE
ERROR_UNKNOWN_VBUS_VOLTAGE
ERROR_ABS_SPI_COM_FAIL
ERROR_UNSUPPORTED_ENCODER_MODE
ERROR_CPR_POLEPAIRS_MISMATCH
ERROR_INVALID_STATE
ERROR_INVALID_INPUT_MODE
ERROR_OVERSPEED
ERROR_INVALID_ESTIMATE
```

## 8. 开发注意事项

- 每完成一个阶段都单独验证，不要跳着写闭环。
- 没有示波器确认 PWM 前，不要接电机大电流测试。
- 电流采样方向必须确认，否则电流环会正反馈。
- 第一次开环只给很小电压。
- FOC 高频回调里不要打印，不要阻塞，不要使用复杂动态内存。
- 通信解析放低速任务，中断只搬运数据。
- Flash 写入不要在高频中断里做。
- 所有 `arm()` 前必须确认母线电压、电流零点、错误状态。
- 所有错误都必须能 `disarm()`。

## 9. 当前优先级

第一轮只做到闭环可用，不追求完整 ODrive 协议：

1. CubeMX + VSCode 工程跑通。
2. UART4 + USB CDC。
3. M0 TIM1 PWM + ADC 采样。
4. 开环转动。
5. 电阻电感测量。
6. 编码器读取和 offset calibration。
7. 电流环、速度环、位置环。
8. 参数保存。
9. CAN 和抗齿槽。

## 10. 阶段完成记录

| 阶段 | 名称 | 状态 | 备注 |
| --- | --- | --- | --- |
| 0 | 工程骨架 | 未开始 | CubeMX + VSCode |
| 1 | 启动/时间/UART4 | 未开始 | 最小系统 |
| 2 | USB/命令/数学工具 | 未开始 | 通信入口 |
| 3 | M0 TIM1/ADC | 未开始 | 高频控制地基 |
| 4 | 开环控制 | 未开始 | 小电压测试 |
| 5 | 电阻电感测量 | 未开始 | motor calibration |
| 6 | 编码器校准 | 未开始 | SPI + PLL |
| 7 | 闭环控制 | 未开始 | 力矩/速度/位置 |
| 8 | 参数保存 | 未开始 | Flash + CRC |
| 9 | 通信协议 | 未开始 | USB/UART/CAN |
| 10 | 抗齿槽 | 未开始 | 最后实现 |
