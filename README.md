# YDrive

YDrive 是一个参考 ODrive v3 / ODrive firmware v0.5.1 思路整理出来的 STM32F405 PMSM/FOC 学习工程。

当前目标不是一次性复刻完整 ODrive，而是把关键链路拆开手撕：

```text
TIM1 center-aligned PWM
  -> TIM1 TRGO update 触发 ADC injected
  -> ADC2/ADC3 采 phase B / phase C 电流
  -> dir=0 作为真实电流采样窗口
  -> motor_axis_task 被唤醒
  -> 开环电压 / 开环电流控制
  -> FOC / SVPWM
  -> 写入下一拍 PWM
```

## 当前状态

- STM32F405 + FreeRTOS CMSIS-RTOS2 + USB CDC
- TIM1 三相互补 PWM，center-aligned 模式
- ADC1/ADC2/ADC3 injected conversion
- ADC2/ADC3 采样 phase B / phase C 电流
- ODrive 风格 `dir=0` 真实电流采样，`dir=1` DC_CAL 零偏更新
- 真实电流控制节拍约 8 kHz
- `MotorAxis` 作为电机轴主对象
- 支持开环电压旋转：`open`
- 支持开环相位 + 电流 PI 测试模式：`cur`
- USB CDC ASCII 命令控制
- 监控输出分为 `sys` / `ctrl` / `adc` 三组

## 目录结构

```text
YDrive/
  Board/          CubeMX 生成代码、CMake、HAL、USB、FreeRTOS
  MotorControl/   PWM/ADC、MotorAxis、FOC、开环控制、电流环
  Communication/  USB CDC 命令解析
  Tasks/          FreeRTOS 应用任务
  Docs/           时序、SVPWM、环境说明笔记
```

职责划分：

```text
Board
  -> 保持 CubeMX/HAL/USB/FreeRTOS 生成层

Tasks
  -> 创建和运行应用任务
  -> monitor_task 打印状态

Communication
  -> USB CDC ASCII 命令解析

MotorControl
  -> 电机控制主逻辑
  -> PWM/ADC 时序
  -> FOC/SVPWM
  -> 开环电压控制
  -> 开环电流 PI 测试
```

## 启动链路

```text
main()
  -> MX_FREERTOS_Init()
     -> MX_USB_DEVICE_Init()
     -> tasks_init()
        -> monitor_task
        -> motor_axis_task
```

`motor_axis_task` 负责初始化并启动 PWM/ADC 时序；`monitor_task` 只负责发送 boot 消息和周期性打印状态。

## 控制时序

```text
TIM1 update/TRGO
  -> ADC injected conversion
  -> ADC IRQ
  -> pwm_adc_trig_adc_cb()
     -> ADC2 保存 ib_raw
     -> ADC3 保存 ic_raw
     -> dir=0:
          计算 ia/ib/ic
          current_meas_count++
          motor_axis_signal_current_meas_from_isr()
     -> dir=1:
          更新 ib/ic offset

motor_axis_task
  -> 等待 MOTOR_AXIS_CURRENT_MEAS_FLAG
  -> 读取 PhaseCurrentSample
  -> 根据 mode 分发控制器
  -> 写入 PWM
  -> control_cycle_finished = 1
```

deadline 机制：

```text
ADC ISR 到来时：
  如果 output_active = 1
  且上一拍 control_cycle_finished = 0
  -> deadline_miss_count++
  -> disarm
```

## MotorControl 模块

```text
motor_axis.c/.h
  电机轴主对象，负责控制模式、deadline、输出使能、状态汇总。

pwm_adc.c/.h
  TIM1 PWM、ADC injected、DC_CAL、相电流采样、EN_GATE。

open_loop_controller.c/.h
  开环电角度推进，支持电压开环，也给电流环提供开环 phase。

current_controller.c/.h
  Id/Iq 电流环雏形：
    phase current -> Clarke/Park -> Id/Iq
    Id/Iq error -> PI -> Vd/Vq
    Vd/Vq -> FOC/SVM

foc.c/.h
  逆 Park、SVPWM、电压调制到 PWM ticks。

utils.c/.h
  clamp、wrap、SVPWM 基础数学工具。

arm_cos_f32.c / arm_sin_f32.c / arm_sin_cos_f32.h
  ODrive 风格 sin/cos 接口。
```

## 控制模式

```c
MOTOR_AXIS_MODE_IDLE              = 0
MOTOR_AXIS_MODE_OPEN_LOOP_VOLTAGE = 1
MOTOR_AXIS_MODE_OPEN_LOOP_CURRENT = 2
```

### 开环电压模式

```text
open <voltage_mod> <electrical_rad_s>
```

例子：

```text
open 0.02 5
```

含义：

- `voltage_mod`：电压调制幅值，不是真实伏特，当前限制在较小范围内
- `electrical_rad_s`：电角速度，单位 rad/s

### 开环电流模式

```text
cur <iq_amp> <electrical_rad_s>
```

例子：

```text
cur 0.02 5
```

含义：

- `iq_amp`：q 轴目标电流，单位 A
- `electrical_rad_s`：开环电角速度，单位 rad/s

注意：当前 `phase_current_gain` 仍是调试默认值，ADC count 到真实 A 的比例还需要标定。第一次测试从很小的 `iq_amp` 开始。

### 停止

```text
stop
idle
```

## USB 命令

```text
open <voltage_mod> <electrical_rad_s>
cur <iq_amp> <electrical_rad_s>
stop
idle
help
```

## 监控输出

监控任务每 1000 ms 打印一次，分三行：

```text
sys: nfault=1 mode=2 out=1 cal=1 loop=12345 miss=0 adc_hz=48000 cur_hz=8000
ctrl: open=1 phase_mrad=120 vel_mrad_s=5000 v_milli=0 iq_sp_ma=20 id_ma=-5 iq_ma=18 vd_milli=1 vq_milli=3 ierr=0
adc: dir=1 dc_cal=2048 vbus=807 raw=386/384 off=384/383 iabc=-2/1/1 pwm=1749/1750/1751
```

`sys`：

- `nfault`：DRV nFAULT 引脚，1 表示没有 fault
- `mode`：MotorAxis 当前模式
- `out`：EN_GATE/PWM 输出是否已启用
- `cal`：ADC offset 是否校准完成
- `loop`：motor axis 控制循环计数
- `miss`：控制 deadline miss 次数
- `adc_hz`：本打印周期内 ADC injected IRQ 次数，1s 打印时约 48000
- `cur_hz`：真实电流采样次数，1s 打印时约 8000

`ctrl`：

- `open`：开环控制器是否 enable
- `phase_mrad`：开环电角度，单位 mrad
- `vel_mrad_s`：开环电角速度，单位 mrad/s
- `v_milli`：开环电压调制量，乘以 1000 打印
- `iq_sp_ma`：q 轴目标电流，单位 mA
- `id_ma` / `iq_ma`：测得 d/q 轴电流，单位 mA
- `vd_milli` / `vq_milli`：电流 PI 输出的 d/q 轴电压调制量，乘以 1000 打印
- `ierr`：电流环错误标志

`adc`：

- `dir`：TIM1 当前计数方向
- `dc_cal`：DC_CAL offset 更新计数
- `vbus`：母线电压 ADC raw
- `raw`：phase B / phase C ADC raw
- `off`：phase B / phase C offset
- `iabc`：offset 后的 ia/ib/ic raw
- `pwm`：三相 PWM CCR ticks

## 硬件配置

- MCU：STM32F405RGTx
- RTOS：FreeRTOS CMSIS-RTOS2
- USB：USB CDC
- PWM：TIM1
- ADC：ADC1/ADC2/ADC3 injected
- EN_GATE：PB12
- nFAULT：PD2

TIM1：

```text
Counter Mode      = Center Aligned 3
Period            = 3500
RepetitionCounter = 2
PWM Mode          = PWM2
TRGO              = Update Event
DeadTime          = 20
```

ADC：

```text
ADC1: VBUS
ADC2: phase B current
ADC3: phase C current
```

## 构建

详细环境说明见：

[Docs/environment_and_build.md](Docs/environment_and_build.md)

常用命令：

```powershell
cd C:\Users\mrg\Desktop\PMSMFocLib\YDrive\Board
cmake --fresh --preset Debug
cmake --build --preset Debug
```

输出文件：

```text
Board/build/Debug/YDrive.elf
Board/build/Debug/YDrive.bin
Board/build/Debug/YDrive.hex
```

## 安全约定

- 上电默认 `EN_GATE=0`
- ADC offset 未校准完成时不允许打开输出
- deadline miss 时 disarm
- 测试从很小命令开始，例如：

```text
open 0.02 5
cur 0.02 5
```

## 参考文档

- [Docs/environment_and_build.md](Docs/environment_and_build.md)
- [Docs/ODrive_motor0_timing_notes.md](Docs/ODrive_motor0_timing_notes.md)
- [Docs/svpwm_understanding_notes.md](Docs/svpwm_understanding_notes.md)
- [Docs/ydrive_cubemx_and_plan.md](Docs/ydrive_cubemx_and_plan.md)

## 下一步

建议继续按这个顺序推进：

1. 标定 ADC raw 到真实相电流 A 的比例。
2. 根据电阻/电感设置更合理的 PI 参数。
3. 增加电流环更严格的 fault 策略。
4. 接入编码器角度，替换开环 phase。
5. 做速度闭环。
6. 做位置闭环。
