# YDrive

YDrive 是一个参考 ODrive v3 / ODrive firmware v0.5.6 架构整理出来的 STM32F405 PMSM/FOC 学习工程。

当前阶段已经完成：

- TIM1 中心对齐三相互补 PWM
- ADC1/ADC2/ADC3 injected 同步采样
- 母线电压采样
- phase B / phase C 电流采样，phase A 由 `ia = -ib - ic` 计算
- DC offset 校准
- 电机相电阻测量
- 电机相电感测量
- 开环电压旋转控制
- USB CDC 命令控制

下一阶段重点是接入编码器，然后做电流闭环、速度闭环和位置闭环。

## 当前架构

实时控制链路模仿 ODrive v0.5.6：

```text
TIM1 center-aligned PWM
  -> TIM1 update interrupt
     -> 判断 TIM1->CR1 DIR
     -> 向上计数：触发 ControlLoop_IRQn
     -> 向下计数：CCR 临时写 50%

ControlLoop_IRQHandler
  -> fetch_and_reset_adcs()
     -> ADC1 JDR1: vbus
     -> ADC2 JDR1: phase B current
     -> ADC3 JDR1: phase C current
  -> current_meas_cb()
     -> 扣除 DC_calib
     -> Clarke transform
     -> 测量模式下处理 R/L 输入侧
  -> axis_control_loop_cb()
     -> open_loop_controller_update()
  -> 等待下一次 injected ADC 完成
  -> fetch_and_reset_adcs()
  -> dc_calib_cb()
  -> pwm_update_cb()
     -> 测量模式输出 R/L 测试电压
     -> 正常模式输出 open-loop FOC voltage
```

`ControlLoop_IRQn` 当前映射到 `OTG_HS_IRQn`：

```c
#define ControlLoop_IRQHandler OTG_HS_IRQHandler
#define ControlLoop_IRQn OTG_HS_IRQn
```

## 目录结构

```text
YDrive/
  Board/          CubeMX 生成代码、HAL、USB、FreeRTOS、CMake
  Communication/  USB CDC 命令解析
  MotorControl/   PWM/ADC 时序、FOC、Motor、Axis、开环控制
  Docs/           环境和架构笔记
```

核心文件：

```text
MotorControl/board.c/.h
  TIM1 PWM/ADC injected/ControlLoop IRQ/电流采样/母线采样

MotorControl/motor.c/.h
  电机参数、arm/disarm、DC calibration、电阻测量、电感测量、PWM 输出分发

MotorControl/foc.c/.h
  Clarke transform、逆 Park、SVM、PWM CCR 写入

MotorControl/open_loop_controller.c/.h
  开环电角度、开环电角速度、电压斜坡、速度斜坡

MotorControl/axis.c/.h
  状态机、校准命令、开环状态入口、控制循环入口

Communication/usb_command.c/.h
  USB CDC ASCII 命令
```

## 启动流程

```text
main()
  -> MX_FREERTOS_Init()
     -> MX_USB_DEVICE_Init()
     -> defaultTask

defaultTask
  -> motor_para_init()
  -> motor_setup()
  -> motor_control_start()
  -> loop:
       USBcommander_run()
       run_state_machine_loop()
       osDelay(1)
```

实时电机控制不放在 FreeRTOS task 中，而是由 TIM1 update 触发硬中断链路。

## 电流采样

ADC injected 采样：

```text
ADC1->JDR1 = vbus
ADC2->JDR1 = phase B current
ADC3->JDR1 = phase C current
phase A    = -phase B - phase C
```

电流换算在 `phase_current_from_adcval()`：

```text
adcval_bal  = adc - 2048
amp_out_v   = 3.3 / 4096 * adcval_bal
shunt_v     = amp_out_v / PHASE_CURRENT_GAIN
current_A   = shunt_v / SHUNT_RESISTANCE
```

当前参数：

```text
VBUS_S_DIVIDER_RATIO = 19.0
SHUNT_RESISTANCE     = 0.0005 ohm
PHASE_CURRENT_GAIN   = 10.0
```

如果 ADC 原始值超出允许范围，会置位：

```text
ERROR_CURRENT_SENSE_SATURATION
```

## 电阻电感测量

USB 发送 `C` 后进入：

```text
AXIS_STATE_MOTOR_CALIBRATION
  -> run_calibration()
     -> measure_phase_resistance()
     -> measure_phase_inductance()
     -> update_current_controller_gains()
```

测量输出示例：

```text
OK: motor calibration start
motor R_mohm=120,L_uH=10
P_milli=10,I_milli=80
status=1,error=0.
```

说明：

- `R_mohm`：相电阻，单位 mOhm
- `L_uH`：相电感，单位 uH
- `P_milli`：电流环 P 参数乘以 1000
- `I_milli`：电流环 I 参数乘以 1000
- `status=1` 表示测量成功
- `error=0` 表示无错误

这里避免使用 `%f/%e` 浮点 printf，防止嵌入式库未开启浮点格式化时输出乱码。

## 开环控制

USB 命令：

```text
O
O <voltage_V> <electrical_rad_s>
S
```

示例：

```text
O
O 0.6 62.8
O 0.6 -62.8
S
```

说明：

- `voltage_V` 是输出电压目标，单位 V
- `electrical_rad_s` 是电角速度，单位 rad/s
- 速度为正时正转，速度为负时反转
- `S` 停止输出并 disarm

默认开环参数：

```text
target_voltage      = 0.6 V
target_vel          = 62.8 electrical rad/s
max_voltage_ramp    = 2.0 V/s
max_phase_vel_ramp  = 1000 rad/s^2
```

如果电机是 7 极对，电角速度到机械转速的换算为：

```text
mechanical_rpm = electrical_rad_s / (2*pi*pole_pairs) * 60
```

例如：

```text
62.8 / (2*pi*7) * 60 = 85.7 rpm
```

纯开环启动时轻微回拉或抖一下是正常现象，因为启动时控制器假设 `phase = 0`，而真实转子可能停在任意角度。

## USB 命令

当前命令：

```text
H / h / ?        help
C / c            measure motor R/L
O / o            open loop start with default target
O <volt> <vel>   open loop start with target voltage and electrical velocity
S / s            stop
```

帮助输出：

```text
YDrive measure mode
H: help
C: measure motor R/L
O [volt vel]: open loop
S: stop
```

开环启动回显：

```text
OK: open loop voltage_mV=600 vel_mrad_s=62800
```

## 硬件配置

```text
MCU      = STM32F405RGTx
RTOS     = FreeRTOS CMSIS-RTOS2
USB      = USB CDC
PWM      = TIM1
ADC      = ADC1/ADC2/ADC3 injected
EN_GATE  = PB12
nFAULT   = PD2
```

TIM1：

```text
Counter Mode      = Center Aligned 3
Period            = TIM_1_8_PERIOD_CLOCKS = 3500
RepetitionCounter = TIM_1_8_RCR = 2
PWM Mode          = PWM2
TRGO              = Update Event
DeadTime          = TIM_1_8_DEADTIME_CLOCKS
```

控制周期：

```text
TIM_1_8_CLOCK_HZ = 168000000
half_period_tick = TIM_1_8_PERIOD_CLOCKS * (TIM_1_8_RCR + 1)
                 = 3500 * (2 + 1)
                 = 10500

current_meas_period = 2 * 10500 / 168000000
                    = 0.000125 s

current_meas_hz = 8000 Hz
```

## 错误位

错误位定义在 `MotorControl/motor.h`：

```text
ERROR_MODULATION_MAGNITUDE
ERROR_CURRENT_SENSE_SATURATION
ERROR_CURRENT_LIMIT_VIOLATION
ERROR_PHASE_RESISTANCE_OUT_OF_RANGE
ERROR_UNKNOWN_VBUS_VOLTAGE
ERROR_NOT_HIGH_CURRENT_MOTOR
ERROR_UNBALANCED_PHASES
ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE
ERROR_BAD_TIMING
ERROR_TIMER_UPDATE_MISSED
ERROR_CONTROL_DEADLINE_MISSED
```

当前错误仍采用 bitmask，便于多个 fault 同时记录。

## 构建

环境说明：

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

- 上电默认 PWM 不输出
- `arm()` 直接置位 `TIM1->BDTR.MOE`
- `disarm()` 直接清除 `TIM1->BDTR.MOE`
- TIM update 方向异常会置位 `ERROR_TIMER_UPDATE_MISSED`
- ControlLoop 未在预期半周期内完成会置位 `ERROR_CONTROL_DEADLINE_MISSED`
- 调试开环时从低电压、低速度开始

推荐测试顺序：

```text
H
C
O 0.4 20
O 0.6 40
O 0.6 62.8
O 0.6 -62.8
S
```

## 下一步

建议继续按这个顺序推进：

1. 接入编码器读取与角度换算。
2. 做编码器 offset calibration。
3. 用编码器角度替换开环 phase。
4. 完成 Id/Iq 电流闭环。
5. 完成速度闭环。
6. 完成位置闭环。
