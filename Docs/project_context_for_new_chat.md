# YDrive 新对话上下文速记

这份文档用于新开对话后快速恢复上下文。它不是完整教程，而是当前工程的“脑图 + 已做决定 + 下一步”。

## 工程目标

YDrive 是一个参考 ODrive v3 / ODrive firmware v0.5.1 的 PMSM/FOC 学习工程。

当前目标是用 C 语言手撕 ODrive 的关键链路：

```text
PWM/ADC 时序
  -> 真实相电流采样
  -> MotorAxis 实时控制任务
  -> 开环电压 / 开环电流
  -> FOC / SVPWM
  -> 下一拍 PWM
```

不是一次性复刻完整 ODrive，而是分阶段建立：

```text
开环电压 -> 电流环 -> 编码器相位 -> 速度环 -> 位置环
```

## 当前代码状态

已经完成：

- TIM1 center-aligned 三相互补 PWM。
- ADC injected 采样。
- ADC2/ADC3 采 phase B / phase C。
- `dir=0` 作为真实电流采样窗口。
- `dir=1` 作为 DC_CAL / offset 更新窗口。
- 约 8 kHz 真实电流控制节拍。
- `MotorAxis` 主控制对象。
- ODrive 风格 deadline 检查。
- 开环电压控制：`open` 命令。
- 开环相位 + 电流 PI 测试：`cur` 命令。
- USB 查看/设置电流环参数：`cparam` / `cgain` 命令。
- `CurrentController` 电流环雏形。
- monitor 输出拆成 `sys` / `ctrl` / `adc` 三组。

还没完成：

- ADC raw count 到真实 A 的精确标定。
- 电阻/电感测量。
- 正式电流环 PI 参数整定。
- 编码器接入和电角度校准。
- 速度环、位置环。

## 关键目录

```text
YDrive/
  Board/          CubeMX/HAL/USB/FreeRTOS/CMake
  MotorControl/   电机控制核心
  Communication/  USB CDC 命令
  Tasks/          FreeRTOS 应用任务
  Docs/           手撕笔记和时序说明
```

## 核心文件

### MotorControl/motor_axis.c/.h

电机轴总管。

负责：

- 创建电机控制主状态。
- 等待 ADC 真实电流采样 flag。
- 运行当前控制模式。
- deadline 检查。
- 输出 enable / disarm。
- 汇总 monitor 状态。

当前模式：

```c
MOTOR_AXIS_MODE_IDLE              = 0
MOTOR_AXIS_MODE_OPEN_LOOP_VOLTAGE = 1
MOTOR_AXIS_MODE_OPEN_LOOP_CURRENT_LOCKIN = 2
MOTOR_AXIS_MODE_OPEN_LOOP_CURRENT_RUN    = 3
```

内部关键变量：

```c
output_active
```

表示 EN_GATE/PWM 输出是否已经真正启用。

```c
control_cycle_finished
```

表示上一拍控制计算是否已经完成并写好 PWM。

deadline 逻辑：

```text
ADC ISR 来了
  如果 output_active = 1
  且 control_cycle_finished = 0
  -> deadline_miss_count++
  -> disarm
```

### MotorControl/pwm_adc.c/.h

PWM/ADC 硬件层。

负责：

- 初始化 gate/PWM 中性值。
- 启动 ADC injected 和 TIM1 PWM。
- 写 PWM CCR。
- 更新 ADC offset。
- 保存 `PhaseCurrentSample`。
- 在真实电流采样完成后调用 `motor_axis_signal_current_meas_from_isr()`。

重要节奏：

```text
dir = 0:
  采真实电流
  计算 ia/ib/ic
  唤醒 motor_axis_task

dir = 1:
  采 zero current
  更新 offset / DC_CAL
```

### MotorControl/open_loop_controller.c/.h

开环电角度推进器。

负责：

- 目标电压调制量斜坡。
- 目标电角速度斜坡。
- `electrical_phase += electrical_phase_vel * dt`
- 给开环电压模式输出 `Vd/Vq`。
- 给开环电流模式提供 `current_phase`。

当前保留 ODrive 风格 phase advance：

```c
pwm_phase = phase + 1.5f * dt * phase_vel;
```

### MotorControl/current_controller.c/.h

电流环雏形。

负责：

```text
phase current raw
  -> phase_current_gain
  -> Clarke
  -> Park
  -> Id/Iq
  -> PI
  -> Vd/Vq modulation
  -> foc_controller_apply_voltage()
```

当前结构：

```c
typedef struct {
    float phase_current_gain;
    float current_limit;
    float max_voltage_mod;
} CurrentControllerConfig;

typedef struct {
    DqVector setpoint;
    DqVector ramped_setpoint;
    DqVector measured;
} CurrentControllerCurrentState;

typedef struct {
    float kp;
    float ki;
    DqVector integral_voltage;
    DqVector output_voltage;
} CurrentControllerPi;

typedef struct {
    CurrentControllerConfig config;
    CurrentControllerCurrentState current;
    CurrentControllerPi pi;
    uint32_t enabled;
    uint32_t error;
} CurrentController;
```

注意：

- `config.phase_current_gain = 0.01f` 只是 bring-up 默认值。
- 当前 `CURRENT_CONTROLLER_ERROR_CURRENT_LIMIT` 主要用于提示，不是最终硬保护。
- `cur` 仍然使用开环相位，不是真正编码器相位。

### MotorControl/foc.c/.h

FOC/SVPWM 输出层。

负责：

- `foc_controller_apply_voltage(v_d, v_q, pwm_phase)`
- 逆 Park 得到 alpha/beta 调制量。
- `svm()`
- 生成三相 PWM ticks。
- 调用 `pwm_adc_write_pwm()`。

### MotorControl/utils.h

基础数学工具。

当前把原来的 `Float2D` 改成了：

```c
typedef struct {
    float d;
    float q;
} DqVector;
```

原因：`DqVector` 一眼能看出是 d/q 坐标系，不像 `Float2D` 那样泛。

以后如果需要 alpha/beta，建议另建：

```c
typedef struct {
    float alpha;
    float beta;
} AlphaBetaVector;
```

### Communication/usb_command.c/.h

USB CDC ASCII 命令解析。

当前命令：

```text
open <voltage_mod> <electrical_rad_s>
cur <iq_amp> <electrical_rad_s>
cparam [phase_current_gain current_limit max_voltage_mod]
cgain [p_gain i_gain]
stop
idle
help
```

测试建议：

```text
open 0.02 5
cur 0.02 5
cparam
cgain
stop
```

### Tasks/monitor_task.c/.h

只负责 USB boot 消息和状态打印。

注意：电机硬件初始化应该属于 `motor_axis` / `pwm_adc`，不要把 PWM/ADC start 逻辑塞回 monitor。

当前 monitor 每 1000 ms 打印：

```text
sys: ...
ctrl: ...
adc: ...
```

`sys` 看系统状态和频率。

`ctrl` 看开环和电流环。

`adc` 看底层 ADC/PWM。

## 当前实时链路

```text
TIM1 update/TRGO
  -> ADC injected conversion
  -> ADC IRQ
  -> pwm_adc_trig_adc_cb()
     -> ADC2 保存 ib_raw
     -> ADC3 保存 ic_raw
     -> 如果 dir=0 且 B/C 都到齐：
          latest_current.ib = ib_raw - ib_offset
          latest_current.ic = ic_raw - ic_offset
          latest_current.ia = -ib - ic
          current_meas_count++
          motor_axis_signal_current_meas_from_isr()

motor_axis_signal_current_meas_from_isr()
  -> deadline 检查
  -> control_cycle_finished = 0
  -> osThreadFlagsSet(motorAxisTaskHandle, MOTOR_AXIS_CURRENT_MEAS_FLAG)

motor_axis_task()
  -> osThreadFlagsWait()
  -> pwm_adc_get_phase_current_sample()
  -> motor_axis_update_controller()
  -> motor_axis_update_output()
  -> control_cycle_finished = 1
```

## 当前电流环路径

```text
USB:
  cur iq_amp electrical_rad_s

motor_axis:
  set open_loop phase velocity
  set current iq target
  mode = OPEN_LOOP_CURRENT_LOCKIN
  lock-in 完成后 mode = OPEN_LOOP_CURRENT_RUN

每次真实电流采样：
  open_loop_controller_update_phase()
  current_phase = open_loop electrical_phase
  pwm_phase = current_phase + 1.5 * dt * phase_vel
  current_controller_update()

current_controller:
  sample ib/ic raw
  * phase_current_gain
  Clarke -> i_alpha/i_beta
  Park -> id/iq
  PI -> vd/vq
  foc_controller_apply_voltage()
```

## 已经踩过的坑

### 1. `CURRENT_MEAS_TIMEOUT_MS` 不是 FOC 周期

`osThreadFlagsWait(..., 10ms)` 是 ADC/TIM 停止时的 watchdog。

真正控制节拍来自 ADC flag，大约 8 kHz。

### 2. `output_active` 不等于 USB 命令刚收到

USB `open/cur` 只切换模式和目标值。

真正完成第一拍 PWM 计算并通过校准检查后，才：

```c
pwm_adc_set_gate_enabled(1U);
output_active = 1U;
```

### 3. `control_cycle_finished` 初始必须是 1

上电还没有 pending 控制任务，不能一进 ADC ISR 就误判 missed deadline。

### 4. `phase_current_gain` 还没标定

当前电流环的 `id/iq` 单位名义上是 A，但比例还只是默认值。

所以 `cur` 命令必须从很小开始：

```text
cur 0.02 5
```

### 5. monitor 不应该初始化电机硬件

monitor 只打印。

PWM/ADC/Gate 初始化应该在 `motor_axis` / `pwm_adc` 这一侧。

## 编码和文档注意事项

- README 曾经出现过乱码，已重写为 UTF-8。
- 新增中文文档时，要确保保存为 UTF-8。
- PowerShell 直接 `Get-Content` 可能显示乱码，不一定代表文件坏了。

## 当前未提交/工作区提醒

常见会出现一个 workspace 文件改动：

```text
ODrive移植keil.code-workspace
```

如果只是 IDE 自动改动，提交代码时一般不要带进去。

## 下一步建议

优先级从高到低：

1. 标定 `phase_current_gain`，让 ADC raw 变成真实 A。
2. 使用 `cparam` 现场写入标定后的电流比例、限流和最大调制量。
3. 使用 `cgain` 根据电机 R/L 重新设计 PI 参数。
4. 做电阻、电感测量流程。
5. 接编码器，得到真实 electrical phase。
6. 用编码器相位替换 open-loop phase，形成真正电流闭环。
7. 在电流环之上做速度环。
8. 在速度环之上做位置环。

## 新对话建议先读哪些文件

推荐顺序：

1. `README.md`
2. `Docs/project_context_for_new_chat.md`
3. `Docs/motor_axis_current_controller_hands_on.md`
4. `MotorControl/motor_axis.c`
5. `MotorControl/current_controller.c`
6. `MotorControl/pwm_adc.c`
7. `Communication/usb_command.c`
8. `Tasks/monitor_task.c`
