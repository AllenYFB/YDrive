# MotorAxis 和 CurrentController 手撕路线

这份笔记记录如果从零手写 `MotorControl/motor_axis.c/.h` 和 `MotorControl/current_controller.c/.h`，应该按什么顺序来。

核心原则：

- `MotorAxis` 是电机轴总管，负责实时任务、模式切换、安全输出、状态汇总。
- `CurrentController` 只负责电流环数学，不直接管 FreeRTOS、不直接管 EN_GATE。
- `FocController` 继续负责把 `Vd/Vq` 或调制量变成 SVPWM/PWM。
- `PwmAdc` 继续负责 ADC/PWM 硬件采样和写 CCR。

## 1. 先写 MotorAxis 的最小外壳

先不要急着写电流环，先把“电机轴对象”搭出来。

`motor_axis.h` 先定义模式：

```c
typedef enum {
    MOTOR_AXIS_MODE_IDLE = 0,
    MOTOR_AXIS_MODE_OPEN_LOOP_VOLTAGE = 1,
    MOTOR_AXIS_MODE_OPEN_LOOP_CURRENT_LOCKIN = 2,
    MOTOR_AXIS_MODE_OPEN_LOOP_CURRENT_RUN = 3,
} MotorAxisMode;
```

含义：

- `IDLE`：不输出。
- `OPEN_LOOP_VOLTAGE`：开环电压旋转。
- `OPEN_LOOP_CURRENT_LOCKIN`：启动时先模仿 ODrive lock-in，缓慢给电流并轻推电角度。
- `OPEN_LOOP_CURRENT_RUN`：lock-in 完成后，进入开环相位 + 电流 PI 测试。

再定义公开状态：

```c
typedef struct {
    uint32_t loop_count;
    uint32_t deadline_miss_count;
    MotorAxisMode mode;
    uint32_t output_active;
} MotorAxisRuntimeStatus;

typedef struct {
    int32_t ia;
    int32_t ib;
    int32_t ic;
} MotorAxisPhaseCurrentStatus;

typedef struct {
    uint32_t enabled;
    float electrical_phase;
    float electrical_phase_vel;
    float voltage_mod;
} MotorAxisOpenLoopStatus;

typedef struct {
    float id_setpoint;
    float iq_setpoint;
    float id_measured;
    float iq_measured;
    float vd_mod;
    float vq_mod;
    float phase_gain;
    float p_gain;
    float i_gain;
    float limit;
    float max_voltage_mod;
    uint32_t error;
} MotorAxisCurrentStatus;

typedef struct {
    uint32_t a;
    uint32_t b;
    uint32_t c;
} MotorAxisPwmStatus;

typedef struct {
    MotorAxisRuntimeStatus runtime;
    MotorAxisPhaseCurrentStatus phase_current;
    MotorAxisOpenLoopStatus open_loop;
    MotorAxisCurrentStatus current;
    MotorAxisPwmStatus pwm;
} MotorAxisStatus;
```

这个 status 是给 USB/monitor 看的，不要把所有内部变量都塞进去，只放调试需要看的东西。

## 2. MotorAxis 内部对象

`motor_axis.c` 里用一个静态对象管理所有内部状态：

```c
typedef struct {
    volatile MotorAxisStatus status;
    volatile uint8_t output_active;
    volatile uint8_t control_cycle_finished;
    MotorAxisMode mode;
    OpenLoopController open_loop;
    CurrentController current;
    FocController foc;
} MotorAxis;
```

几个变量要分清：

- `status`：给外部读取的快照。
- `output_active`：EN_GATE/PWM 输出是否已经真正启用。
- `control_cycle_finished`：上一拍控制计算是否已经完成并写好 PWM。
- `mode`：当前控制模式。
- `open_loop`：开环相位/电压控制器。
- `current`：电流 PI 控制器。
- `foc`：FOC/SVPWM 输出对象。

初始值：

```c
static MotorAxis motor_axis = {
    .control_cycle_finished = 1U,
    .mode = MOTOR_AXIS_MODE_IDLE,
};
```

为什么 `control_cycle_finished` 初始是 1？

因为刚上电还没有 pending 的电流采样，不应该一进中断就误判 deadline miss。

## 3. 写任务主循环

MotorAxis 是实时任务，每次由 ADC 真实电流采样唤醒。

```c
void motor_axis_task(void *argument)
{
    motor_axis_init();

    for (;;) {
        uint32_t flags = osThreadFlagsWait(MOTOR_AXIS_CURRENT_MEAS_FLAG,
                                           osFlagsWaitAny,
                                           CURRENT_MEAS_WATCHDOG_TIMEOUT_MS);

        if ((flags & MOTOR_AXIS_CURRENT_MEAS_FLAG) == 0U) {
            if (motor_axis.output_active != 0U) {
                motor_axis_disarm();
            }
            continue;
        }

        motor_axis_run_once();
    }
}
```

这里有两个时间概念：

- 正常 FOC 节拍靠 `MOTOR_AXIS_CURRENT_MEAS_FLAG` 唤醒，大约 8 kHz。
- `CURRENT_MEAS_WATCHDOG_TIMEOUT_MS = 10ms` 不是控制周期，只是 ADC/TIM 停掉时的看门狗。

## 4. 写 ADC ISR 到任务的握手

ADC ISR 在 `dir=0` 真实电流采样完成后调用：

```c
void motor_axis_signal_current_meas_from_isr(void)
{
    if ((motor_axis.output_active != 0U) &&
        (motor_axis.control_cycle_finished == 0U)) {
        motor_axis.status.runtime.deadline_miss_count++;
        motor_axis_disarm();
    }

    motor_axis.control_cycle_finished = 0U;

    if (motorAxisTaskHandle != 0) {
        osThreadFlagsSet(motorAxisTaskHandle, MOTOR_AXIS_CURRENT_MEAS_FLAG);
    }
}
```

这段是 ODrive 风格 deadline 检查的核心。

小白话：

```text
新的电流采样来了。
如果上一拍还没算完 PWM，说明控制线程没赶上。
已经在输出时，这属于危险情况，直接 disarm。
```

## 5. 写 motor_axis_run_once()

每次电流采样后，任务只做一件事：拿样本，跑当前模式，决定是否输出。

```c
static void motor_axis_run_once(void)
{
    PhaseCurrentSample sample;
    uint32_t controller_active;

    pwm_adc_get_phase_current_sample(&sample);

    motor_axis.status.phase_current.ia = sample.ia;
    motor_axis.status.phase_current.ib = sample.ib;
    motor_axis.status.phase_current.ic = sample.ic;

    controller_active = motor_axis_update_controller(&sample);
    motor_axis_update_output(controller_active);

    motor_axis.control_cycle_finished = 1U;
    motor_axis.status.runtime.loop_count++;
}
```

注意这里不要把控制逻辑全部写在 `run_once()` 里。它应该像调度器，只串流程。

## 6. 写模式分发

```c
static uint32_t motor_axis_update_controller(const PhaseCurrentSample *sample)
{
    switch (motor_axis.mode) {
    case MOTOR_AXIS_MODE_OPEN_LOOP_VOLTAGE:
        return motor_axis_run_open_loop_voltage();

    case MOTOR_AXIS_MODE_OPEN_LOOP_CURRENT_LOCKIN:
        return motor_axis_run_open_loop_current_lockin(sample);

    case MOTOR_AXIS_MODE_OPEN_LOOP_CURRENT_RUN:
        return motor_axis_run_open_loop_current(sample);

    case MOTOR_AXIS_MODE_IDLE:
    default:
        return 0U;
    }
}
```

以后加闭环速度/位置，也是在这里加 case。

不要让 USB 命令、ADC ISR、monitor 到处判断具体控制模式；统一让 `MotorAxis` 分发。

## 7. 先接开环电压模式

开环电压模式最简单：

```c
static uint32_t motor_axis_run_open_loop_voltage(void)
{
    return open_loop_controller_update(&motor_axis.open_loop,
                                       &motor_axis.foc,
                                       MOTOR_AXIS_PERIOD_SEC);
}
```

这一步验证：

```text
open 0.02 5
```

电机能低压缓慢转，说明：

- PWM/ADC 时序基本 OK
- FOC_voltage/SVPWM 基本 OK
- MotorAxis 任务唤醒 OK

## 8. 再接开环电流模式

开环电流模式的目的：暂时不用编码器，用开环相位给 Park 变换提供角度，先验证电流 PI。

```c
static uint32_t motor_axis_run_open_loop_current(const PhaseCurrentSample *sample)
{
    float current_phase;
    float pwm_phase;

    if (open_loop_controller_update_phase(&motor_axis.open_loop,
                                          MOTOR_AXIS_PERIOD_SEC) == 0U) {
        return 0U;
    }

    current_phase = motor_axis.open_loop.electrical_phase;
    pwm_phase = wrap_pm_pi(current_phase +
                           1.5f * MOTOR_AXIS_PERIOD_SEC *
                               motor_axis.open_loop.electrical_phase_vel);

    return current_controller_update(&motor_axis.current,
                                     &motor_axis.foc,
                                     sample,
                                     current_phase,
                                     pwm_phase,
                                     MOTOR_AXIS_PERIOD_SEC);
}
```

这里有两个角度：

- `current_phase`：当前采样电流对应的电角度，用于 Park。
- `pwm_phase`：预测 PWM 真正起作用时的电角度，用于逆 Park/SVPWM。

先用 ODrive 的 `1.5 * current_meas_period`。

## 9. 写输出使能和安全关闭

输出打开前要检查 offset 是否校准完成：

```c
static void motor_axis_update_output(uint32_t controller_active)
{
    PwmAdcStatus pwm_status;

    if (controller_active == 0U) {
        motor_axis_disarm();
        return;
    }

    pwm_adc_get_status(&pwm_status);

    if (motor_axis_output_ready(&pwm_status) == 0U) {
        motor_axis_disarm();
        return;
    }

    motor_axis_capture_status();
    motor_axis_arm_output();
}

static uint32_t motor_axis_output_ready(const PwmAdcStatus *pwm_status)
{
    if (pwm_status == 0) {
        return 0U;
    }

    return (pwm_status->offset_calibrated == PWM_ADC_OFFSET_CALIBRATED) ? 1U : 0U;
}
```

打开输出：

```c
static void motor_axis_arm_output(void)
{
    pwm_adc_set_gate_enabled(1U);
    motor_axis.output_active = 1U;
}
```

关闭输出：

```c
static void motor_axis_disarm(void)
{
    motor_axis.output_active = 0U;
    motor_axis.mode = MOTOR_AXIS_MODE_IDLE;
    open_loop_controller_set_enabled(&motor_axis.open_loop, 0U);
    current_controller_set_enabled(&motor_axis.current, 0U);
    pwm_adc_set_gate_enabled(0U);
    pwm_adc_write_pwm_neutral();
    motor_axis.control_cycle_finished = 1U;
    motor_axis_capture_status();
}
```

## 10. 再手撕 CurrentController 的状态

`CurrentController` 只管电流环，不管任务和硬件 gate。

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

含义：

- `config.phase_current_gain`：ADC raw count 到 A 的比例，当前还需要标定。
- `config.current_limit`：电流限制。
- `config.max_voltage_mod`：PI 输出的最大电压调制量。
- `pi.kp` / `pi.ki`：电流 PI 参数。
- `current.setpoint.d/q`：USB/上层给定的目标 Id/Iq。
- `current.ramped_setpoint.d/q`：经过软启动斜坡后的 Id/Iq，真正送入 PI。默认电流斜坡较慢，用来减小开环启动瞬间抖动。
- `current.measured.d/q`：滤波后的测量 Id/Iq，用于打印。
- `pi.integral_voltage.d/q`：PI 积分电压项。
- `pi.output_voltage.d/q`：最终输出给 FOC 的 Vd/Vq 调制量。
- `enabled`：电流环是否启用。
- `error`：电流环错误标志。

## 11. 初始化 CurrentController

```c
void current_controller_init(CurrentController *controller)
{
    controller->config.phase_current_gain = 0.01f;
    controller->config.current_limit = 5.0f;
    controller->config.max_voltage_mod = 0.08f;
    controller->pi.kp = 0.002f;
    controller->pi.ki = 1.0f;
    ...
}
```

这些值只是 bring-up 默认值，不是最终参数。

真正使用前要做：

1. 标定 `phase_current_gain`
2. 根据电机电阻/电感调 PI
3. 再设置严格 fault 策略

## 12. 写 set_target / set_enabled

`set_target()` 只负责目标值：

```c
controller->current.setpoint.d = clamp_float(id_setpoint, -limit, limit);
controller->current.setpoint.q = clamp_float(iq_setpoint, -limit, limit);
```

`set_enabled(0)` 要清积分项和输出：

```c
controller->current.ramped_setpoint.d = 0.0f;
controller->current.ramped_setpoint.q = 0.0f;
controller->pi.integral_voltage.d = 0.0f;
controller->pi.integral_voltage.q = 0.0f;
controller->pi.output_voltage.d = 0.0f;
controller->pi.output_voltage.q = 0.0f;
```

这样 stop 后不会残留 PI 积分。

## 13. 写 CurrentController update 主流程

电流环 update 的顺序固定：

```text
检查指针和 enabled
  -> Clarke/Park 得到 Id/Iq
  -> 检查电流限制
  -> 滤波保存 Id/Iq 给 monitor
  -> PI 算 Vd/Vq
  -> foc_controller_apply_voltage()
```

代码骨架：

```c
uint32_t current_controller_update(...){
    float id;
    float iq;

    current_controller_clarke_park(sample, gain, current_phase, &id, &iq);
    current_controller_check_limit(controller, id, iq);
    current_controller_update_pi(controller, id, iq, dt);

    return foc_controller_apply_voltage(foc_controller,
                                        controller->voltage_mod.d,
                                        controller->voltage_mod.q,
                                        pwm_phase);
}
```

## 14. Clarke/Park 变换

当前只采 B/C 两相，A 相由：

```text
ia = -ib - ic
```

ODrive 里常见写法：

```c
float ib = sample->ib * phase_current_gain;
float ic = sample->ic * phase_current_gain;
float i_alpha = -ib - ic;
float i_beta = ONE_BY_SQRT3 * (ib - ic);
```

Park：

```c
float c = arm_cos_f32(current_phase);
float s = arm_sin_f32(current_phase);

id = c * i_alpha + s * i_beta;
iq = c * i_beta - s * i_alpha;
```

直觉：

- `Id` 是磁场方向电流。
- `Iq` 是产生转矩的电流。

PMSM 常见目标：

```text
Id -> 0
Iq -> 需要的转矩
```

## 15. PI 控制

误差：

```c
err_d = id_setpoint - id;
err_q = iq_setpoint - iq;
```

比例 + 积分：

```c
vd = integrator_d + err_d * p_gain;
vq = integrator_q + err_q * p_gain;
```

限幅：

```c
v_mag = sqrtf(vd * vd + vq * vq);
if (v_mag > max_voltage_mod) {
    scale = max_voltage_mod / v_mag;
    vd *= scale;
    vq *= scale;
    integrator *= 0.99f;
} else {
    integrator_d += err_d * i_gain * dt;
    integrator_q += err_q * i_gain * dt;
}
```

为什么饱和时不继续积分？

因为 PWM 已经给不出更多电压了，如果继续积分，会产生 windup，后面恢复时会冲得很厉害。

## 16. USB 命令接法

`open`：

```text
open <voltage_mod> <electrical_rad_s>
```

调用：

```c
motor_axis_start_open_voltage(voltage, electrical_phase_vel);
```

`cur`：

```text
cur <iq_amp> <electrical_rad_s>
```

调用：

```c
motor_axis_start_open_current(iq_setpoint, electrical_phase_vel);
```

测试建议：

```text
cur 0.02 5
```

先看 monitor：

```text
mode=2
out=1
iq_sp_ma=20
iq_ramp_ma=...
id_ma=...
iq_ma=...
vd_milli=...
vq_milli=...
ierr=...
```

## 17. 推荐手撕顺序

真正从空文件写时，建议按这个顺序：

1. 写 `MotorAxisMode` 和 `MotorAxisStatus`。
2. 写 `MotorAxis` 内部 struct。
3. 写 `motor_axis_task()`，只等待 flag。
4. 写 `motor_axis_signal_current_meas_from_isr()`。
5. 写 `motor_axis_run_once()`。
6. 写 `motor_axis_update_controller()`，先只支持 idle。
7. 接 `OPEN_LOOP_VOLTAGE`，确认 `open` 能转。
8. 写 `CurrentController` struct 和 init。
9. 写 `current_controller_set_target()` / `set_enabled()`。
10. 写 Clarke/Park，先只打印 Id/Iq，不闭环。
11. 写 PI，但先让 `max_voltage_mod` 很小。
12. 接 `OPEN_LOOP_CURRENT_LOCKIN` / `OPEN_LOOP_CURRENT_RUN`。
13. 加 USB `cur` 命令。
14. 看 monitor 的 `id_ma/iq_ma/vd_milli/vq_milli`。
15. 再考虑 fault、标定、编码器。

## 18. 当前版本还不是最终闭环

现在这版电流环只是 bring-up 阶段：

- `phase_current_gain` 还没按硬件标定。
- PI 参数只是保守默认值。
- `cur` 仍然使用开环相位，不是真实编码器相位。
- 电流 limit 目前更偏提示，不是最终硬保护。

下一步应该是：

1. 标定 ADC count 到 A。
2. 用电机 R/L 重新算 PI。
3. 接编码器角度。
4. 再做真正电流闭环。
5. 最后速度环、位置环。
