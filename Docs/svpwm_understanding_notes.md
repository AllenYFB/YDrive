# YDrive SVPWM 周期与扇区理解笔记

本文使用 YDrive 当前工程的实际 TIM1 参数来解释 SVPWM、PWM 周期、电角度、机械角度和扇区之间的关系。

当前工程相关配置来自：

```text
Board/Core/Src/main.c
Board/Core/Src/tim.c
```

## 1. YDrive 当前 TIM1 参数

当前系统时钟配置：

```text
SYSCLK = 168 MHz
APB2 prescaler = /2
PCLK2 = 84 MHz
TIM1 clock = 168 MHz
```

STM32F4 中，如果 APB prescaler 不是 1，那么定时器时钟是 PCLK 的 2 倍。因此 TIM1 实际计数频率是：

```text
TIM1_CLK = 168 MHz
```

当前 TIM1 配置：

```text
Prescaler         = 0
CounterMode       = CenterAligned3
ARR / Period      = 3500
RepetitionCounter = 2
PWM Mode          = PWM2
```

TIM1 一个计数 tick 的时间：

```text
1 / 168 MHz = 5.952 ns
```

中心对齐 PWM 会先从 `0 -> ARR`，再从 `ARR -> 0`。

半个计数行程：

```text
3500 / 168 MHz = 20.833 us
```

一个完整中心对齐 PWM 载波周期：

```text
2 * 3500 / 168 MHz = 41.667 us
```

对应 PWM 载波频率：

```text
1 / 41.667 us ≈ 24 kHz
```

所以 YDrive 当前不是 25 us / 40 kHz，而是：

```text
PWM 载波周期 ≈ 41.667 us
PWM 载波频率 ≈ 24 kHz
```

## 2. YDrive 当前 ADC/控制节拍

TIM1 配置了：

```text
TIM1_TRGO = Update Event
RepetitionCounter = 2
```

中心对齐计数中，update 事件与向上/向下计数边界相关。当前工程实测 USB 输出：

```text
adc_irq=48000
current=7999
```

`adc_irq=48000` 的含义是：

```text
ADC1 + ADC2 + ADC3 总回调次数 ≈ 48000 次/秒
```

因为每次 TIM1_TRGO 会触发：

```text
ADC1: VBUS
ADC2: IB
ADC3: IC
```

所以 TIM1_TRGO/ADC injected 触发频率约为：

```text
48000 / 3 = 16000 Hz
```

也就是：

```text
ADC injected 触发周期 ≈ 62.5 us
ADC injected 触发频率 ≈ 16 kHz
```

YDrive 当前代码在 ADC 回调中判断：

```c
TIM1->CR1 & TIM_CR1_DIR
```

含义：

```text
dir = 0：向上计数，作为真实电流采样窗口
dir = 1：向下计数，作为 DC_CAL/零偏校准窗口
```

因此真实电流采样频率约为 ADC 触发频率的一半：

```text
16000 / 2 = 8000 Hz
```

这就解释了实测：

```text
current=7999
```

所以 YDrive 当前有三种“周期”要区分：

```text
TIM1 PWM 载波周期       ≈ 41.667 us  ≈ 24 kHz
ADC injected 触发周期   ≈ 62.5 us    ≈ 16 kHz
真实电流控制采样周期    ≈ 125 us     ≈ 8 kHz
```

后续如果把 FOC 电流环接到 `ControlLoopTask`，那么当前默认控制环更新频率就是：

```text
约 8 kHz
```

## 3. 为什么 ODrive 区分向上计数和向下计数

ODrive 原工程在 `pwm_trig_adc_cb()` 里有这段判断：

```c
counting_down = TIMx->CR1 & TIM_CR1_DIR;
current_meas_not_DC_CAL = !counting_down;
```

它的注释含义是：

```text
向上计数：刚刚采到 SVM vector 0，作为真实电流
向下计数：刚刚采到 SVM vector 7，作为 zero current / DC_CAL
```

这里不要简单理解成“因为 PWM mode2，所以这样采样”。更准确地说，这是下面几件事共同决定的：

```text
中心对齐 PWM
PWM2 输出模式
互补 PWM 输出和死区
SVPWM 的零矢量排列
TIM update 触发 ADC
低边/相电流采样硬件
```

PWM mode2 会影响比较值和 PWM 输出电平之间的关系，但它本身不是全部原因。ODrive 真正利用的是：中心对齐 PWM 在一个完整载波周期内天然有向上计数和向下计数两个半周期，而 SVPWM 可以把两个零矢量窗口安排在这两个半周期中。这样 ADC 每次由 TIM update 硬件触发时，就能根据当前 `DIR` 判断这次采样落在哪一种窗口里。

ODrive 的设计目标不是“随便找个时刻采 ADC”，而是找一个确定、安静、可重复的采样时刻：

```text
避开 MOS 开关瞬间
避开死区切换瞬间
避开电流尖峰和振铃
保证 ADC2/ADC3 同一时刻采 IB/IC
保证控制线程被真实电流采样唤醒
```

在 ODrive 的时序里，向上计数那次采样被认为处在可以表示真实相电流的零矢量窗口，所以 ADC2 先保存 `IB`，ADC3 再保存 `IC`，然后唤醒控制线程。向下计数那次采样被认为处在 zero current 窗口，用来慢慢更新电流采样零偏，也就是 `DC_calib`。

因此 `ADC injected ≈ 16 kHz` 不等于 FOC 电流环也跑 `16 kHz`。它可以理解成：

```text
ADC injected 总触发      ≈ 16 kHz
真实电流采样/控制唤醒    ≈ 8 kHz
DC_CAL/零偏更新          ≈ 8 kHz
```

向下计数时进入 ADC 中断仍然有用，因为这次中断不是为了唤醒 FOC，而是为了读取当前采样链路的零点。电流采样链路里有运放偏置、ADC 偏置、温漂和电源噪声，如果只在上电时校准一次，运行一段时间后零点可能会慢慢漂。ODrive 在运行中持续利用 zero current 窗口更新 `DC_calib`，这样后面真实电流采样时可以一直减掉最新的零偏。

按照常见 SVPWM 开关状态定义：

```text
SVM vector 0 = 000：三相下桥臂导通，三相端子都接到 GND
SVM vector 7 = 111：三相上桥臂导通，三相端子都接到 VBUS
```

ODrive v3 这类低边采样结构里，真实相电流适合在 `000` 这个零矢量窗口采。因为三相下桥臂导通时，电机绕组电流会通过低边采样路径，ADC 能看到有意义的相电流。到了 `111` 这个零矢量窗口，三相端子被上桥臂接到母线，低边采样路径理论上没有真实相电流流过，ADC 读到的主要就是采样链路自身的偏置，所以适合拿来做 `DC_CAL`。

所以 ODrive 选择这个采样点的核心原因是：

```text
vector 0：低边采样能看到真实相电流
vector 7：低边采样理论上看到零电流，适合校零
```

再加上这两个点都属于零矢量窗口，电机端三相被短接到同一个电位，开关状态比较稳定，比在有效矢量切换边沿附近采样更干净。

所以它不是单纯为了“每次都在下桥 MOS 导通时采样”。对于三相桥来说，某一瞬间到底是哪几个上管/下管导通，还和 PWM2、互补输出极性、死区、驱动器输入逻辑、SVPWM 当前扇区有关。更稳妥的理解是：

```text
真实电流窗口：这个窗口里的采样值能代表当前相电流
DC_CAL 窗口：这个窗口理论上应该接近零电流，用来跟踪 ADC/运放偏置
```

这样做的好处是：

```text
真实电流采样时刻固定，FOC 计算节拍稳定
零偏可以持续跟踪温漂和运放/ADC 漂移
采样避开开关噪声，电流值更干净
ADC 中断可以直接成为控制线程的节拍源
```

对于 YDrive 当前单电机工程，可以先沿用 ODrive 的简化规则：

```text
dir = 0：真实电流采样，计算 ia/ib/ic，并唤醒控制线程
dir = 1：DC_CAL/零偏更新，不唤醒控制线程
```

后续真正上功率、接电机时，最好用示波器同时看：

```text
TIM1 PWM 输出
ADC 触发点
相电流采样运放输出
```

确认真实采样点确实落在干净、有效的电流采样窗口里。

### 直接理解版

`ADC injected ≈ 16 kHz` 里，一半是：

```text
dir = 0：真实电流采样，唤醒控制线程，FOC 跑一次
```

另一半是：

```text
dir = 1：进入 ADC 中断，但不跑 FOC，只更新 DC_CAL/零偏
```

所以向下计数进 ADC 中断的作用是：持续跟踪电流采样链路的零点漂移。运放、ADC、温度、电源都会让零点慢慢变，ODrive 用这半拍来更新 offset。

常见 SVPWM 定义里：

```text
SVM vector 0 = 000：三相下桥臂导通
SVM vector 7 = 111：三相上桥臂导通
```

ODrive v3 是低边/相电流采样结构，所以：

```text
vector 0：下桥臂导通，低边采样路径能看到真实相电流
vector 7：上桥臂导通，低边采样理论上接近零电流，适合做 DC_CAL
```

因此它取这个采样点的原因不是“PWM2 本身神奇”，而是为了在稳定的零矢量窗口里采样：

```text
vector 0 采真实电流
vector 7 采零偏
```

### CCR preload 与控制流水线

还要注意：CCR 的新值不是由 `DIR` 这个标志直接装载的，而是由 `TIM update event` 装载的。

但是在当前这种节奏里，`dir = 0` 那次 ADC 之后才会唤醒控制线程，线程算完 FOC/SVM 后把新的 CCR 写入 preload。最近的下一次 update 通常正好对应 `dir = 1`，所以可以这样理解这个流水线：

```text
Update A，dir = 0
  -> ADC 采真实电流
  -> 跑 FOC
  -> 写 CCR preload

Update B，dir = 1
  -> TIM update 事件到来
  -> 硬件装载上次写好的 CCR
  -> ADC 采 zero current
  -> 更新 DC_CAL
  -> 不跑 FOC

Update C，dir = 0
  -> ADC 采真实电流
  -> 跑 FOC
  -> 写 CCR preload

Update D，dir = 1
  -> TIM update 事件到来
  -> 硬件装载上次写好的 CCR
  -> ADC 采 zero current
  -> 更新 DC_CAL
  -> 不跑 FOC
```

所以不是“不规律”，而是稳定交替：

```text
dir = 0：反馈采样 + 控制计算
dir = 1：PWM 更新生效 + 零偏校准
dir = 0：反馈采样 + 控制计算
dir = 1：PWM 更新生效 + 零偏校准
```

一句话记忆：

```text
dir = 0 负责“看电流并计算”
dir = 1 负责“让上次计算结果生效并校零”
```

更底层、更准确的规则仍然是：

```text
CCR preload 在 TIM update event 时统一装载
DIR 只用来判断这次 ADC 采样属于真实电流窗口还是 DC_CAL 窗口
```

## 4. PWM 周期不是电机转过的角度

PWM 周期只是逆变器刷新电压的时间步长。

在 YDrive 当前工程里，一个 PWM 载波周期约为：

```text
41.667 us
```

真实电流采样和控制线程唤醒约为：

```text
125 us
```

这些都是时间，不是角度。

一个 PWM 周期不等于电机走过一个 SVPWM 扇区，也不等于电机转过一圈。

可以这样理解：

```text
PWM 载波周期：三相桥高速开关的基础节拍
ADC 触发周期：硬件采样电流的节拍
控制采样周期：FOC 后续计算新电压矢量的节拍
```

## 5. SVPWM 的 6 个扇区是电角度区域

SVPWM 的 6 个扇区覆盖一圈电角度：

```text
360 电角度 / 6 = 每个扇区 60 电角度
```

扇区只是表示当前目标电压矢量落在哪个 60 电角度区域中。

它不表示：

```text
一个 PWM 周期走一个扇区
```

真实情况是：

```text
一个扇区可能经历很多个 PWM 周期
```

经历多少个 PWM 周期，取决于：

```text
电机转速
极对数
PWM 载波频率
控制环更新频率
```

## 6. 电角度和机械角度

FOC/SVPWM 用的是电角度。

关系式：

```text
电角度 = 机械角度 * 极对数
```

如果电机是 7 极对，那么机械转一圈时：

```text
机械角度 = 360 度
电角度 = 360 * 7 = 2520 度
```

也就是电角度转了 7 圈。

机械一圈经过的 SVPWM 扇区数量：

```text
6 * 极对数
```

例如 7 极对：

```text
6 * 7 = 42 个电角度扇区
```

## 7. 按 YDrive 当前周期计算一个例子

假设：

```text
PWM 载波频率 = 24 kHz
控制采样频率 = 8 kHz
电机转速 = 1000 rpm
极对数 = 7
```

机械转速：

```text
1000 rpm = 16.667 转/秒
```

机械一圈时间：

```text
1 / 16.667 = 0.06 s = 60 ms
```

机械一圈内包含多少个 PWM 载波周期：

```text
60 ms / 41.667 us ≈ 1440 个 PWM 载波周期
```

机械一圈内包含多少个控制采样周期：

```text
60 ms / 125 us = 480 个控制采样周期
```

因为 7 极对电机机械一圈包含 7 圈电角度，所以一圈电角度包含：

```text
480 / 7 ≈ 68.6 个控制采样周期
```

一个 SVPWM 扇区是 60 电角度，也就是一圈电角度的 1/6：

```text
68.6 / 6 ≈ 11.4 个控制采样周期
```

如果按 PWM 载波周期计算：

```text
1440 / 7 / 6 ≈ 34.3 个 PWM 载波周期/扇区
```

所以在这个例子里：

```text
一个 SVPWM 扇区约经历 34 个 PWM 载波周期
一个 SVPWM 扇区约经历 11 个控制采样周期
```

这说明：

```text
不是 6 个 PWM 周期电机转一圈
也不是一个扇区只持续一个 PWM 周期
```

## 8. 一个 SVPWM 周期里如何合成空间矢量

在某个控制采样时刻，FOC 会得到一个目标电压矢量：

```text
幅值
电角度
```

SVPWM 根据这个电角度判断当前处于哪个扇区，然后选择相邻两个有效矢量和零矢量：

```text
V1
V2
V0/V7
```

并计算：

```text
T1
T2
T0
```

满足：

```text
T1 + T2 + T0 = 当前用于合成的 PWM 时间窗口
```

含义：

```text
V1 作用 T1 时间
V2 作用 T2 时间
零矢量作用 T0 时间
```

这三个时间比例在一个高速开关周期内形成一个平均电压矢量。

电机绕组有电感，会把高频开关动作滤波，所以电机主要响应的是平均电压和平均电流趋势。

## 9. 每个控制周期占空比会逐渐变化

目标电压矢量的电角度是连续变化的。

例如在某个扇区内，电角度可能这样变化：

```text
第 1 次控制更新：10.0 度
第 2 次控制更新：10.9 度
第 3 次控制更新：11.8 度
```

每次控制更新都会重新计算：

```text
T1
T2
T0
三相占空比
```

因此占空比是逐步变化的。

在一个扇区内可以粗略理解为：

```text
靠近扇区起点：T1 多，T2 少
扇区中间：    T1 和 T2 比较均衡
靠近扇区末尾：T1 少，T2 多
```

也就是说：

```text
T1/T2/T0 随电角度变化
三相占空比随电角度变化
很多个 PWM 周期连起来，平均电压矢量表现为平滑旋转
```

## 10. 不要混淆这几个周期

YDrive 当前工程里，至少要区分：

```text
PWM 载波周期       ≈ 41.667 us  ≈ 24 kHz
ADC injected 周期  ≈ 62.5 us    ≈ 16 kHz
真实电流采样周期   ≈ 125 us     ≈ 8 kHz
电角度周期         由电机电角速度决定
机械角度周期       由电机机械转速决定
```

它们不是同一个东西。

`SVPWM 扇区` 是电角度区域，不是固定时间。

`PWM 周期` 是开关时间单位，不是角度单位。

## 11. 最终记忆版

最重要的几句话：

```text
YDrive 当前 PWM 载波约 24 kHz，周期约 41.667 us。
YDrive 当前 ADC injected 约 16 kHz，周期约 62.5 us。
YDrive 当前真实电流采样/控制唤醒约 8 kHz，周期约 125 us。
SVPWM 扇区是 60 电角度，不是一个 PWM 周期。
一个扇区会经历多少 PWM 周期，取决于电机转速和极对数。
每次控制更新都会根据当前电角度重新计算三相占空比。
机械转一圈需要经过 极对数 圈电角度。
```
