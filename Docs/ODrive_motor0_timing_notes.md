# ODrive motor0 时序笔记

这篇笔记只看 ODrive 0.5.1 的 **motor0**，把 `TIM1`、`ADC2/ADC3`、`encoder`、`FOC`、`axis0 线程` 放到同一条时间线上，帮助后面复刻时快速建立直觉。

相关代码主要在这些文件里：

- `Firmware/Board/v3/Odrive.ioc`
- `Firmware/Board/v3/Src/stm32f4xx_it.c`
- `Firmware/MotorControl/low_level.cpp`
- `Firmware/MotorControl/axis.cpp`
- `Firmware/MotorControl/axis.hpp`
- `Firmware/MotorControl/encoder.cpp`
- `Firmware/MotorControl/controller.cpp`
- `Firmware/MotorControl/motor.cpp`

## 1. 先记住总原则

motor0 不是“一个函数从头跑到尾”，而是“硬件和线程分工”。

- `TIM1` 负责打拍子
- `ADC2/ADC3` 负责在合适的时刻采电流
- `ADC_IRQHandler()` 负责把采样结果分发到回调
- `signal_current_meas()` 负责叫醒 axis0 线程
- axis0 线程负责更新编码器、控制器、FOC，并写下一拍 PWM

所以更准确的说法是：

**PWM 一直在跑，ADC 在硬件触发下采样，线程被采样完成事件唤醒后再算控制。**

## 2. ADC 不是软件“跳过去”的，而是硬件触发的

在 `Odrive.ioc` 里，motor0 的 ADC injected trigger 配成了 `TIM1_TRGO`：

- `ADC2.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_TRGO`
- `ADC3.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_TRGO`

同时 `TIM1` 的 master trigger 配成了 update：

- `TIM1.TIM_MasterOutputTrigger = TIM_TRGO_UPDATE`

这意味着：

1. `TIM1` 到了 update 点
2. `TIM1` 通过 TRGO 发出触发
3. `ADC2/ADC3` 自动开始 injected conversion
4. 转换结束后进入 `ADC_IRQHandler()`

这条链路没有“软件手动跳转 ADC”这一步，核心是硬件触发。

## 3. ADC 中断里怎么进到 motor0 的电流采样

在 `Firmware/Board/v3/Src/stm32f4xx_it.c` 里：

```c
void ADC_IRQHandler(void)
{
  ADC_IRQ_Dispatch(&hadc1, &vbus_sense_adc_cb);
  ADC_IRQ_Dispatch(&hadc2, &pwm_trig_adc_cb);
  ADC_IRQ_Dispatch(&hadc3, &pwm_trig_adc_cb);
  return;
}
```

意思是：

- `hadc1` 走母线电压采样回调
- `hadc2` 和 `hadc3` 都走 `pwm_trig_adc_cb()`

所以 motor0 的电流采样回调不是“自动冒出来”的，而是：

`TIM1_TRGO -> ADC2/ADC3 -> ADC_IRQHandler -> ADC_IRQ_Dispatch -> pwm_trig_adc_cb`

## 4. `signal_current_meas()` 是怎么被触发的

在 `Firmware/MotorControl/low_level.cpp` 的 `pwm_trig_adc_cb()` 里：

- `ADC2` 先到，先存 `phB`
- `ADC3` 后到，再存 `phC`
- 当两路电流都到齐后，调用 `axis.signal_current_meas()`

而 `Axis::signal_current_meas()` 在 `Firmware/MotorControl/axis.cpp` 里只是做一件事：

```c
osSignalSet(thread_id_, M_SIGNAL_PH_CURRENT_MEAS);
```

这就像是给 axis0 线程按一下门铃。

## 5. axis0 线程是怎么被唤醒的

axis0 线程在 `Firmware/MotorControl/axis.hpp` 的 `run_control_loop()` 里会在每一拍末尾等待信号：

```c
wait_for_current_meas();
```

它内部是：

```c
osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT)
```

所以真实流程是：

1. ADC 回调里 `osSignalSet()`
2. axis0 线程从 `osSignalWait()` 返回
3. 线程继续做下一拍的更新

这不是“跳函数”，而是 RTOS 的事件唤醒。

## 6. motor0 一个 PWM 周期里都发生什么

可以把 motor0 的一拍理解成下面这样：

```text
TIM1 在跑 PWM
  |
  |-- 到 update 点
  |     -> 产生 TRGO
  |     -> 触发 ADC2/ADC3 injected conversion
  |
  |-- ADC2/ADC3 转换完成
  |     -> 进入 ADC_IRQHandler()
  |     -> pwm_trig_adc_cb()
  |     -> 读出 phB / phC
  |     -> 算出 phA
  |     -> signal_current_meas()
  |
  |-- axis0 线程醒来
  |     -> do_updates()
  |        -> encoder_.update()
  |        -> sensorless_estimator_.update()
  |     -> controller_.update()
  |     -> motor_.update()
  |        -> Park / 逆Park / PI / SVM
  |        -> 写下一拍 CCR1/CCR2/CCR3
  |
  |-- 下一次 update 点
        -> 硬件把新 CCR 真正用于输出
```

## 7. 编码器在哪里算

motor0 的编码器不是在 PWM 中断里直接算完的。

更准确地说：

- `tim_update_cb()` 先在 PWM update 点抓一次快照
- axis0 线程醒来后，在 `Encoder::update()` 里正式更新位置和速度估计

也就是说：

**先采样，后计算。**

这和“中断里直接把控制全做完”的写法不一样。

## 8. FOC 在哪里做

FOC 的核心逻辑在 `Firmware/MotorControl/motor.cpp` 的 `Motor::update()` 里。

它做的事情可以按顺序记：

1. 把转矩目标换成电流目标
2. 用编码器算出的电角度做 Park 变换
3. 计算电流误差
4. PI 电流环
5. 逆 Park
6. SVM
7. 写下一拍 PWM

一句话概括：

**FOC 就是把“想要的力气”翻译成下一拍三相 PWM。**

## 9. PWM 占空比很小的时候，MOS 管到底是什么状态

这是很容易误解的地方。

PWM 不是“开/关总按钮”，而是定时器比较结果决定哪段时间导通。

对 motor0 来说：

- `TIM1` 是中心对齐 PWM
- `TIM1` 使用 `TIM_OCMODE_PWM2`
- MOS 管的最终开关由比较器、死区和驱动器共同决定

所以：

- 占空比为 0，不代表电机回路被“物理抹掉”
- 只是主动驱动几乎没有了
- 电流仍可能通过二极管、另一相绕组、同步整流路径流动

也就是说：

**PWM 小，不等于电流测不到；只是采样窗口会变窄，噪声更容易进来。**

## 10. 向上计数和向下计数怎么理解

中心对齐 PWM 可以把一个周期理解成两半：

- `CNT` 从 `0 -> ARR`：向上计数
- `CNT` 从 `ARR -> 0`：向下计数

对 motor0 来说，代码里会根据 `TIM_CR1_DIR` 判断当前方向。

### 向上计数

更适合把它理解成“真实电流采样窗口”。

### 向下计数

更适合把它理解成“另一侧窗口”或者“校准/配合窗口”。

代码里就是用这个方向来区分：

- 什么时候当成真实电流采样
- 什么时候当成另一种校准采样

## 11. 中心对齐 + PWM mode2 时，PWM 波形到底长什么样

ODrive 的 motor0 用的是：

- **中心对齐计数**
- **PWM mode2**
- **TIM1 / TIM8 的互补输出**
- **带死区时间**

可以先把它想成一条“来回跑”的锯齿形计数器：

```text
CNT:
0        /\
        /  \
       /    \
      /      \
ARR  /        \
    /          \
0  /            \
   向上计数      向下计数
```

然后再看 PWM2 的含义。  
在硬件比较器里，`CNT` 和 `CCR` 做比较，决定当前这一瞬间输出该处于“有效”还是“无效”状态。

你可以先用最直白的话记：

- `CCR` 决定“这一相有多宽的导通窗口”
- `CNT` 是当前时间点
- `PWM2` 是一种比较方式，和 `PWM1` 相比逻辑是反过来的

所以在波形上，**PWM2 更像是把 PWM1 反相了一下**。  
如果不用太死抠寄存器细节，理解上可以直接记成：

不过在 ODrive 里，不能只盯着“比较器输出高低”，还要再经过：

1. **通道极性配置**
2. **互补输出 CHxN**
3. **死区时间**
4. **MOS 驱动器**

所以最后真正到 MOS 管上的高低电平，不是“软件直接画出来的方波”，而是上面这几层共同作用的结果。

### 一个更适合新手的看法

对电机来说，你更应该把它看成“每个 PWM 周期里，有一段时间这相被积极驱动，另一段时间不驱动或者交给互补管处理”。

比如某一相的占空比很大：

- 这一相的有效导通窗口就宽
- MOS 管在大部分周期里保持驱动状态

某一相的占空比很小：

- 这一相的有效导通窗口就窄
- MOS 管只在很短的时间片里切换

### MOS 管到底怎么开关

对半桥来说，真正的开关不是“这一拍整个周期都开/关”，而是：

- 上管和下管不会同时导通
- 两者之间会插入 **deadtime**
- 在一个 PWM 周期里，比较器决定当前应该让哪一侧导通

所以你可以把它理解成：

```text
一个周期里，半桥在“上管导通”和“下管导通”之间来回切换
中间故意留一小段空白，避免上下管打架
```

### 为什么采样通常放在中间

因为开关瞬间最吵：

- MOS 管刚翻转时，电压和电流变化最快
- 二极管反向恢复、寄生电感、驱动延迟都会带来噪声

所以 ODrive 会把电流采样尽量放在“比较平稳”的时间点，而不是刚开刚关的时候。

这也是为什么中心对齐 PWM 很适合电机控制：

- 波形前后对称
- 采样点更容易放在中间
- 电流测量更稳定

### 如果占空比为 0，会怎样

占空比为 0 时，不代表“电机完全没有电流路径”，而是：

- 这一相几乎不主动输出
- 但绕组电流仍可能通过别的相、体二极管、同步整流路径存在
- 所以“PWM 为 0”不等于“电流一定是 0”

对控制来说，这意味着：

- 输出能力很弱
- 采样窗口变窄
- 误差和噪声更容易影响测量

### 如果占空比很小，MOS 管是在什么时候打开

答案是：

**不是在“计数器开始向上计数”的那一刻才开，也不是“一有占空比就持续开着”。**

而是定时器在整个周期里按照 CCR 比较结果，决定在哪一段时间导通。  
对 ODrive 来说，还要叠加互补输出和死区，所以真正的 MOS 开关时刻由硬件自动完成。

你可以把它记成一句更顺口的话：

**PWM 不是手动按开关，而是定时器按规则在周期里安排开关。**

## 12. 最终记忆版

如果只记一句话，建议记这个：

**motor0 的一个周期里，TIM1 打拍，ADC 硬件采样，ADC 中断唤醒 axis0 线程，线程里算编码器和 FOC，最后写下一拍 PWM。**

如果再压缩一点：

**硬件负责采样，线程负责计算，PWM 负责执行。**

## 13. 为什么这样设计

这样做的好处是：

- 采样时刻更准
- 控制节拍更稳
- 编码器、控制器、保护逻辑分层清楚
- 比把所有东西都堆进一个中断里更容易扩展

对后面复刻来说，这个结构最值得保留。
