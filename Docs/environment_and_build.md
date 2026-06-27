# YDrive 环境配置与 VSCode 编译运行说明

本文只记录开发环境、新工程在 VSCode 中如何编译/烧录/调试，以及哪些工程文档需要自己维护。

## 1. 目录约定

仓库根目录：

```powershell
C:\Users\mrg\Desktop\PMSMFocLib\YDrive
```

STM32 CubeMX/CMake 工程目录：

```powershell
C:\Users\mrg\Desktop\PMSMFocLib\YDrive\Board
```

注意：`CMakePresets.json` 在 `Board` 目录里，所以执行 CMake 命令时要进入 `Board`。

正确：

```powershell
cd C:\Users\mrg\Desktop\PMSMFocLib\YDrive\Board
```

错误：

```powershell
cd C:\Users\mrg\Desktop\PMSMFocLib\YDrive
cmake --preset Debug
```

## 2. 需要自己安装的软件

### 2.1 STM32CubeMX

用于打开和修改 `.ioc`：

```powershell
C:\Users\mrg\Desktop\PMSMFocLib\YDrive\Board\YDrive.ioc
```

当前工程使用：

```text
STM32CubeMX v6.16.0
STM32Cube FW_F4 V1.28.3
```

如果新工程是 CubeMX 生成的，建议统一选择：

```text
Toolchain / IDE = CMake
Generator = Ninja
```

### 2.2 CMake

检查命令：

```powershell
cmake --version
```

当前验证版本：

```text
cmake version 3.26.0
```

### 2.3 Ninja

检查命令：

```powershell
ninja --version
```

当前验证版本：

```text
1.13.2
```

### 2.4 GNU Arm Embedded Toolchain

检查命令：

```powershell
arm-none-eabi-gcc --version
arm-none-eabi-gdb --version
```

当前验证版本：

```text
arm-none-eabi-gcc 10.3-2021.10
```

需要把工具链的 `bin` 目录加入 Windows `PATH`，例如：

```text
D:\gcc-arm-none-eabi-10.3-2021.10\bin
```

### 2.5 OpenOCD

用于 STLINK 烧录和调试。

检查命令：

```powershell
openocd --version
```

当前验证路径：

```text
D:\OpenOCD-20240916-0.12.0\bin\openocd.exe
```

需要把 OpenOCD 的 `bin` 目录加入 Windows `PATH`。

### 2.6 VSCode 扩展

建议安装：

```text
C/C++
CMake Tools
Cortex-Debug
```

作用：

```text
C/C++         代码补全、跳转、诊断
CMake Tools   配合 CMake 工程
Cortex-Debug  使用 STLINK/OpenOCD 调试 Cortex-M
```

## 3. 命令行编译方法

进入工程目录：

```powershell
cd C:\Users\mrg\Desktop\PMSMFocLib\YDrive\Board
```

第一次配置或清理后重新配置：

```powershell
cmake --fresh --preset Debug
```

编译：

```powershell
cmake --build --preset Debug
```

输出目录：

```powershell
C:\Users\mrg\Desktop\PMSMFocLib\YDrive\Board\build\Debug
```

主要产物：

```text
YDrive.elf
YDrive.bin
YDrive.hex
YDrive.map
```

用途：

```text
YDrive.elf  调试用，包含符号信息
YDrive.bin  裸二进制固件
YDrive.hex  Intel HEX 固件
YDrive.map  查看 FLASH/RAM 占用和符号分布
```

## 4. 新工程在 VSCode 中需要写哪些配置

如果是一个新的 STM32 CMake 工程，建议自己维护 `.vscode` 下的三个文件：

```text
.vscode/tasks.json
.vscode/launch.json
.vscode/c_cpp_properties.json
```

### 4.1 tasks.json

需要自己写。

用途：

```text
配置 Debug
编译 Debug
探测 STLINK
烧录固件
```

YDrive 当前任务：

```text
YDrive: configure Debug
YDrive: build Debug
YDrive: probe STLINK
YDrive: flash with STLINK
```

关键点：

```json
"cwd": "${workspaceFolder}/Board"
```

因为 CMake 工程在 `Board`，不是仓库根目录。

烧录任务核心命令：

```powershell
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "adapter speed 1000" -c "program build/Debug/YDrive.elf verify reset exit"
```

新工程需要改：

```text
工程目录
elf 文件名
target 配置，例如 stm32f4x.cfg / stm32g4x.cfg / stm32h7x.cfg
```

### 4.2 launch.json

需要自己写。

用途：

```text
让 VSCode 用 Cortex-Debug + OpenOCD + STLINK 调试 MCU
```

YDrive 当前调试配置：

```text
YDrive: Debug STLINK/OpenOCD
```

关键字段：

```json
"type": "cortex-debug",
"servertype": "openocd",
"executable": "${workspaceFolder}/Board/build/Debug/YDrive.elf",
"configFiles": [
    "interface/stlink.cfg",
    "target/stm32f4x.cfg"
],
"runToEntryPoint": "main",
"preLaunchTask": "YDrive: build Debug"
```

新工程需要改：

```text
name
executable
device
target/*.cfg
preLaunchTask
```

是否一定要写 `launch.json`：

```text
只编译/烧录：不一定需要
要在 VSCode 里 F5 调试、断点、单步：需要
```

### 4.3 c_cpp_properties.json

建议自己写。

用途：

```text
让 VSCode IntelliSense 知道芯片宏、HAL/CMSIS/FreeRTOS include 路径和交叉编译器
```

如果不写，常见现象：

```text
__IO 未定义
RCC_CR_HSEON 未定义
TIM14 未定义
FLASH_LATENCY_5 未定义
代码能编译，但 VSCode 红线很多
```

YDrive 需要的关键宏：

```json
"defines": [
    "USE_HAL_DRIVER",
    "STM32F405xx",
    "DEBUG"
]
```

YDrive 需要的关键 include：

```text
MotorControl
Board/Core/Inc
Board/Drivers/STM32F4xx_HAL_Driver/Inc
Board/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
Board/Drivers/CMSIS/Device/ST/STM32F4xx/Include
Board/Drivers/CMSIS/Include
Board/Middlewares/Third_Party/FreeRTOS/Source/include
Board/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
Board/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
Board/Middlewares/ST/STM32_USB_Device_Library/Core/Inc
Board/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
```

修改后可以在 VSCode 执行：

```text
Ctrl+Shift+P
C/C++: Reset IntelliSense Database
```

## 5. VSCode 中如何编译运行

打开仓库根目录：

```powershell
code C:\Users\mrg\Desktop\PMSMFocLib\YDrive
```

编译：

```text
Terminal -> Run Task -> YDrive: build Debug
```

探测 STLINK：

```text
Terminal -> Run Task -> YDrive: probe STLINK
```

烧录：

```text
Terminal -> Run Task -> YDrive: flash with STLINK
```

调试：

```text
Run and Debug -> YDrive: Debug STLINK/OpenOCD -> F5
```

如果只想用命令行，不用 VSCode 任务，也可以直接执行：

```powershell
cd C:\Users\mrg\Desktop\PMSMFocLib\YDrive\Board
cmake --build --preset Debug
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "adapter speed 1000" -c "program build/Debug/YDrive.elf verify reset exit"
```

## 6. STLINK 接线与常见连接问题

最少接线：

```text
STLINK SWDIO  -> MCU SWDIO
STLINK SWCLK  -> MCU SWCLK / SWC
STLINK GND    -> 目标板 GND
STLINK VTref  -> 目标板 3.3V
```

目标板需要稳定 3.3V。

OpenOCD 正常连接时会看到：

```text
Target voltage: 3.xxx
target halted
```

如果看到：

```text
target voltage may be too low
init mode failed
```

优先检查：

```text
目标板 3.3V 是否正常
STLINK GND 是否和目标板 GND 共地
STLINK VTref 是否接到目标板 3.3V
SWDIO/SWCLK 是否接反
```

## 7. 新工程移植时要改哪里

如果新建一个 STM32 CMake 工程，通常需要检查这些地方：

```text
Board/CMakePresets.json
Board/CMakeLists.txt
Board/cmake/gcc-arm-none-eabi.cmake
Board/cmake/stm32cubemx/CMakeLists.txt
.vscode/tasks.json
.vscode/launch.json
.vscode/c_cpp_properties.json
```

重点确认：

```text
工程名是否正确
输出 elf 文件名是否正确
linker script 是否匹配芯片
芯片宏是否正确，例如 STM32F405xx
OpenOCD target cfg 是否正确
includePath 是否包含 HAL/CMSIS/FreeRTOS/USB
```

## 8. 需要自己维护的文档

建议每个工程都维护这些文档。

### 8.1 环境与编译文档

文件：

```text
Docs/environment_and_build.md
```

记录：

```text
软件版本
工具链路径
CMake/Ninja 使用方法
VSCode tasks.json/launch.json/c_cpp_properties.json 说明
烧录和 Debug 方法
常见环境问题
```

### 8.2 CubeMX 与硬件配置文档

建议文件：

```text
Docs/ydrive_cubemx_and_plan.md
```

记录：

```text
MCU 型号
时钟树
GPIO 分配
TIM/ADC/SPI/UART/USB 配置
FreeRTOS 配置
重要引脚说明
```

每次修改 `.ioc` 或重新生成代码后，要同步更新。

### 8.3 控制时序文档

建议文件：

```text
Docs/ODrive_motor0_timing_notes.md
```

记录：

```text
参考工程的控制时序
TIM/ADC/中断/线程之间的关系
关键回调函数
控制环执行顺序
后续要复刻或简化的内容
```

### 8.4 Bring-up 实验记录

建议新增：

```text
Docs/bringup_log.md
```

记录：

```text
日期
固件版本或改动说明
接线方式
供电方式
烧录结果
调试现象
示波器观察结果
遇到的问题
下一步计划
```

硬件调试很依赖现场状态，这份日志后面会非常有用。

## 9. 常见问题

### 9.1 CMake 找不到 preset

现象：

```text
CMake Error: Could not read presets from ...\YDrive: File not found
```

原因：

```text
当前目录错了。CMakePresets.json 在 Board 目录。
```

解决：

```powershell
cd C:\Users\mrg\Desktop\PMSMFocLib\YDrive\Board
cmake --fresh --preset Debug
cmake --build --preset Debug
```

### 9.2 VSCode 报 __IO 未定义

原因：

```text
IntelliSense 没读到 CMSIS include 路径或芯片宏。
```

解决：

```text
检查 .vscode/c_cpp_properties.json
确认定义了 STM32F405xx 和 USE_HAL_DRIVER
确认 includePath 包含 Drivers/CMSIS/Include
确认 includePath 包含 Drivers/CMSIS/Device/ST/STM32F4xx/Include
执行 C/C++: Reset IntelliSense Database
```

### 9.3 能编译但 VSCode 红线很多

优先相信：

```powershell
cmake --build --preset Debug
```

如果命令行编译通过，而 VSCode 仍然报红线，大概率是 IntelliSense 配置问题，不是工程编译问题。
