# YDrive 环境配置与编译说明

本文说明如何配置 YDrive 的 Windows 开发环境，以及如何在 VSCode/PowerShell 中编译生成 `.elf`、`.bin`、`.hex` 文件。

## 1. 目录说明

仓库根目录是：

```powershell
C:\Users\mrg\Desktop\PMSMFocLib\YDrive
```

真正的 STM32 CubeMX/CMake 工程目录是：

```powershell
C:\Users\mrg\Desktop\PMSMFocLib\YDrive\Board
```

也就是说，执行 `cmake --preset Debug` 时必须进入 `Board` 目录，而不是仓库根目录。

正确：

```powershell
cd C:\Users\mrg\Desktop\PMSMFocLib\YDrive\Board
```

错误：

```powershell
cd C:\Users\mrg\Desktop\PMSMFocLib\YDrive
```

如果在仓库根目录执行 CMake，会看到类似错误：

```text
CMake Error: Could not read presets from ...\YDrive: File not found
```

原因是 `CMakePresets.json` 在 `Board` 目录里，不在仓库根目录里。

## 2. 需要安装的软件

### 2.1 STM32CubeMX

当前工程使用：

```text
STM32CubeMX v6.16.0
STM32Cube FW_F4 V1.28.3
```

CubeMX 用来打开和修改：

```powershell
C:\Users\mrg\Desktop\PMSMFocLib\YDrive\Board\YDrive.ioc
```

注意：修改 `.ioc` 后重新生成代码时，要确认 `USER CODE BEGIN/END` 中的用户代码没有被破坏。

### 2.2 CMake

推荐版本：

```text
CMake 3.26.0 或更高
```

检查命令：

```powershell
cmake --version
```

示例输出：

```text
cmake version 3.26.0
```

### 2.3 Ninja

当前 `CMakePresets.json` 使用 Ninja 生成器。

检查命令：

```powershell
ninja --version
```

示例输出：

```text
1.13.2
```

### 2.4 GNU Arm Embedded Toolchain

当前验证过的版本：

```text
arm-none-eabi-gcc 10.3-2021.10
```

检查命令：

```powershell
arm-none-eabi-gcc --version
```

示例输出：

```text
arm-none-eabi-gcc.exe (GNU Arm Embedded Toolchain 10.3-2021.10) 10.3.1
```

需要保证 `arm-none-eabi-gcc.exe` 所在目录已经加入 Windows `PATH`。

## 3. 一次完整编译流程

进入工程目录：

```powershell
cd C:\Users\mrg\Desktop\PMSMFocLib\YDrive\Board
```

重新配置 CMake：

```powershell
cmake --fresh --preset Debug
```

编译：

```powershell
cmake --build --preset Debug
```

编译成功后，输出目录是：

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

其中：

- `YDrive.elf`：调试用，包含符号信息
- `YDrive.bin`：裸二进制固件，可用于烧录
- `YDrive.hex`：Intel HEX 固件，也可用于烧录
- `YDrive.map`：链接映射文件，用来查看 RAM/FLASH 占用和符号分布

## 4. 为什么不手动 objcopy

`Board/CMakeLists.txt` 已经加入了 post-build 规则：

```cmake
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex
        $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
        ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary
        $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
        ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin
    COMMENT "Generating ${CMAKE_PROJECT_NAME}.hex and ${CMAKE_PROJECT_NAME}.bin"
)
```

所以只要执行：

```powershell
cmake --build --preset Debug
```

就会自动生成 `.bin` 和 `.hex`。

不要再使用旧工程的命令：

```powershell
arm-none-eabi-objcopy -O binary build/Debug/stm32F407IGT6.elf build/Debug/stm32F407IGT6.bin
```

这个命令不适合 YDrive，因为：

- YDrive 的工程名是 `YDrive`
- 输出文件是 `YDrive.elf`
- 当前 MCU 是 `STM32F405RGTx`，不是 `STM32F407IGT6`

如果确实要手动生成 `.bin`，正确命令是：

```powershell
cd C:\Users\mrg\Desktop\PMSMFocLib\YDrive\Board
arm-none-eabi-objcopy -O binary build\Debug\YDrive.elf build\Debug\YDrive.bin
```

但正常情况下不需要手动执行。

## 5. 常见问题

### 5.1 CMake 找不到 preset

现象：

```text
CMake Error: Could not read presets from C:/Users/mrg/Desktop/PMSMFocLib/YDrive: File not found
```

原因：

```text
当前目录错了。
```

解决：

```powershell
cd C:\Users\mrg\Desktop\PMSMFocLib\YDrive\Board
cmake --fresh --preset Debug
cmake --build --preset Debug
```

### 5.2 找不到 elf 文件

现象：

```text
arm-none-eabi-objcopy.exe: 'build/Debug/stm32F407IGT6.elf': No such file
```

原因：

```text
文件名写错了。YDrive 生成的是 YDrive.elf。
```

解决：

```powershell
cd C:\Users\mrg\Desktop\PMSMFocLib\YDrive\Board
cmake --build --preset Debug
```

编译后查看：

```powershell
dir build\Debug\YDrive.*
```

### 5.3 PowerShell 显示中文乱码

如果 PowerShell 中查看中文 Markdown 出现乱码，可以先切换 UTF-8：

```powershell
chcp 65001
```

这只影响终端显示，不代表文件内容损坏。GitHub 和 VSCode 通常会按 UTF-8 正常显示。

## 6. 推荐 VSCode 使用方式

用 VSCode 打开仓库根目录：

```powershell
code C:\Users\mrg\Desktop\PMSMFocLib\YDrive
```

编译时可以打开终端，进入 `Board`：

```powershell
cd Board
cmake --fresh --preset Debug
cmake --build --preset Debug
```

调试时使用：

```text
Board\build\Debug\YDrive.elf
```

烧录时使用：

```text
Board\build\Debug\YDrive.bin
```

## 7. 当前启动验证目标

当前固件阶段只验证基础系统：

- 能编译
- 能下载
- 能调试进入 `main()`
- FreeRTOS `defaultTask` 跑起来
- USB CDC 输出 `YDrive boot`
- USB CDC 枚举成功
- 周期读取 `nFAULT`
- `PB12 EN_GATE` 默认保持 LOW

这一阶段不要启动电机功率输出，不要拉高 `EN_GATE`。


