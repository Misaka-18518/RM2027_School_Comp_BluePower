# RM2027 校内赛电控代码（BluePower）

RoboMaster 2027 校内赛机器人底盘控制代码，基于 **DJI RoboMaster 开发板 C 型（STM32F407IGH6）**。

## 硬件配置

| 项目 | 说明 |
|---|---|
| 主控 | RoboMaster 开发板 C 型（STM32F407IGH6，168MHz） |
| 底盘 | 4 × M3508 电机 + C620 电调，麦轮底盘，全部接在 **CAN1** |
| 电机 ID | FL=1，FR=2，BL=3，BR=4 |
| 遥控 | 天地飞 ET08 + RF206S 接收机，接收机 W.BUS 输出接 C 板 DBUS 接口 |
| IMU | 板载 BMI088（SPI1，用于无头模式航向估计） |

## 已实现功能

- **麦轮逆运动学**：前后/左右/自转 → 四轮转速（`userCode/tasks/Src/ChassisTask.cpp`）
- **无头模式**（场地坐标系 + 航向保持）与**车头模式**（`userCode/tasks/Src/ControlTask.cpp`）
- **遥控失联保护**：超过 100ms 收不到遥控帧强制停车
- **电机 ID 配置与冲突检测**（`userCode/devices/Src/Motor.cpp`），编译期 `static_assert` + 运行期检查
- **死区与主动刹车**：摇杆死区、平移/转向输出死区、零速主动刹车，静止不爬行、松手停得快
- **USB CDC 调试输出**：遥控通道解析结果 + 4 个电机的目标/反馈/电流（50Hz）
- **三档开关解码**：SB/SC 按原始值区间判断（天地飞实测 353 / 1024 / 1694），不再依赖特定低字节值

## 摇杆与开关

| 操作 | 功能 |
|---|---|
| SB 上档 | **无头模式**：右摇杆按场地坐标系平移，左摇杆左右改目标车头朝向（松手保持） |
| SB 中档 | **车头模式**：右摇杆前后左右，左摇杆左右自转 |
| SB 下档 | 底盘停车 |
| SC 下档 | **急停**（底盘停 + UserStop） |

> 摇杆映射可用 `ControlTask.cpp` 顶部 `USE_LEFT_STICK_TRANSLATION` 宏切换（1=左摇杆平移，需左摇杆双回中；0=右摇杆平移）。

## 编译

### Keil MDK（Windows）

打开 `MDK-ARM/RM_Frame_C.uvprojx`，编译并下载。

### CMake + arm-none-eabi-gcc（macOS / Linux / Windows）

```bash
cmake -S . -B build
cmake --build build -j
```

需要 `arm-none-eabi-gcc` 在 PATH 中（macOS 可直接用 STM32CubeIDE 自带工具链）。
产物：`build/RM_Frame_C.elf` / `.hex` / `.bin`。

## 烧录

- **SWD**：ST-Link + STM32CubeProgrammer / OpenOCD（C 板 J8 SWD 接口）
- **USB DFU**：短接 BOOT 配置接口后经 Micro-USB 用 dfu-util / STM32CubeProgrammer 烧录 `.bin`

## 目录结构

```text
├── Src/ Inc/            CubeMX 外设初始化（CAN/USART/TIM/SPI/USB 等）
├── Drivers/             STM32F4 HAL + CMSIS
├── Middlewares/         USB Device 库（CDC）
├── userCode/
│   ├── devices/         设备抽象：Motor（C620 PID）、Servo、RemoteControl（DBUS/W.BUS）、IMU（BMI088）
│   └── tasks/           控制任务：ChassisTask（麦轮解算）、ControlTask（模式/摇杆）、UserTask（自定义）
├── startup/             GCC 启动文件
├── MDK-ARM/             Keil 工程
├── documents/           设计/调试说明文档
├── CMakeLists.txt       CMake 构建脚本（CubeMX 生成）
└── RM_Frame_C.ioc       CubeMX 工程文件
```

## 说明

- 本仓库仅包含**电控代码**，机械设计、赛务资料等不在此仓库。
- 代码基于 DJI RoboMaster 校内赛电控培训框架修改，仓库沿用其 **AGPL-3.0** 许可证（见 [LICENSE](LICENSE)）。
