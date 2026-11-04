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
- **开关解码**：CH5~CH8 四个开关全部按原始值区间解出位置（天地飞实测 353 / 1024 / 1694）；**SA=模式、SD=急停**
- **底盘标定/测量模式**：SB 上=直行 2 m 测试、SB 下=原地转 360° 测试，IMU 实测转角并从 USB CDC 打印标定建议

## 摇杆与开关

| 操作 | 功能 |
|---|---|
| SA 上档 | **无头模式**：右摇杆按场地坐标系平移，左摇杆左右改目标车头朝向（松手保持） |
| SA 下档 | **车头模式**：右摇杆前后左右，左摇杆左右自转 |
| SD 下档 | **急停**（底盘停 + UserStop；SC 下档暂时保留兜底） |
| SB / SC | 暂未使用，留给摆臂/夹爪等机构 |

> 注意：ET08 手册 P10 写明 **SA 是两档开关**、SB/SC 是三档、SD 是两档——所以 SA 只有 上=无头 / 下=车头 两个模式，没有"停车档"（停车用 SD）。

> 开关通道：CH5~CH8 已经全部解成 `rcInfo.sw[0..3]`。默认 **SA=CH7（辅助2）、SD=CH8（辅助3）**，需要在遥控器【通用功能】→【通道设置】里把 辅助2 配成 SA、辅助3 配成 SD；如果实际分到别的通道，改 `ControlTask.cpp` 顶部的 `RC_SW_IDX_SA / RC_SW_IDX_SD`（0~3 对应 CH5~CH8）。
> 摇杆映射可用 `ControlTask.cpp` 顶部 `USE_LEFT_STICK_TRANSLATION` 宏切换（1=左摇杆平移，需左摇杆双回中；0=右摇杆平移）。

## 底盘标定（测量模式）

用 **SB（三档开关，CH5）** 触发（测试期间摇杆无效，**SD 急停和遥控失联保护始终有效**）：

| SB 档位 | 测试 | 动作 |
|---|---|---|
| 上档 | 直行标定 | 0.5 m/s 匀速 **3.8 s** → **0.4 s 线性减速** → **0.2 s 闭环刹停**（名义 **2.000 m**，受控停车不甩头） |
| 中档 | 正常驾驶 | — |
| 下档 | 自转标定 | 1.0 rad/s 逆时针转 **6.283 s**（名义 **360°**）后自动停 |

> 同一个档位只触发一次；测完要先回中档，再拨到上/下档才会重新测。

### 不用连 ST-Link 测（推荐流程）

测试时完全离线，结果会**常驻在板子内存里**：

1. ST-Link 烧录完成后**拔掉**（测试不需要 ST-Link）；
2. 接电池，把 SB 拨到上档跑直行、再回中档、拨下档跑自转；
3. 测完**先别断电**，把 ST-Link 插回来，读内存里的结果：
   - 查符号地址（每次重新编译后地址可能变，以 nm 输出为准）：
     NM 用 CubeIDE 自带工具链，执行 `$NM -C build/RM_Frame_C.elf | grep -E "measStraightResult|measRotateResult"`
   - OpenOCD 读，每项 **6 个 word**：seq / elapsedMs / nominalMm / measuredX100 / kRealMm / wheelCounts
     示例：`openocd -f c_board_stlink.cfg -c "init" -c "halt" -c "mdw 0x20002a3c 6" -c "mdw 0x20002a54 6" -c "resume" -c "exit"`
   - 字段含义：seq=第几次测试、measuredX100=直行 yaw 漂移 / 自转实测角度（deg×100）、kRealMm=自转建议的 k（mm）、wheelCounts=轮子平均累计转角（转子 counts，8192/圈）；
4. 如果中途断了电，结果丢失，重测一次即可（直行 4 s、自转 6.3 s）。

**同时结果也会从 USB CDC（虚拟串口）打印：**

- 直行（times 4.4 s：匀速 3.8 s + 减速 0.4 s + 刹停 0.2 s，名义 2.000 m）：
  - `yawDrift` 是车头漂移（越小越直）；
  - 轮子实际滚动距离 = `wheelCounts / 8192 / 19 × π × 0.1525`（m），可以和名义值对比判断速度环有没有静差；
  - 如果要绝对距离精度：卷尺量实际距离 L，则 `WHEEL_DIAMETER = 0.1525 × L / 轮子滚动距离`。
- 自转（6.283 s，名义 360°）：用 IMU 实测角度和 `wheelCounts` 反推真实 k：
  ```
  k_real = (wheelCounts/8192/19 × π × 0.1525) / (实测角度 × π/180)
  ```
  控制器里用 `ROTATION_CALIB`（`ChassisTask.h`）修正指令，不再直接改 `k`。

### 标定结果记录（2026-11-04）

| 项 | 数据 | 处理 |
|---|---|---|
| 直行 | 名义 2.000 m，轮子累计 4.2366 圈（编码器距离 2.030 m），yaw 漂移 **-0.84°** | 速度环 P-only 静差已用 **Ki=1.0** 消除；轮径保持标称 0.1525 m |
| 自转 | 名义 360°，实测 **387.97°**（多 7.97°）；两种算法 k_real ≈ 0.362~0.366 m | `ROTATION_CALIB = 0.933`（对应 k_real≈0.364 m，代码内 0.390 m） |

## 参数统一管理（userCode/Inc/Config.h）

所有**需要按实车/手感调整**的常数都集中在 [userCode/Inc/Config.h](userCode/Inc/Config.h)，改参数只改这一个文件：

| 分组 | 内容 |
|---|---|
| 1. 底盘几何/标定 | `WHEEL_DIAMETER`、`TRACK_WIDTH`、`WHEEL_BASE`、`REDUCTION_RATIO`、`ROTATION_CALIB` |
| 2. 底盘电机 ID | `CHASSIS_ID_FL/FR/BL/BR`（对应 C620 ID 1~4） |
| 3. 速度环 PID | `CHASSIS_PID_KP/KI/KD`、限幅、`CHASSIS_PID_OUT_MAX`、`MOTOR_STOP_DEADBAND_RPM` |
| 4. 驾驶手感/无头 | 最大速度/转向、各类死区、方向修正、摇杆映射开关 |
| 5. 遥控开关映射 | `RC_SW_IDX_SA/SD`、`SA_IS_THREE_POSITION`、开关原始值阈值、SBUS 参数 |
| 6. 标定/测量模式 | 直行/自转测试的速度、时长、减速段 |
| 7. IMU | `IMU_YAW_SIGN`、零偏校准采样数、静止死区、SPI 分频 |
| 8. 用户自定义电机 | `USER_MOTOR_ID/REDUCTION`、`USER_PID_*` |
| 9. 调试输出 | USB CDC 打印周期 |

> **不搬进来的**：协议/寄存器/位掩码类定义（`MOTOR_ID_*`、`SERVO_ID_*`、BMI088 寄存器地址、`INRANGE`/`AFFINE` 工具宏）——它们属于驱动接口，不属于可调参数。
> Keil 工程已在 `MDK-ARM/RM_Frame_C.uvprojx` 的 IncludePath 里加上 `../userCode/Inc`；CMake 已加 `userCode/Inc`。

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
│   ├── Inc/Config.h     **全车可调参数统一管理（改参数只看这一个文件）**
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
