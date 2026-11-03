# 无头控制（Headless / 场向控制）

> 参考 FTC 代码 `MecanumDrive`（Pinpoint 场向驱动 + 航向 PID）实现，适配 RM 开发板 C 型 + M3508 麦轮底盘。
> 新增文件：`userCode/devices/Inc/IMU.h`、`userCode/devices/Src/IMU.cpp`、`documents/headless_control.md`
> 修改文件：`userCode/devices/Src/Device.cpp`（初始化/1ms 调用）、`userCode/tasks/Src/ChassisTask.cpp`（麦轮解算）、`userCode/tasks/Src/ControlTask.cpp`（无头控制）

## 1. 操作方式

遥控器 **SB（左侧三位开关）** 选模式，**SC（右侧三位开关）下档** = 急停（原有逻辑不变）：

| SB 档位 | 模式 | 说明 |
|---|---|---|
| 一档（上） | **无头模式** | **左摇杆**按场地坐标系前后/左右平移；**右摇杆左右**改变目标车头朝向；松手自动保持车头 |
| 二档（中） | 车头模式 | 左摇杆前后/左右（车头坐标系），右摇杆左右自转 |
| 三档（下） | 停车 | 底盘电机停转（调参用） |

无头模式下：进入一档的瞬间，**当前车头方向 = 场地正前方**，之后无论车头怎么转，右摇杆“前”始终是场地那个方向。

## 2. 代码结构

| 文件 | 作用 |
|---|---|
| `userCode/devices/Inc/IMU.h` / `Src/IMU.cpp` | BMI088 SPI 驱动 + 陀螺零偏校准 + Z 轴角速度积分出 yaw |
| `userCode/tasks/Src/ChassisTask.cpp` | `WheelsSpeedCalc()`：麦轮逆运动学（前后/左右/自转 → 四轮转速） |
| `userCode/tasks/Src/ControlTask.cpp` | `CtrlHandle()`：模式切换、场向→车体坐标旋转、航向角环 PID |
| `userCode/devices/Src/Device.cpp` | 主函数里加 `IMU::Init()`；1ms 中断里加 `IMU::Handle()` |

数据流（全部在 TIM10 的 1ms 中断里）：

```
IMU::Handle()            // 读陀螺，更新 yaw
   ↓
CtrlHandle()             // 摇杆 → 场地坐标系速度 → 车体坐标系速度 + 目标朝向角环
   ↓
ChassisHandle()          // 四轮转速 → 每轮 PID → 电流值
   ↓
Motor::CANPackageSend()  // CAN 0x200/0x1FF 下发
```

## 3. 与 FTC 代码的对应关系

| FTC（31558 TeamCode） | 本框架实现 |
|---|---|
| `MecanumDrive.moveRobotFieldRelative()` 里的 `rotX/rotY` 旋转 | `ControlTask.cpp` 里 `vFieldForward/vFieldLeft` → `vBodyForward/vBodyLeft` |
| 麦轮功率分配 `leftFrontPower = ...` | `ChassisTask::WheelsSpeedCalc()` 四轮线速度公式 |
| `headingController`（P 控制） + `targetHeading` | `HEADING_KP/KD` + `targetYaw` 角环 |
| `drive.reset(0)`（A 键重置车头） | 进入无头模式的瞬间 `IMU::resetYaw(0)` |
| Pinpoint 的 IMU 航向 | BMI088 陀螺 Z 轴积分（`IMU::getYaw()`） |
| 右摇杆 `setRelativeTargetHeading`（trigger 改目标角） | 左摇杆横拨 `targetYaw += ...`（无头模式） |

## 4. 可调参数

`ControlTask.cpp` 顶部：

| 宏 | 默认 | 说明 |
|---|---|---|
| `MAX_TRANS_VELOCITY` | 3.5 | 平移最大速度 m/s |
| `MAX_ROT_SPEED_RPM` | 60 | 车头模式最大自转（对应框架原来 `left_rol*60`） |
| `HEADLESS_YAW_RATE` | 2.5 | 无头模式左摇杆满舵时的目标角速度 rad/s |
| `HEADING_KP / KD` | 100 / 5 | 航向角环（输出 RPM），抖就减 KP、加 KD |
| `RC_DEADBAND` | 0.02 | 摇杆死区 |
| `HEADLESS_ROT_SIGN` | 1 | 实测自转方向反了改 -1 |

其它：
- `IMU.cpp` 的 `IMU_YAW_SIGN`：yaw 方向反了改 -1；
- `ChassisTask.cpp` 的 `dirFL/dirFR/dirBL/dirBR`：哪个轮子转向反了就改 -1（或直接反接电机线）。

## 5. 首次上车调试步骤

1. **架空车**：推右摇杆向前，四个轮子应全部向前转；有一个反了就改 `dirXX`；
2. 左摇杆向右，车应**顺时针**转（俯视）；反了改 `HEADLESS_ROT_SIGN`；
3. **落地、车静止**，重新上电（IMU 初始化会做约 1 秒零偏校准，此时不要碰车）；
4. SB 拨一档进无头模式，把车原地转 90°，再推右摇杆前——车应沿**进入模式时**的车头方向前进，而不是当前车头方向；
5. 手转车头后松手，车应自动转回目标朝向（航向保持）；转得太猛/抖就调 `HEADING_KP/KD`。

## 6. 已知限制 & 注意

- 航向只用陀螺积分（没有磁力计/里程计融合），**会缓慢漂移**；校内赛一局时间短没问题，长时间跑建议后续加 IST8310 或零速修正；
- 上电时**车必须静止**做零偏校准，否则 yaw 会一直漂；
- 无头模式只做**速度**控制（没有位置保持），和 FTC Pinpoint 的位置环不同；
- IMU 初始化是阻塞的（约 1.2 秒），开机稍慢属于正常；
- **Keil 用户注意**：本目录新增了 `IMU.cpp`，CMake 会自动包含；如果用 Keil，需要手动把 `userCode/devices/Src/IMU.cpp` 加进工程（头文件路径已有）。
