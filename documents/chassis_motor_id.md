# 底盘电机（C620）总线 ID 配置说明

> 适用：RM 开发板 C 型 + 4 个 M3508 + C620 电调，全部接在 **CAN1** 上

## 1. 当前代码配置

配置集中在 [`userCode/tasks/Src/ChassisTask.cpp`](../userCode/tasks/Src/ChassisTask.cpp) 顶部：

```cpp
#define CHASSIS_ID_FL   MOTOR_ID_1   // 左前 FL = ID 1
#define CHASSIS_ID_FR   MOTOR_ID_2   // 右前 FR = ID 2
#define CHASSIS_ID_BL   MOTOR_ID_3   // 左后 BL = ID 3
#define CHASSIS_ID_BR   MOTOR_ID_4   // 右后 BR = ID 4
```

| 位置 | 代码变量 | 电调 ID | 控制帧 | 反馈帧 | 总线 |
|---|---|---|---|---|---|
| 左前 FL | `CMFL` | 1 | 0x200 | 0x201 | CAN1 |
| 右前 FR | `CMFR` | 2 | 0x200 | 0x202 | CAN1 |
| 左后 BL | `CMBL` | 3 | 0x200 | 0x203 | CAN1 |
| 右后 BR | `CMBR` | 4 | 0x200 | 0x204 | CAN1 |

- C620 控制帧：CAN1 上 ID 1~4 用 `0x200`，ID 5~8 用 `0x1FF`（由 `Motor::CANPackageSend()` 自动处理）；
- 反馈帧：ID N 对应 `0x200 + N`（默认改不了），`Motor::IT_Handle()` 用 `StdId - 0x201` 找对应电机对象；
- ID 1~8 走 CAN1，9~16 才走 CAN2（见 `Motor.h` 注释）。

## 2. 硬件侧：把 C620 电调本身也设成 1~4

两种方式：

1. **RoboMaster Assistant**（USB 接 C 板 / 或电调调试口）：图形化设置每个电调的 ID；
2. **电调上的 SET 键**：电调正常上电（绿灯按 ID 闪 N 次）→ 短按 SET 进入设置 ID 模式（灯灭）→ 短按 SET N 次（N=1~4，每次橙灯闪一下）→ **3 秒不操作自动保存** → 重新上电生效。

设置完上电，**看电调指示灯**：绿灯每秒闪 N 次 = 当前 ID 为 N。

## 3. 接线 & 注意事项

- 4 个 C620 的 CAN 线并接在 C 板 **CAN1** 口（2-pin：CANL/CANH，别接反）；
- CAN 总线两端要接终端电阻（C 板一端通常已有 120Ω，另一端电调附近需要 120Ω），否则通信不稳定；
- 同一总线 **ID 不能重复**：代码里已加编译期检查（`static_assert`）和运行期检查（`Motor::idErrorFlag`），ID 重复时冲突的电机对象不会输出；
- 检查 ID 是否正确的方法：上电后看每个 C620 的绿灯闪烁次数，或用 RoboMaster Assistant 查看/修改。

## 4. 改 ID 时改哪里

只改 `ChassisTask.cpp` 顶部 4 个 `#define`，并把电调本身也改成对应 ID；
如果需要把某个电机改到 CAN2（ID 9~16）或加第 5 个电机（如 ID 5），在 `UserTask.cpp` 里按同样方式定义即可。
