//
// Config.h —— 全车可调参数统一管理
// 说明：所有"需要按实车/手感调整"的常数都集中在这里，改参数只改这个文件。
//       协议/寄存器/位掩码类定义（如 MOTOR_ID_*、BMI088 寄存器地址、舵机 ID）仍留在各自驱动文件里，
//       因为它们属于驱动接口，不属于可调参数。
// 维护：改完参数重新编译烧录即可；带"标定"字样的参数请按 README 的标定流程实测后再改。
//
#ifndef RM_FRAME_C_CONFIG_H
#define RM_FRAME_C_CONFIG_H

#include "main.h"

/* ===================== 1. 底盘几何 / 标定 ===================== */
#define WHEEL_DIAMETER             0.1525f   // 麦轮直径 m（标称 152.5mm）
#define TRACK_WIDTH                0.40f     // 轮距 m（左右轮接地中心距）
#define WHEEL_BASE                 0.38f     // 轴距 m（前后轮接地中心距）
#define REDUCTION_RATIO            19.0f     // M3508 P19 减速比
/* 自转标定系数：实测"命令 360° 实际转了 388°"，等效 k 比 (TRACK_WIDTH+WHEEL_BASE)/2 小
 * 约 7%（麦轮滚动/滑移几何与理想模型有差）。omega 乘这个系数后，自转角度才和指令一致。
 * 标定记录：0.933 实测 361.59°/360° → 0.929（2026-11-04）。换轮子/改布局后要重新标。 */
#define ROTATION_CALIB             0.929f

/* ===================== 2. 底盘电机 ID（C620，CAN1） ===================== */
#define CHASSIS_ID_FL              0x00000001u   // C620 ID 1
#define CHASSIS_ID_FR              0x00000002u   // C620 ID 2
#define CHASSIS_ID_BL              0x00000004u   // C620 ID 3
#define CHASSIS_ID_BR              0x00000008u   // C620 ID 4

/* ===================== 3. 底盘速度环 PID（M3508 + C620） ===================== */
#define CHASSIS_PID_KP             60.0f
#define CHASSIS_PID_KI             1.0f      // 加积分消除 P-only 稳态掉速（实测 10% 静差）
#define CHASSIS_PID_KD             0.0f
#define CHASSIS_PID_KP_MAX         16384.0f
#define CHASSIS_PID_KI_MAX         4000.0f   // 积分输出上限（≈5A），防饱和
#define CHASSIS_PID_KD_MAX         0.0f
#define CHASSIS_PID_OUT_MAX        16384.0f  // C620: 16384 = 20A，可调小保护硬件
#define MOTOR_STOP_DEADBAND_RPM    4.0f      // 停车死区：低于该转速不再输出（主动刹车）

/* ===================== 4. 驾驶手感 / 无头控制 ===================== */
#define MAX_TRANS_VELOCITY         3.5f      // 平移最大速度 m/s
#define MAX_ROT_SPEED_RPM          60.0f     // 车头模式最大自转（RPM）
#define HEADLESS_YAW_RATE          2.5f      // 无头模式目标朝向转速 rad/s
#define HEADING_KP                 100.0f    // 航向环 P
#define HEADING_KD                 5.0f      // 航向环 D
#define RC_DEADBAND                0.03f     // 摇杆死区（归一化）
#define TRANS_OUTPUT_DEADBAND      0.06f     // 平移输出死区 m/s
#define ROT_OUTPUT_DEADBAND        1.5f      // 转向输出死区 RPM
#define HEADING_DEADBAND           0.05f     // 航向死区 rad
#define HEADLESS_ROT_SIGN          1.0f      // 无头模式转向修正（反了改 -1）
#define MANUAL_ROT_SIGN            (-1.0f)   // 手动转向修正（反了改 1）
#define USE_LEFT_STICK_TRANSLATION 0         // 0=右摇杆平移+左摇杆转向；1=左摇杆平移

/* ===================== 5. 遥控开关 -> 通道映射 ===================== */
#define RC_SW_IDX_SA               2u        // rcInfo.sw[] 下标：0=CH5 1=CH6 2=CH7 3=CH8
#define RC_SW_IDX_SD               3u
#define SA_IS_THREE_POSITION       0         // ET08 手册 P10：SA 是两档开关
#define RC_SW_UP_THRESHOLD         700u      // 开关原始值 < 700 = 上档
#define RC_SW_DOWN_THRESHOLD       1300u     // 开关原始值 > 1300 = 下档
#define SBUS_RX_BUF_NUM            36u
#define RC_FRAME_LENGTH            25u
#define RC_CH_VALUE_OFFSET         1024

/* ===================== 6. 底盘标定 / 测量模式（SB 三档触发） ===================== */
#define MEAS_STRAIGHT_SPEED        0.5f      // 直行测试速度 m/s
#define MEAS_STRAIGHT_HOLD_MS      3800u     // 匀速段：0.5×3.8 = 1.900 m
#define MEAS_STRAIGHT_DECEL_MS     400u      // 线性减速段：0.5×0.5×0.4 = 0.100 m
#define MEAS_STRAIGHT_BRAKE_MS     200u      // 闭环刹停段（目标 0，防止滑停甩头）
#define MEAS_STRAIGHT_TIME_MS      (MEAS_STRAIGHT_HOLD_MS + MEAS_STRAIGHT_DECEL_MS + MEAS_STRAIGHT_BRAKE_MS)
#define MEAS_ROTATE_RADPS          1.0f      // 自转测试角速度 rad/s
#define MEAS_ROTATE_TIME_MS        6283u     // 6.283 s = 2π rad = 360°
#define MEAS_ROTATE_RPM            (MEAS_ROTATE_RADPS * 30.0f / 3.1415926f)

/* ===================== 7. IMU（BMI088） ===================== */
#define IMU_YAW_SIGN               1.0f      // yaw 方向修正（反了改 -1）
#define IMU_CALI_SAMPLES           1000      // 陀螺零偏校准采样数（1ms/次 ≈ 1s）
#define IMU_GYRO_DEADBAND          0.004f    // 陀螺静止死区 rad/s
#define IMU_SPI_PRESCALER          SPI_BAUDRATEPRESCALER_64

/* ===================== 8. 用户自定义电机（M2006 / USER_MOTOR） ===================== */
#define USER_MOTOR_ID              0x00000010u   // CAN1 ID 5
#define USER_MOTOR_REDUCTION       36.0f
#define USER_PID_KP                60.0f
#define USER_PID_KI                0.0f
#define USER_PID_KD                0.0f
#define USER_PID_KP_MAX            10000.0f
#define USER_PID_KI_MAX            0.0f
#define USER_PID_KD_MAX            0.0f
#define USER_PID_OUT_MAX           10000.0f

/* ===================== 9. USB CDC 调试输出周期 ===================== */
#define DEBUG_RC_PERIOD_MS         100u      // 遥控/IMU/CAN 状态行周期
#define DEBUG_MOTOR_PERIOD_MS      20u       // 电机 T/F/I 行周期（50Hz）

#endif // RM_FRAME_C_CONFIG_H
