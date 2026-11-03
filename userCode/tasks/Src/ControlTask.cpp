//
// Created by LEGION on 2021/10/4.
// 控制任务：无头模式（参考 FTC 场向控制）+ 车头模式 + 急停
//
#include "ControlTask.h"
#include "ChassisTask.h"
#include "IMU.h"
#include <math.h>

/* ==================== 可调参数（第一次装车按实际再调） ==================== */
// 平移最大速度 m/s
#define MAX_TRANS_VELOCITY      3.5f
// 车头模式：旋转摇杆横拨对应的最大自转速度（RPM）
#define MAX_ROT_SPEED_RPM       60.0f
// 无头模式：旋转摇杆横拨满舵时，目标朝向的旋转角速度（rad/s）
#define HEADLESS_YAW_RATE       2.5f
// 无头模式航向角环 PID（输出单位 RPM）
#define HEADING_KP              100.0f
#define HEADING_KD              5.0f

/* ---------------------------- 死区（防爬行） ---------------------------- */
// 摇杆死区（归一化 -1~1）：推杆小于该值视为 0
#define RC_DEADBAND             0.03f
// 平移输出死区（m/s）：小于该速度直接置 0
#define TRANS_OUTPUT_DEADBAND   0.06f
// 转向输出死区（RPM）：小于该转速直接置 0
#define ROT_OUTPUT_DEADBAND     1.5f
// 无头模式航向死区（rad）：航向误差小于该值不修正（避免 IMU 零漂让轮子慢转）
#define HEADING_DEADBAND        0.05f
/* ---------------------------------------------------------------------- */

// 底盘自转方向修正：实车转向反了改成 -1.0f
#define HEADLESS_ROT_SIGN       1.0f
// 手动转向方向修正（车头模式/兜底模式）：旋转摇杆右拨应右转；实测反了改成 -1.0f
#define MANUAL_ROT_SIGN         (-1.0f)
/* ====================================================================== */

/* ===================== 摇杆映射选择（只改这一个开关） =====================
 * 1 = 左摇杆平移 + 右摇杆旋转   —— 需要左摇杆是"双回中"（松手自动回中）
 * 0 = 右摇杆平移 + 左摇杆左右旋转 —— 出厂天地飞左摇杆不回中时用这个
 */
#define USE_LEFT_STICK_TRANSLATION  0

#if USE_LEFT_STICK_TRANSLATION
  #define RC_TRANS_COL   rc.left_col    // 平移：前后
  #define RC_TRANS_ROL   rc.left_rol    // 平移：左右
  #define RC_ROT_ROL     rc.right_rol   // 旋转：左右
#else
  #define RC_TRANS_COL   rc.right_col
  #define RC_TRANS_ROL   rc.right_rol
  #define RC_ROT_ROL     rc.left_rol
#endif

static float targetYaw = 0.0f;      // 无头模式目标朝向（rad，场地坐标系）
static bool  headlessActive = false;// 进入无头模式时用于把当前车头设为 0 度

static float wrapPI(float angle) {
    while (angle > 3.1415926f)  angle -= 6.2831853f;
    while (angle < -3.1415926f) angle += 6.2831853f;
    return angle;
}

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float deadband(float v) {
    return (fabsf(v) < RC_DEADBAND) ? 0.0f : v;
}

void CtrlHandle() {
    const RC_Info_t &rc = RemoteControl::rcInfo;

    // ===== RC 失联保护：超过 100ms 没收到遥控帧就强制停车（防止按最后指令失控）=====
    {
        uint32_t rcNow = HAL_GetTick();
        uint32_t rcAge = (RemoteControl::lastFrameTick == 0u)
                        ? 0xFFFFFFFFu
                        : (rcNow - RemoteControl::lastFrameTick);
        if (rcAge > 100u) {
            ChassisStop();
            UserStop();
            headlessActive = false;
            return;
        }
    }

    // ===== 急停：遥控器 SC（右侧开关）拨到最下档，保持框架原有逻辑 =====
    if (rc.sRight == DOWN_POS) {
        ChassisStop();
        UserStop();
        headlessActive = false;
        return;
    }

    switch (rc.sLeft) {
    // ===================== 一档：无头模式（场地坐标系） =====================
    case UP_POS: {
        if (!headlessActive) {
            // 进入无头模式的瞬间，把当前车头方向定为“场地正前方”
            IMU::resetYaw(0.0f);
            targetYaw = 0.0f;
            headlessActive = true;
        }

        // 安全兜底：IMU 初始化失败时无头模式不可用，退回车头模式
        if (IMU::errorCode != 0u) {
            float fb  = deadband(RC_TRANS_COL) * MAX_TRANS_VELOCITY;
            float lr  = deadband(RC_TRANS_ROL) * MAX_TRANS_VELOCITY;
            float rot = MANUAL_ROT_SIGN * deadband(RC_ROT_ROL) * MAX_ROT_SPEED_RPM;
            if (fabsf(rot) < ROT_OUTPUT_DEADBAND) rot = 0.0f;
            if (fabsf(fb) == 0.0f && fabsf(lr) == 0.0f && rot == 0.0f) {
                ChassisSetVelocity(0.0f, 0.0f, 0.0f);   // 零速主动刹车（比断输出停得快）
            } else {
                ChassisSetVelocity(fb, lr, rot);
            }
            break;
        }

        // 旋转摇杆：改变目标朝向（FTC 里是 triggerDelta 改 targetHeading）
        targetYaw = wrapPI(targetYaw - deadband(RC_ROT_ROL) * HEADLESS_YAW_RATE * 0.001f);

        // 航向角环：带死区 + 输出死区，静止时不会因 IMU 零漂让轮子慢转
        float yawErr = wrapPI(targetYaw - IMU::getYaw());
        float rotRpm = 0.0f;
        if (fabsf(yawErr) > HEADING_DEADBAND) {
            rotRpm = clampf(HEADING_KP * yawErr - HEADING_KD * IMU::getGyroZ(),
                            -MAX_ROT_SPEED_RPM, MAX_ROT_SPEED_RPM);
            rotRpm *= HEADLESS_ROT_SIGN;
            if (fabsf(rotRpm) < ROT_OUTPUT_DEADBAND) {
                rotRpm = 0.0f;
            }
        }

        // 平移摇杆：场地坐标系的前后（上=前）/ 左右（右=右）
        float vFieldForward = deadband(RC_TRANS_COL) * MAX_TRANS_VELOCITY;
        float vFieldLeft    = -deadband(RC_TRANS_ROL) * MAX_TRANS_VELOCITY;
        if (fabsf(vFieldForward) < TRANS_OUTPUT_DEADBAND) vFieldForward = 0.0f;
        if (fabsf(vFieldLeft) < TRANS_OUTPUT_DEADBAND)    vFieldLeft = 0.0f;

        // 完全静止（无平移、无转向）→ 强制停车，避免电机 PID 爬行
        if (vFieldForward == 0.0f && vFieldLeft == 0.0f && rotRpm == 0.0f) {
            ChassisSetVelocity(0.0f, 0.0f, 0.0f);   // 零速主动刹车
            break;
        }

        // 场地坐标系 → 车体坐标系（车头越歪，摇杆“前”方向越要补偿）
        float yaw = IMU::getYaw();
        float c = cosf(yaw), s = sinf(yaw);
        float vBodyForward = vFieldForward * c + vFieldLeft * s;
        float vBodyLeft    = -vFieldForward * s + vFieldLeft * c;

        // ChassisSetVelocity 约定：第 2 个参数右为正
        ChassisSetVelocity(vBodyForward, -vBodyLeft, rotRpm);
        break;
    }
    // ===================== 二档：车头模式（常规操控） =====================
    case MID_POS: {
        headlessActive = false;

        float fb  = deadband(RC_TRANS_COL) * MAX_TRANS_VELOCITY;
        float lr  = deadband(RC_TRANS_ROL) * MAX_TRANS_VELOCITY;
        float rot = MANUAL_ROT_SIGN * deadband(RC_ROT_ROL) * MAX_ROT_SPEED_RPM;

        if (fabsf(fb) < TRANS_OUTPUT_DEADBAND)  fb = 0.0f;
        if (fabsf(lr) < TRANS_OUTPUT_DEADBAND)  lr = 0.0f;
        if (fabsf(rot) < ROT_OUTPUT_DEADBAND)   rot = 0.0f;

        if (fb == 0.0f && lr == 0.0f && rot == 0.0f) {
            ChassisSetVelocity(0.0f, 0.0f, 0.0f);   // 零速主动刹车
        } else {
            ChassisSetVelocity(fb, lr, rot);
        }
        break;
    }
    // ===================== 三档：底盘停车（便于调参） =====================
    case DOWN_POS:
    default: {
        headlessActive = false;
        ChassisStop();
        break;
    }
    }
}
