//
// Created by LEGION on 2021/10/4.
// 控制任务：无头模式（参考 FTC 场向控制）+ 车头模式 + 急停
//
#include "ControlTask.h"
#include "ChassisTask.h"
#include "IMU.h"
#include <math.h>

/* ==================== 数值参数统一放在 userCode/Inc/Config.h ====================
 * 包括：最大速度/转向、死区、方向修正、开关通道映射（RC_SW_IDX_SA/SD、SA_IS_THREE_POSITION）、
 *       底盘 PID、几何/标定、测量模式、IMU、用户电机、调试周期。
 * 这里只保留依赖局部变量 rc 的摇杆映射宏。
 */
#if USE_LEFT_STICK_TRANSLATION
  #define RC_TRANS_COL   rc.left_col    // 平移：前后
  #define RC_TRANS_ROL   rc.left_rol    // 平移：左右
  #define RC_ROT_ROL     rc.right_rol   // 旋转：左右
#else
  #define RC_TRANS_COL   rc.right_col
  #define RC_TRANS_ROL   rc.right_rol
  #define RC_ROT_ROL     rc.left_rol
#endif

/* 开关 → 功能映射（通道号/几位开关都见 Config.h） */
#define SW_SA          (RemoteControl::rcInfo.sw[RC_SW_IDX_SA])
#define SW_SD          (RemoteControl::rcInfo.sw[RC_SW_IDX_SD])

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

/* ===================== 底盘标定 / 测量模式（SB 三档，CH5） =====================
 * 用途：标定麦轮参数 WHEEL_DIAMETER / TRACK_WIDTH / WHEEL_BASE。
 * 操作（SD 急停、遥控失联保护始终有效；测试期间摇杆无效）：
 *   SB 上档 = 直行测试：0.5 m/s 匀速 3.8 s → 0.4 s 线性减速 → 0.2 s 闭环刹停（名义 2.000 m）
 *   SB 中档 = 正常驾驶
 *   SB 下档 = 自转测试：1.0 rad/s 逆时针转 6.283 s（名义 360°）后自动停
 * 结果通过 USB CDC 打印（见 Device.cpp）：
 *   直行：用卷尺量实际距离 L，则 WHEEL_DIAMETER = 0.1525 * (L / 2.000)
 *   自转：用 IMU 实测转角，并直接给出建议的 TRACK_WIDTH + WHEEL_BASE
 * 同一个档位只触发一次，回到中档再拨才会再测。
 */
/* 测量模式的数值参数（速度/时长/减速段）见 Config.h 第 6 节 */

typedef enum {
    MEAS_IDLE = 0,
    MEAS_RUN_STRAIGHT,
    MEAS_RUN_ROTATE
} MEASURE_STATE_E;

volatile uint8_t         measureResultFlag = 0u;
volatile MEASURE_RESULT_T measStraightResult;
volatile MEASURE_RESULT_T measRotateResult;

static MEASURE_STATE_E measState = MEAS_IDLE;
static uint32_t        measStartTick = 0u;
static float           measYawStart = 0.0f;
static float           measRotAccumDeg = 0.0f;   // 自转测试：逐周期累加 yaw（不能直接用首尾差，360° 会折回）
static float           measYawLast = 0.0f;
static bool            measFirstCall = true;
static SWITCH_STATE_E  measSbLast = MID_POS;

/* 轮子累计转角（C620 反馈的转子角度，8192 counts/圈）。
 * 用途：把"轮子实际滚了多远"和"车是否真的按指令走"分开——
 * P 速度环有静差（负载越大速度掉得越多），光用时间×速度算距离会把几何误差算歪。 */
static int16_t measAngleLast[4] = {0, 0, 0, 0};
static int64_t measWheelCounts[4] = {0, 0, 0, 0};

static void MeasureWheelsStart(void)
{
    for (uint8_t i = 0; i < 4u; ++i) {
        Motor *m = Motor::motorPtrs[0][i];
        measAngleLast[i] = (m != nullptr) ? m->feedback.angle : 0;
        measWheelCounts[i] = 0;
    }
}

static void MeasureWheelsAccumulate(void)
{
    for (uint8_t i = 0; i < 4u; ++i) {
        Motor *m = Motor::motorPtrs[0][i];
        if (m == nullptr) {
            continue;
        }
        const int16_t a = m->feedback.angle;
        int16_t d = (int16_t)(a - measAngleLast[i]);
        if (d > 4096) {
            d -= 8192;
        } else if (d < -4096) {
            d += 8192;
        }
        measWheelCounts[i] += (d < 0) ? -d : d;
        measAngleLast[i] = a;
    }
}

static int32_t MeasureWheelsAvgCounts(void)
{
    int64_t sum = 0;
    uint8_t n = 0u;
    for (uint8_t i = 0; i < 4u; ++i) {
        if (Motor::motorPtrs[0][i] != nullptr) {
            sum += measWheelCounts[i];
            ++n;
        }
    }
    return (n != 0u) ? (int32_t)(sum / n) : 0;
}

static float yawDegDelta(float from, float to)
{
    return wrapPI(to - from) * 57.2957795f;
}

static void MeasureAbort(void)
{
    if (measState != MEAS_IDLE) {
        measState = MEAS_IDLE;
        ChassisStop();
    }
}

/* 返回 true = 测量模式接管底盘（调用者不要再走正常控制逻辑） */
static bool MeasureHandle(const RC_Info_t &rc)
{
    const SWITCH_STATE_E sb = rc.sw[0];   // CH5 = SB（三档）

    if (measFirstCall) {          // 上电第一次只记录档位，避免开机就触发
        measSbLast = sb;
        measFirstCall = false;
        return false;
    }
    if (sb == MID_POS) {          // 中档 = 正常驾驶
        measState = MEAS_IDLE;
        measSbLast = sb;
        return false;
    }

    const bool edge = (measSbLast == MID_POS);
    if (edge && measState == MEAS_IDLE) {
        if (sb == UP_POS) {                    // ===== 直行测试 =====
            measState = MEAS_RUN_STRAIGHT;
            measStartTick = HAL_GetTick();
            measYawStart = IMU::getYaw();
            MeasureWheelsStart();
            headlessActive = false;
        } else if (sb == DOWN_POS) {           // ===== 自转测试 =====
            if (IMU::errorCode != 0u) {        // 转角要靠 IMU 测
                measureResultFlag = 3u;
                measSbLast = sb;
                return true;
            }
            measState = MEAS_RUN_ROTATE;
            measStartTick = HAL_GetTick();
            measYawStart = IMU::getYaw();
            measYawLast = measYawStart;
            measRotAccumDeg = 0.0f;
            MeasureWheelsStart();
            headlessActive = false;
        }
    }

    if (measState == MEAS_RUN_STRAIGHT) {
        MeasureWheelsAccumulate();
        const uint32_t elapsed = HAL_GetTick() - measStartTick;
        if (elapsed < MEAS_STRAIGHT_HOLD_MS) {
            // 匀速段
            ChassisSetVelocity(MEAS_STRAIGHT_SPEED, 0.0f, 0.0f);
        } else if (elapsed < MEAS_STRAIGHT_HOLD_MS + MEAS_STRAIGHT_DECEL_MS) {
            // 线性减速段：目标 0.5 → 0（避免直接从 0.5 m/s 断力矩，靠摩擦滑停会甩头）
            const float frac = (float)(elapsed - MEAS_STRAIGHT_HOLD_MS) / (float)MEAS_STRAIGHT_DECEL_MS;
            ChassisSetVelocity(MEAS_STRAIGHT_SPEED * (1.0f - frac), 0.0f, 0.0f);
        } else if (elapsed < MEAS_STRAIGHT_TIME_MS) {
            // 闭环刹停段：目标 0，让速度环把四个轮子一起压到 0
            ChassisSetVelocity(0.0f, 0.0f, 0.0f);
        } else {
            ChassisStop();
            measState = MEAS_IDLE;
            measStraightResult.seq++;
            measStraightResult.elapsedMs    = MEAS_STRAIGHT_TIME_MS;
            // 名义距离 = 匀速段 + 减速段（三角形面积）= 0.5×3.8 + 0.5×0.5×0.4 = 2.000 m
            measStraightResult.nominalMm    = (int32_t)(MEAS_STRAIGHT_SPEED * (float)MEAS_STRAIGHT_HOLD_MS
                                                        + 0.5f * MEAS_STRAIGHT_SPEED * (float)MEAS_STRAIGHT_DECEL_MS);
            measStraightResult.measuredX100 = (int32_t)(yawDegDelta(measYawStart, IMU::getYaw()) * 100.0f);
            measStraightResult.kRealMm      = 0;
            measStraightResult.wheelCounts  = MeasureWheelsAvgCounts();
            measureResultFlag = 1u;
        }
        measSbLast = sb;
        return true;
    }

    if (measState == MEAS_RUN_ROTATE) {
        // 每 1ms 把 yaw 增量累加，整圈也不会折回（首尾差只能得到 ±180°）
        {
            const float yawNow = IMU::getYaw();
            measRotAccumDeg += wrapPI(yawNow - measYawLast) * 57.2957795f;
            measYawLast = yawNow;
        }
        MeasureWheelsAccumulate();
        if (HAL_GetTick() - measStartTick < MEAS_ROTATE_TIME_MS) {
            ChassisSetVelocity(0.0f, 0.0f, MEAS_ROTATE_RPM);
        } else {
            ChassisStop();
            measState = MEAS_IDLE;
            const float actualDeg = measRotAccumDeg;
            const float kCode = (TRACK_WIDTH + WHEEL_BASE) * 0.5f;
            const float kReal = (fabsf(actualDeg) > 1.0f) ? (kCode * 360.0f / fabsf(actualDeg)) : kCode;
            measRotateResult.seq++;
            measRotateResult.elapsedMs    = MEAS_ROTATE_TIME_MS;
            measRotateResult.nominalMm    = 0;
            measRotateResult.measuredX100 = (int32_t)(actualDeg * 100.0f);
            measRotateResult.kRealMm      = (int32_t)(kReal * 1000.0f);
            measRotateResult.wheelCounts  = MeasureWheelsAvgCounts();
            measureResultFlag = 2u;
        }
        measSbLast = sb;
        return true;
    }

    // 停在测试档位但没在跑：保持停车（要再测先回中档）
    measSbLast = sb;
    ChassisStop();
    return true;
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
            MeasureAbort();
            return;
        }
    }

    // ===== 急停：SD 拨到最下档（SC 下档先保留兜底，确认 SD 通道无误后可删掉后半句） =====
    if (SW_SD == DOWN_POS || rc.sRight == DOWN_POS) {
        ChassisStop();
        UserStop();
        headlessActive = false;
        MeasureAbort();
        return;
    }

    // ===== 测量/标定模式（SB 三档）：上=直行、中=正常、下=自转；接管底盘时摇杆无效 =====
    if (MeasureHandle(rc)) {
        return;
    }

    // SA 若是两档开关，把下档当成"车头模式"（三位开关时下档是停车）
    SWITCH_STATE_E saPos = SW_SA;
#if (SA_IS_THREE_POSITION == 0)
    if (saPos == DOWN_POS) {
        saPos = MID_POS;
    }
#endif

    switch (saPos) {
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
    // ===================== 车头模式（SA 两档时：下档；SA 三位时：中档） =====================
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
    // ===================== 底盘停车（仅 SA 为三位开关时可达；两档时停车请用 SD 急停） =====================
    case DOWN_POS:
    default: {
        headlessActive = false;
        ChassisStop();
        break;
    }
    }
}
