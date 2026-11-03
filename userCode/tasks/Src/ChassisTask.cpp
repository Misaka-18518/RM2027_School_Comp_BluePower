//
// Created by LEGION on 2021/10/4.
//
#include "ChassisTask.h"

PID_Regulator_t pidRegulator = {//此为储存pid参数的结构体，四个底盘电机共用
        .kp = 60,
        .ki = 0,
        .kd = 0,
        .componentKpMax = 16384,
        .componentKiMax = 0,
        .componentKdMax = 0,
        .outputMax = 16384 //3508电机输出电流上限，可以调小，勿调大
};

MOTOR_INIT_t chassisMotorInit = {//四个底盘电机共用的初始化结构体
        .speedPIDp = &pidRegulator,
        .anglePIDp = nullptr,
        ._motorID = MOTOR_ID_1,
        .reductionRatio = 19.0f,
        .ctrlType = SPEED_Single,
};
/* ==================== 底盘电机总线 ID 配置 ====================
 * 电调：C620（配 M3508），全部挂在 CAN1 总线上
 *   FL 左前 = ID 1
 *   FR 右前 = ID 2
 *   BL 左后 = ID 3
 *   BR 右后 = ID 4
 *
 * 说明：MOTOR_ID_1 ~ MOTOR_ID_8 对应 CAN1 上的 1~8 号电调；9~16 才走 CAN2。
 * 改 ID 只改下面四行；同时记得把电调本身的 ID 也设成一样的
 * （RoboMaster Assistant 里改，或按电调上的 SET 键：短按进入 ID 模式，
 *   短按 N 次 = ID N，停 3 秒自动保存，重新上电生效）。
 */
#define CHASSIS_ID_FL   MOTOR_ID_1
#define CHASSIS_ID_FR   MOTOR_ID_2
#define CHASSIS_ID_BL   MOTOR_ID_3
#define CHASSIS_ID_BR   MOTOR_ID_4

// 编译期检查：四个 ID 不能重复
static_assert((CHASSIS_ID_FL & CHASSIS_ID_FR) == 0u, "底盘电机 ID 重复: FL / FR");
static_assert((CHASSIS_ID_FL & CHASSIS_ID_BL) == 0u, "底盘电机 ID 重复: FL / BL");
static_assert((CHASSIS_ID_FL & CHASSIS_ID_BR) == 0u, "底盘电机 ID 重复: FL / BR");
static_assert((CHASSIS_ID_FR & CHASSIS_ID_BL) == 0u, "底盘电机 ID 重复: FR / BL");
static_assert((CHASSIS_ID_FR & CHASSIS_ID_BR) == 0u, "底盘电机 ID 重复: FR / BR");
static_assert((CHASSIS_ID_BL & CHASSIS_ID_BR) == 0u, "底盘电机 ID 重复: BL / BR");
// 编译期检查：四个 ID 必须都在 CAN1 的 1~8 范围内
static_assert(((CHASSIS_ID_FL | CHASSIS_ID_FR | CHASSIS_ID_BL | CHASSIS_ID_BR) & 0xFFFFFF00u) == 0u,
              "底盘电机 ID 必须在 CAN1 的 1~8 范围内");

Motor CMFL(CHASSIS_ID_FL,&chassisMotorInit);//左前 FL，ID 1，CAN1
Motor CMFR(CHASSIS_ID_FR,&chassisMotorInit);//右前 FR，ID 2，CAN1
Motor CMBL(CHASSIS_ID_BL,&chassisMotorInit);//左后 BL，ID 3，CAN1
Motor CMBR(CHASSIS_ID_BR,&chassisMotorInit);//右后 BR，ID 4，CAN1

uint8_t ChassisStopFlag = 1;
float FBVelocity,LRVelocity,RTVelocity;

void ChassisStart(){

}
/**
 * @brief 底盘任务的处理函数，定时执行
 * @callergraph void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) in Device.cpp
 */
void ChassisHandle() {
    if(ChassisStopFlag == 0) {
        WheelsSpeedCalc(FBVelocity, LRVelocity, RTVelocity);
    }
    CMFL.Handle();
    CMFR.Handle();
    CMBL.Handle();
    CMBR.Handle();
}
/**
 * @brief 用于控制任务控制底盘速度
 * @param _fbV 底盘前后方向速度
 * @param _lrV 底盘左右方向速度
 * @param _rtV 底盘旋转速度
 */
void ChassisSetVelocity(float _fbV,float _lrV,float _rtV){
    ChassisStopFlag = 0;
    FBVelocity = _fbV;
    LRVelocity = _lrV;
    RTVelocity = _rtV;
}
/**
 * @brief 执行急停模式的底盘任务处理
 */
void ChassisStop(){
    ChassisStopFlag = 1;
    CMFL.Stop();
    CMFR.Stop();
    CMBL.Stop();
    CMBR.Stop();
}

void WheelsSpeedCalc(float fbVelocity, float lrVelocity, float rtVelocity) {
    float CMFLSpeed, CMFRSpeed, CMBLSpeed, CMBRSpeed;

    // ===================== 麦轮逆运动学（无头控制也走这里） =====================
    // 约定：fbVelocity 前为正(m/s)；lrVelocity 右为正(m/s)；rtVelocity 逆时针为正(RPM)
    // 车体坐标系：x 向前、y 向左
    float omega = RPM2RADpS(rtVelocity);              // 自转角速度 rad/s，逆时针为正
    float vx = fbVelocity;                            // 向前 m/s
    float vy = -lrVelocity;                           // 向左 m/s（参数是右为正）
    const float k = (TRACK_WIDTH + WHEEL_BASE) * 0.5f; // 麦轮自转等效力臂，0.39 m

    // 四个轮子线速度 = 前后 + 左右 + 自转
    CMFLSpeed = vx - vy - omega * k;
    CMFRSpeed = vx + vy + omega * k;
    CMBLSpeed = vx + vy - omega * k;
    CMBRSpeed = vx - vy + omega * k;

    // 装车后若某个轮子方向相反，把对应 dir 改成 -1.0f（一次改一个，看现象）
    // 当前：ID2(FR)、ID4(BR) 方向反了 -> 取 -1（右侧电机镜像安装/接线方向不同）
    const float dirFL = 1.0f;
    const float dirFR = -1.0f;
    const float dirBL = 1.0f;
    const float dirBR = -1.0f;

    // 轮子线速度 → 轮子角速度(rad/s) → 输出轴转速(RPM)
    CMFLSpeed = dirFL * CMFLSpeed /(WHEEL_DIAMETER/2.0f);
    CMFRSpeed = dirFR * CMFRSpeed /(WHEEL_DIAMETER/2.0f);
    CMBLSpeed = dirBL * CMBLSpeed /(WHEEL_DIAMETER/2.0f);
    CMBRSpeed = dirBR * CMBRSpeed /(WHEEL_DIAMETER/2.0f);
    //控制底盘电机转速
    CMFL.SetTargetSpeed(RADpS2RPM(CMFLSpeed));
    CMFR.SetTargetSpeed(RADpS2RPM(CMFRSpeed));
    CMBL.SetTargetSpeed(RADpS2RPM(CMBLSpeed));
    CMBR.SetTargetSpeed(RADpS2RPM(CMBRSpeed));
}
