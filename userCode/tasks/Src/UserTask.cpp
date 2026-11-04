//
// Created by LEGION on 2021/10/4.
//
#include "UserTask.h"

PID_Regulator_t userPidRegulator = {
        .kp = USER_PID_KP,
        .ki = USER_PID_KI,
        .kd = USER_PID_KD,
        .componentKpMax = USER_PID_KP_MAX,
        .componentKiMax = USER_PID_KI_MAX,
        .componentKdMax = USER_PID_KD_MAX,
        .outputMax = USER_PID_OUT_MAX
};

MOTOR_INIT_t userMotorInit = {
        .speedPIDp = &userPidRegulator,
        .anglePIDp = nullptr,
        ._motorID = MOTOR_ID_1,   // 实际 ID 由下面构造函数传入 USER_MOTOR_ID 覆盖
        .reductionRatio = USER_MOTOR_REDUCTION,
        .ctrlType = SPEED_Single,
};

// 编译期检查：Config.h 的 USER_MOTOR_ID 必须是 CAN1 ID5 对应的位掩码
static_assert(USER_MOTOR_ID == MOTOR_ID_5, "Config.h: USER_MOTOR_ID 必须对应 MOTOR_ID_5");

// 在这里初始化除了4个M3508轮电机之外的电机
Motor UserMotor(USER_MOTOR_ID, &userMotorInit);

/***
 * 在这里放入xxx.stop()即可使舵机,电机在遥控器急停挡位断电
 */
void UserStop(){
    UserMotor.Stop();
}

/***
 * 在这里写入初始化内容
 */
void UserInit(){

}

/***
 * 用户自定义任务主循环
 */
void UserHandle(){
    UserMotor.Handle();
}