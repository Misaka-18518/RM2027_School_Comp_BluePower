//
// Created by LEGION on 2021/10/4.
//

#include "Motor.h"

/* 零速停止死区 MOTOR_STOP_DEADBAND_RPM 已移到 Config.h（第 3 节） */

Motor* Motor::motorPtrs[2][8] = {0};
int16_t Motor::motor_intensity[16];
uint32_t Motor::motor_IDs;
uint8_t Motor::idErrorFlag = 0;

/**
 * @brief 检查电机 ID 是否合法：必须是 MOTOR_ID_1 ~ MOTOR_ID_16 之一，且不能与已定义电机重复
 * @return 0 = 合法；1 = 非法/冲突
 */
static uint8_t motorIdInvalid(uint32_t id)
{
    const uint32_t allIdMask = MOTOR_ID_1 | MOTOR_ID_2 | MOTOR_ID_3 | MOTOR_ID_4 |
                               MOTOR_ID_5 | MOTOR_ID_6 | MOTOR_ID_7 | MOTOR_ID_8 |
                               MOTOR_ID_9 | MOTOR_ID_10 | MOTOR_ID_11 | MOTOR_ID_12 |
                               MOTOR_ID_13 | MOTOR_ID_14 | MOTOR_ID_15 | MOTOR_ID_16;
    if (id == 0u) return 1u;
    if ((id & (id - 1u)) != 0u) return 1u;          // ID 必须只占一个 bit（2 的幂）
    if ((id & allIdMask) == 0u) return 1u;          // 必须是 1~16 号
    if ((Motor::motor_IDs & id) != 0u) return 1u;   // 已经被别的电机对象占用
    return 0u;
}

/**
 * 计算pid算法的控制量
 * @param target 目标量
 * @param feedback 反馈量
 * @return 控制量
 */
float PID::PIDCalc(float target,float feedback) {
    PIDInfo.fdb = feedback;
    PIDInfo.ref = target;
    PIDInfo.err[3] = PIDInfo.ref - PIDInfo.fdb;
    PIDInfo.componentKp = PIDInfo.err[3] * PIDInfo.kp;
    PIDInfo.errSum += PIDInfo.err[3];
    INRANGE(PIDInfo.errSum, -1 * PIDInfo.componentKiMax / PIDInfo.ki, PIDInfo.componentKiMax / PIDInfo.ki);
    PIDInfo.componentKi = PIDInfo.errSum * PIDInfo.ki;
    PIDInfo.componentKd = (PIDInfo.err[3] - PIDInfo.err[2]) * PIDInfo.kd;
    INRANGE(PIDInfo.componentKp, -1 * PIDInfo.componentKpMax, PIDInfo.componentKpMax);
    INRANGE(PIDInfo.componentKi, -1 * PIDInfo.componentKiMax, PIDInfo.componentKiMax);
    INRANGE(PIDInfo.componentKd, -1 * PIDInfo.componentKdMax, PIDInfo.componentKdMax);
    PIDInfo.output = PIDInfo.componentKp + PIDInfo.componentKi + PIDInfo.componentKd;
    INRANGE(PIDInfo.output, -1 * PIDInfo.outputMax, PIDInfo.outputMax);
    PIDInfo.err[2] = PIDInfo.err[3];
    return PIDInfo.output;
}


/**
 * @brief 电机类的初始化，主要是CAN通信的相关配置
 * @callergraph int main(void) in Device.cpp
 */
void Motor::Init(){
    HAL_CAN_Start(&hcan1);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

    CAN_FilterTypeDef canFilterTypeDef;

    canFilterTypeDef.FilterMode = CAN_FILTERMODE_IDMASK;
    canFilterTypeDef.FilterScale = CAN_FILTERSCALE_32BIT;
    canFilterTypeDef.FilterIdHigh = 0x0000;
    canFilterTypeDef.FilterIdLow = 0x0000;
    canFilterTypeDef.FilterMaskIdHigh = 0x0000;
    canFilterTypeDef.FilterMaskIdLow = 0x0000;
    canFilterTypeDef.FilterFIFOAssignment = CAN_RX_FIFO0;
    canFilterTypeDef.FilterActivation = ENABLE;
    canFilterTypeDef.FilterBank = 0;
    canFilterTypeDef.SlaveStartFilterBank = 0;

    HAL_CAN_ConfigFilter(&hcan1, &canFilterTypeDef);
    HAL_CAN_ConfigFilter(&hcan2, &canFilterTypeDef);
}
/**
 * @brief can消息包发送任务
 * @callergraph void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
 *              in Device.cpp
 */
void Motor::CANPackageSend() {
    uint8_t canBuf[8];

    CAN_TxHeaderTypeDef txHeaderTypeDef;


    txHeaderTypeDef.DLC = 0x08;
    txHeaderTypeDef.IDE = CAN_ID_STD;
    txHeaderTypeDef.RTR = CAN_RTR_DATA;
    txHeaderTypeDef.TransmitGlobalTime = DISABLE;

    uint32_t txMailbox;   // HAL 把实际使用的邮箱号写回这里（原来传的是地址 0x1 的野指针）

    if(motor_IDs & MOTOR_ID_CAN1_1_MASK) {
        for (int i = 0; i < 4; ++i) {
            canBuf[2 * i] = motor_intensity[i] >> 8u;
            canBuf[2 * i + 1] = motor_intensity[i];
        }
        txHeaderTypeDef.StdId = 0x200;
        HAL_CAN_AddTxMessage(&hcan1, &txHeaderTypeDef, canBuf, &txMailbox);
    }

    if(motor_IDs & MOTOR_ID_CAN1_2_MASK) {
        for (int i = 0; i < 4; ++i) {
            canBuf[2 * i] = motor_intensity[i + 4] >> 8u;
            canBuf[2 * i + 1] = motor_intensity[i + 4];
        }
        txHeaderTypeDef.StdId = 0x1FF;
        HAL_CAN_AddTxMessage(&hcan1, &txHeaderTypeDef, canBuf, &txMailbox);
    }

    if(motor_IDs & MOTOR_ID_CAN2_1_MASK) {
        for (int i = 0; i < 4; ++i) {
            canBuf[2 * i] = motor_intensity[i + 8] >> 8u;
            canBuf[2 * i + 1] = motor_intensity[i + 8];
        }
        txHeaderTypeDef.StdId = 0x200;
        HAL_CAN_AddTxMessage(&hcan2, &txHeaderTypeDef, canBuf, &txMailbox);
    }

    if(motor_IDs & MOTOR_ID_CAN2_2_MASK) {
        for (int i = 0; i < 4; ++i) {
            canBuf[2 * i] = motor_intensity[i + 12] >> 8u;
            canBuf[2 * i + 1] = motor_intensity[i + 12];
        }
        txHeaderTypeDef.StdId = 0x1FF;
        HAL_CAN_AddTxMessage(&hcan2, &txHeaderTypeDef, canBuf, &txMailbox);
    }
}
/**
 * @brief Motor类的构造函数
 * @param _init 类的初始化结构体指针
 */
Motor::Motor(MOTOR_INIT_t* _init){

    deviceID = _init->_motorID;
    deviceType = MOTOR;

    // ===== 电机 ID 合法性 & 冲突检查（重复 ID 会让 CAN 反馈串到别的对象）=====
    if (motorIdInvalid(_init->_motorID)) {
        idConflictFlag = 1u;
        idErrorFlag = 1u;
        stopFlag = 1u;
        return;   // 不注册进 motorPtrs / motor_IDs，Handle() 也不会输出
    }

    auto motorPos = GET_MOTOR_POS(_init->_motorID);
    motorPtrs[motorPos/8][motorPos%8] = this;
    motor_IDs |= _init->_motorID;
    memset(&feedback,0,sizeof(C6x0Rx_t));

    if(_init->speedPIDp) speedPID.PIDInfo = *_init->speedPIDp;
    if(_init->anglePIDp) anglePID.PIDInfo = *_init->anglePIDp;

    ctrlType = _init->ctrlType;
    reductionRatio = _init->reductionRatio;
    targetSpeed = 0;
}
/**
 * @brief Motor类的构造函数另一重载，可以方便地定义参数相同，ID不同的电机
 * @param _id 电机ID
 * @param _init 电机初始化结构体指针
 */
Motor::Motor(uint32_t _id, MOTOR_INIT_t* _init) {
    _init->_motorID = _id;

    deviceID = _init->_motorID;
    deviceType = MOTOR;

    // ===== 电机 ID 合法性 & 冲突检查 =====
    if (motorIdInvalid(_init->_motorID)) {
        idConflictFlag = 1u;
        idErrorFlag = 1u;
        stopFlag = 1u;
        return;
    }

    auto motorPos = GET_MOTOR_POS(_init->_motorID);
    motorPtrs[motorPos/8][motorPos%8] = this;
    motor_IDs |= _init->_motorID;
    memset(&feedback,0,sizeof(C6x0Rx_t));

    if(_init->speedPIDp) speedPID.PIDInfo = *_init->speedPIDp;
    if(_init->anglePIDp) anglePID.PIDInfo = *_init->anglePIDp;

    ctrlType = _init->ctrlType;
    reductionRatio = _init->reductionRatio;
    targetSpeed = 0;
}
/**
 * @brief 电机类的析构函数
 */
Motor::~Motor(){
    if (!idConflictFlag) {
        motor_IDs &= (~deviceID);
    }
}
/**
 * @brief 电机类的执行处理函数，囊括每个电机的主要处理任务。此函数需定时执行，否则对应电机无法正常工作
 */
void Motor::Handle(){
    if (idConflictFlag) {
        return;   // ID 非法/重复的对象不参与控制，也不会覆盖其他电机的电流
    }
    MotorStateUpdate();

    if (stopFlag == 1){
        // 停车状态：清掉积分，避免下次启动带旧积分冲一下
        speedPID.PIDInfo.errSum = 0.0f;
        motor_intensity[GET_MOTOR_POS(deviceID)] = 0;
        return;
    }

    int16_t intensity = IntensityCalc();

    if (stopFlag == 1){
        motor_intensity[GET_MOTOR_POS(deviceID)] = 0;
    }else {
        motor_intensity[GET_MOTOR_POS(deviceID)] = intensity;
    }
}


/**
 * @brief 电机的中断处理函数，用于电调返回数据的接受
 * @param hcan
 */
void Motor::IT_Handle(CAN_HandleTypeDef *hcan) {
    uint8_t canBuf[8];
    CAN_RxHeaderTypeDef rx_header;
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, canBuf) != HAL_OK) {
        return;
    }
    // C620 反馈 ID 固定为 0x201~0x208；过滤器是全通，其他任何 ID 都不能拿来索引数组
    if ((rx_header.IDE != CAN_ID_STD) || (rx_header.StdId < 0x201u) || (rx_header.StdId > 0x208u)) {
        return;
    }
    uint8_t canPos = (hcan == &hcan1) ? 0u : 1u;
    uint8_t motorPos = (uint8_t)(rx_header.StdId - 0x201u);
    Motor *m = motorPtrs[canPos][motorPos];
    if (m == nullptr) {   // 该 ID 没有对应的 Motor 对象
        return;
    }
    m->feedback.angle = (int16_t)((canBuf[0] << 8u) | canBuf[1]);
    m->feedback.speed = (int16_t)((canBuf[2] << 8u) | canBuf[3]);
    m->feedback.moment = (int16_t)((canBuf[4] << 8u) | canBuf[5]);
    m->feedback.temp = canBuf[6];
}
/**
 * @brief 用于设置电机速度，只用当电机控制类型对应时才有效
 * @param _targetSpeed 目标速度
 */
void Motor::SetTargetSpeed(float _targetSpeed) {
    stopFlag = 0;
    targetSpeed = _targetSpeed;
}
/**
 * @brief 用于设置电机角度，只用当电机控制类型对应时才有效
 * @param _targetAngle 目标角度
 */
void Motor::SetTargetAngle(float _targetAngle) {
    stopFlag = 0;
    targetAngle = _targetAngle;
}

void Motor::Stop() {
    stopFlag = 1;
}

void Motor::ErrorHandle() {

}
/**
 * @brief 更新电机的相关状态
 * @callergraph this->Handle()
 */
void Motor::MotorStateUpdate() {
    state.speed = feedback.speed/reductionRatio;
    state.moment = feedback.moment;
    state.temperature = feedback.temp;

    float realAngle = state.angle;
    float thisAngle = feedback.angle;
    if (thisAngle <= lastRead) {
        if (lastRead - thisAngle > 4000)
            realAngle += (thisAngle + 8192 - lastRead) * 360.0f / 8192.0f / reductionRatio;
        else
            realAngle -= (lastRead - thisAngle) * 360.0f / 8192.0f / reductionRatio;
    } else {
        if (thisAngle - lastRead > 4000)
            realAngle -= (lastRead + 8192 - thisAngle) * 360.0f / 8192.0f / reductionRatio;
        else
            realAngle += (thisAngle - lastRead) * 360.0f / 8192.0f / reductionRatio;
    }

    state.angle = realAngle;
    lastRead = feedback.angle;
}
/**
 * @brief 计算电机实际控制电流
 * @return 控制电流值
 */
int16_t Motor::IntensityCalc(){
    int16_t intensity;
    switch (ctrlType) {
        case SPEED_Single:
            // 目标为 0 且轮子已接近停转 -> 输出 0，避免静止时 PID 噪声让轮子爬行
            if (fabsf(targetSpeed) < 0.1f && fabsf(state.speed) < MOTOR_STOP_DEADBAND_RPM) {
                speedPID.PIDInfo.errSum = 0.0f;   // 已经停稳，清积分
                return 0;
            }
            intensity = speedPID.PIDCalc(targetSpeed, state.speed);
            break;
        case POSITION_Double:
            float _targetSpeed = anglePID.PIDCalc(targetAngle,state.angle);
            intensity = speedPID.PIDCalc(_targetSpeed, state.speed);
            break;
    }
    return intensity;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
    Motor::IT_Handle(hcan);
}
