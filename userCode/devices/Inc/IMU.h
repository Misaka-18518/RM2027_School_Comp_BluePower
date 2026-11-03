//
// IMU.h —— BMI088 陀螺仪 + 航向角（yaw）估计
// 无头控制用：提供相对上电/复位时刻的车头朝向
//
#ifndef RM_FRAME_C_IMU_H
#define RM_FRAME_C_IMU_H

#include "main.h"

/**
 * @brief BMI088 六轴 IMU（只使用陀螺仪 Z 轴做航向估计）
 *        用法：main 里 IMU::Init()，1ms 定时器里 IMU::Handle()，控制任务里 IMU::getYaw()
 */
class IMU {
public:
    /**
     * @brief 初始化 BMI088 并做陀螺零偏校准（阻塞约 1~2 秒，只在开机时调用一次）
     * @note  失败时 errorCode 非 0，但不会卡死
     */
    static void Init();

    /**
     * @brief 1ms 调用一次：读陀螺 Z 轴并积分出 yaw
     */
    static void Handle();

    /** 当前 yaw（rad），逆时针（向左转）为正，范围 [-pi, pi] */
    static float getYaw();

    /** 陀螺 Z 轴角速度（rad/s，已去零偏），调试/PID 微分项用 */
    static float getGyroZ();

    /** 把当前车头方向设为指定角度（一般设为 0） */
    static void resetYaw(float yaw = 0.0f);

    /** 初始化结果：0 成功；非 0 为错误码，可用 USB 调试口打印 */
    static uint8_t errorCode;
    static uint8_t debugAccelId;   // 最近读到的 ACC 芯片 ID（正常 0x1E）
    static uint8_t debugGyroId;    // 最近读到的 GYRO 芯片 ID（正常 0x0F）

private:
    static float yaw_;
    static float gyroBias_[3];
    static float gyro_[3];
    static uint8_t initAccel_();
    static uint8_t initGyro_();
    static void readGyro_(float gyro[3]);
};

#endif //RM_FRAME_C_IMU_H
