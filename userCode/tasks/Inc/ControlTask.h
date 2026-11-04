//
// Created by LEGION on 2021/10/4.
//

#ifndef RM_FRAME_C_CONTROLTASK_H
#define RM_FRAME_C_CONTROLTASK_H

#include "main.h"
#include "ChassisTask.h"
#include "Motor.h"
#include "UserTask.h"
#include "RemoteControl.h"

/*枚举类型定义------------------------------------------------------------*/

/*
 * 控制任务错误信息枚举
 */
typedef enum{
    NONE1,
}CTRL_ERROR_E;

/*结构体定义--------------------------------------------------------------*/

/*
 * 控制任务状态结构体
 */
typedef struct{

}CTRL_STATE_T;

/*
 * 控制任务控制结构体
 */
typedef struct{


}CTRL_CONTROL_T;



/*结构体成员取值定义组------------------------------------------------------*/

/*
 * 底盘标定/测量模式结果（ControlTask.cpp 的测量模式写入，Device.cpp 从 USB CDC 打印）
 */
typedef struct {
    uint32_t seq;           // 每完成一次测试 +1（0 = 还没测过）
    uint32_t elapsedMs;     // 测试时长
    int32_t  nominalMm;     // 直行：名义距离 mm
    int32_t  measuredX100;  // 直行：yaw 漂移 deg×100；自转：实际转角 deg×100
    int32_t  kRealMm;       // 自转：建议的 k = (TRACK_WIDTH + WHEEL_BASE)/2，单位 mm
    int32_t  wheelCounts;   // 测试期间轮子平均累计转角（C620 反馈，转子 counts，8192 counts/圈）
} MEASURE_RESULT_T;

/*外部变量声明-------------------------------------------------------------*/

extern CTRL_STATE_T     ctrlState;
extern CTRL_CONTROL_T   ctrlControl;

extern volatile uint8_t        measureResultFlag;   // 1=直行结果 2=自转结果 3=IMU 不可用
// 最近一次测试结果常驻内存：测的时候不用连 ST-Link，测完（断电前）插上就能读；seq 为结果序号
extern volatile MEASURE_RESULT_T measStraightResult;  // 最近一次"直行测试"结果
extern volatile MEASURE_RESULT_T measRotateResult;    // 最近一次"自转测试"结果

/*外部函数声明-------------------------------------------------------------*/

void CtrlHandle();   // 实际实现是 void CtrlHandle()（无参），原型的旧版本对不上，已清理

#endif //RM_FRAME_C_CONTROLTASK_H
