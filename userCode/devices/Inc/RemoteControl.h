//
// Created by LEGION on 2021/10/10.
//

#ifndef RM_FRAME_C_REMOTECONTROL_H
#define RM_FRAME_C_REMOTECONTROL_H

#include "Device.h"

/* SBUS_RX_BUF_NUM / RC_FRAME_LENGTH / RC_CH_VALUE_OFFSET 已移到 Config.h */
/*枚举类型定义------------------------------------------------------------*/
typedef enum{
    UP_POS = 97u,MID_POS = 0u,DOWN_POS = 158u
}SWITCH_STATE_E;
/*结构体定义--------------------------------------------------------------*/
typedef struct {
    int32_t ch[4];
    uint32_t s[2];
    int16_t optional[2];
}rc_t;

typedef struct {
    int32_t x,y,z,press_l,press_r;
}mouse_t;

typedef struct {
    uint32_t v;
}keyboard_t;

typedef struct {
    rc_t rc;
    mouse_t mouse;
    keyboard_t key;
}RC_ctrl_t;
typedef struct {
    float left_col,left_rol;
    float right_col,right_rol;
    SWITCH_STATE_E sLeft;        // 兼容旧代码，等于 sw[0]（CH5）
    uint32_t pat;
    SWITCH_STATE_E sRight;       // 兼容旧代码，等于 sw[1]（CH6）
    int16_t optional[2];
    SWITCH_STATE_E sw[4];        // CH5~CH8 四个开关的位置：sw[0]=CH5 ... sw[3]=CH8
}RC_Info_t;
/*类型定义----------------------------------------------------------------*/
class RemoteControl : public Device {
public:
    static RC_Info_t rcInfo;
    static RC_ctrl_t rc_ctrl;
    static uint32_t lastFrameTick;   // 最近一次成功解析遥控帧的时间（HAL_GetTick），调试用

    static void init();
    static void ITHandle();
    static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
    static uint16_t debugCh[16];   // 调试：SBUS 全部 16 通道原始值（0~2047）
    static uint16_t debugChMin[16]; // 调试：各通道历史最小值
    static uint16_t debugChMax[16]; // 调试：各通道历史最大值
    static void sbus_to_rc(volatile const uint8_t *sbus_buf);
};

/*结构体成员取值定义组------------------------------------------------------*/

/*外部变量声明-------------------------------------------------------------*/

/*外部函数声明-------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif
extern void USART3_IRQHandler(void);
#ifdef __cplusplus
}
#endif

#endif //RM_FRAME_C_REMOTECONTROL_H
