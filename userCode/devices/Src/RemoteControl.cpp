//
// Created by LEGION on 2021/10/10.
//

#include "RemoteControl.h"

RC_Info_t RemoteControl::rcInfo;
RC_ctrl_t RemoteControl::rc_ctrl;
uint32_t RemoteControl::lastFrameTick = 0;

uint8_t RemoteControl::sbus_rx_buf[2][SBUS_RX_BUF_NUM];
uint16_t RemoteControl::debugCh[16];
uint16_t RemoteControl::debugChMin[16];
uint16_t RemoteControl::debugChMax[16];

void RemoteControl::init() {
    // 调试通道 min/max 初始化
    for (uint8_t dbgI = 0; dbgI < 16; ++dbgI) {
        debugChMin[dbgI] = 0xFFFFu;
        debugChMax[dbgI] = 0x0000u;
    }
    //enable the DMA transfer for the receiver request
    //使能 DMA 串口接收
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);
    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
    //disable DMA
    //失效 DMA
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }
    hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);
    //memory buffer 1
    //内存缓冲区 1
    hdma_usart3_rx.Instance->M0AR = (uint32_t)(sbus_rx_buf[0]);
    //memory buffer 2
    //内存缓冲区 2
    hdma_usart3_rx.Instance->M1AR = (uint32_t)(sbus_rx_buf[1]);
    //data length
    //数据长度
    hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;
    //enable double memory buffer
    //使能双缓冲区
    CLEAR_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_CIRC);
    //enable DMA
    //使能 DMA
    __HAL_DMA_ENABLE(&hdma_usart3_rx);
}


/* 开关原始值(0~2047) → 位置：阈值见 Config.h（RC_SW_UP_THRESHOLD / RC_SW_DOWN_THRESHOLD）。
 * 实测 ET08 三位开关 上=353 / 中=1024 / 下=1694；两档开关只会出现 上/下。 */
static SWITCH_STATE_E rawToSwitch(uint32_t raw)
{
    if (raw < RC_SW_UP_THRESHOLD)   return UP_POS;
    if (raw > RC_SW_DOWN_THRESHOLD) return DOWN_POS;
    return MID_POS;
}

void RemoteControl::sbus_to_rc(const volatile uint8_t *sbus_buf) {
    // ===== 调试：解出 SBUS 全部 16 个通道原始值（0~2047），用于排查开关通道 =====
    debugCh[0]  = (sbus_buf[1] | (sbus_buf[2] << 8)) & 0x07ff;
    debugCh[1]  = ((sbus_buf[2] >> 3) | (sbus_buf[3] << 5)) & 0x07ff;
    debugCh[2]  = ((sbus_buf[3] >> 6) | (sbus_buf[4] << 2) | (sbus_buf[5] << 10)) & 0x07ff;
    debugCh[3]  = ((sbus_buf[5] >> 1) | (sbus_buf[6] << 7)) & 0x07ff;
    debugCh[4]  = ((sbus_buf[6] >> 4) | (sbus_buf[7] << 4)) & 0x07ff;
    debugCh[5]  = ((sbus_buf[7] >> 7) | (sbus_buf[8] << 1) | (sbus_buf[9] << 9)) & 0x07ff;
    debugCh[6]  = ((sbus_buf[9] >> 2) | (sbus_buf[10] << 6)) & 0x07ff;
    debugCh[7]  = ((sbus_buf[10] >> 5) | (sbus_buf[11] << 3)) & 0x07ff;
    debugCh[8]  = (sbus_buf[12] | (sbus_buf[13] << 8)) & 0x07ff;
    debugCh[9]  = ((sbus_buf[13] >> 3) | (sbus_buf[14] << 5)) & 0x07ff;
    debugCh[10] = ((sbus_buf[14] >> 6) | (sbus_buf[15] << 2) | (sbus_buf[16] << 10)) & 0x07ff;
    debugCh[11] = ((sbus_buf[16] >> 1) | (sbus_buf[17] << 7)) & 0x07ff;
    debugCh[12] = ((sbus_buf[17] >> 4) | (sbus_buf[18] << 4)) & 0x07ff;
    debugCh[13] = ((sbus_buf[18] >> 7) | (sbus_buf[19] << 1) | (sbus_buf[20] << 9)) & 0x07ff;
    debugCh[14] = ((sbus_buf[20] >> 2) | (sbus_buf[21] << 6)) & 0x07ff;
    debugCh[15] = ((sbus_buf[21] >> 5) | (sbus_buf[22] << 3)) & 0x07ff;
    rc_ctrl.rc.ch[0] = (sbus_buf[1] | sbus_buf[2] << 8) & 0x07ff;       // Channel 1
    rc_ctrl.rc.ch[1] = (sbus_buf[2] >> 3 | sbus_buf[3] << 5) & 0x07ff;  // Channel 2
    rc_ctrl.rc.ch[2] = (sbus_buf[3] >> 6 | sbus_buf[4] << 2 | sbus_buf[5] << 10) &
                    0x07ff;                                          // Channel 3
    rc_ctrl.rc.ch[3] = (sbus_buf[5] >> 1 | sbus_buf[6] << 7) & 0x07ff;  // Channel 4
    rc_ctrl.rc.s[0] = (sbus_buf[6] >> 4 | sbus_buf[7] << 4) & 0x07ff;   // Channel 5
    rc_ctrl.rc.s[1] = (sbus_buf[7] >> 7 | sbus_buf[8] << 1 | sbus_buf[9] << 9) &
                   0x07ff;   // Channel 6
    rc_ctrl.rc.optional[0] = (sbus_buf[9] >> 2 | sbus_buf[10] << 6) & 0x07ff;   // Channel 7
    rc_ctrl.rc.optional[1] = (sbus_buf[10] >> 5 | sbus_buf[11] << 3) & 0x07ff;  // Channel 8
    rc_ctrl.rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rcInfo.right_rol = rc_ctrl.rc.ch[0]/671.0f;
    rcInfo.left_col = rc_ctrl.rc.ch[1]/671.0f;
    rcInfo.right_col = rc_ctrl.rc.ch[2]/671.0f;
    rcInfo.left_rol = rc_ctrl.rc.ch[3]/671.0f;
    /* 四个开关全部解成位置：sw[0]=CH5, sw[1]=CH6, sw[2]=CH7, sw[3]=CH8。
     * 按原始值区间判断（实测 ET08：上=353、中=1024、下=1694），比直接强转枚举可靠。 */
    rcInfo.sw[0] = rawToSwitch((uint32_t)rc_ctrl.rc.s[0]);
    rcInfo.sw[1] = rawToSwitch((uint32_t)rc_ctrl.rc.s[1]);
    rcInfo.sw[2] = rawToSwitch((uint32_t)rc_ctrl.rc.optional[0]);
    rcInfo.sw[3] = rawToSwitch((uint32_t)rc_ctrl.rc.optional[1]);
    rcInfo.sLeft  = rcInfo.sw[0];   // 兼容旧代码
    rcInfo.sRight = rcInfo.sw[1];
    rcInfo.optional[0] = rc_ctrl.rc.optional[0];
    rcInfo.optional[1] = rc_ctrl.rc.optional[1];

    // 调试：记录每个通道的历史最小/最大值（用于识别开关实际通道）
    for (uint8_t dbgI = 0; dbgI < 16; ++dbgI) {
        if (debugCh[dbgI] < debugChMin[dbgI]) debugChMin[dbgI] = debugCh[dbgI];
        if (debugCh[dbgI] > debugChMax[dbgI]) debugChMax[dbgI] = debugCh[dbgI];
    }

    lastFrameTick = HAL_GetTick();   // 记录收到有效帧的时间，调试用
}
void RemoteControl::ITHandle() {
    if (huart3.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    } else if (USART3->SR & UART_FLAG_IDLE) {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET) {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;

            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if (sbus_rx_buf[0][23]  == 0) {
                HAL_IWDG_Refresh(&hiwdg);
            }
            if (this_time_rx_len == RC_FRAME_LENGTH) {
                RemoteControl::sbus_to_rc(RemoteControl::sbus_rx_buf[0]);
            }
        } else {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if (sbus_rx_buf[1][23]  == 0) {
                HAL_IWDG_Refresh(&hiwdg);
            }
            if (this_time_rx_len == RC_FRAME_LENGTH) {
                //处理遥控器数据
                RemoteControl::sbus_to_rc(RemoteControl::sbus_rx_buf[1]);
            }
        }
    }
}
/**
  * @brief This function handles USART3 global interrupt.
  */

void USART3_IRQHandler(void)
{
    RemoteControl::ITHandle();
}