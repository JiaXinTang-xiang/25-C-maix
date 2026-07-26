/**
 ******************************************************************************
 * @file    RC_SBUS.h
 * @brief   SBUS 遥控器协议解析模块
 * @note    SBUS 协议规格:
 *          - 波特率: 100000, 8E2 (8数据位, 偶校验, 2停止位)
 *          - 帧长度: 25 字节
 *          - 帧格式: [0x0F] [22字节通道数据] [标志位] [0x00]
 *          - 16 通道 × 11 位 = 176 位 / 8 = 22 字节
 *          - 通道值范围: 0 ~ 2047 (11位)
 *          - PWM 映射: SBUS 300~1700 → PWM 1000~2000us
 ******************************************************************************
 */

#ifndef RC_SBUS_H
#define RC_SBUS_H

/* Includes ------------------------------------------------------------------*/
#include "bsp.h"

/* Exported defines ----------------------------------------------------------*/

/* SBUS 协议常量 ------------------------------------------------------------*/
#define SBUS_FRAME_SIZE         25U     /* 一帧 SBUS 数据的字节数               */
#define SBUS_START_BYTE         0x0FU   /* 帧头标识                            */
#define SBUS_END_BYTE           0x00U   /* 帧尾标识                            */
#define SBUS_CHANNEL_COUNT      16U     /* 通道数量                            */

/* SBUS 信号范围 ------------------------------------------------------------*/
#define SBUS_RANGE_MIN          300     /* SBUS 通道最小值 (对应最小行程)        */
#define SBUS_RANGE_MAX          1700    /* SBUS 通道最大值 (对应最大行程)        */
#define SBUS_RANGE_MID          1000    /* SBUS 通道中值                        */

/* PWM 输出范围 -------------------------------------------------------------*/
#define SBUS_PWM_MIN            1000    /* PWM 输出最小值 (us)                  */
#define SBUS_PWM_MAX            2000    /* PWM 输出最大值 (us)                  */

/* 缩放系数: (PWM_MAX - PWM_MIN) / (RANGE_MAX - RANGE_MIN)                   */
#define SBUS_SCALE_FACTOR       0.714285714f  /* 1000 / 1400                    */
#define SBUS_TARGET_MIN         1000.0f

/* 信号丢失判断阈值 ----------------------------------------------------------*/
#define SBUS_SIGNAL_TIMEOUT_MS  500U    /* 超过此时间未收到数据则判为断连        */

/* Exported types ------------------------------------------------------------*/

/**
 * @brief SBUS 遥控器通道数据结构体
 * @note  16 个通道值 (0~2047 原始 SBUS 值) + 连接状态标志
 */
typedef struct
{
    uint16_t CH1;       /* 通道 1  一般: 副翼 (Roll)       */
    uint16_t CH2;       /* 通道 2  一般: 升降 (Pitch)      */
    uint16_t CH3;       /* 通道 3  一般: 油门 (Throttle)   */
    uint16_t CH4;       /* 通道 4  一般: 方向 (Yaw)        */
    uint16_t CH5;       /* 通道 5  一般: 模式切换 / AUX1    */
    uint16_t CH6;       /* 通道 6  一般: AUX2              */
    uint16_t CH7;       /* 通道 7                          */
    uint16_t CH8;       /* 通道 8                          */
    uint16_t CH9;       /* 通道 9                          */
    uint16_t CH10;      /* 通道 10                         */
    uint16_t CH11;      /* 通道 11                         */
    uint16_t CH12;      /* 通道 12                         */
    uint16_t CH13;      /* 通道 13                         */
    uint16_t CH14;      /* 通道 14                         */
    uint16_t CH15;      /* 通道 15                         */
    uint16_t CH16;      /* 通道 16                         */
    uint8_t ConnectState; /* 遥控器与接收机连接状态:
                             0 = 未连接 / 丢帧 / 失控保护
                             1 = 正常连接                        */
} SBUS_CH_Struct;

/* Exported variables --------------------------------------------------------*/

extern SBUS_CH_Struct SBUS_CH;          /* 全局 SBUS 通道数据                 */
extern uint32_t SBUS_LastReceiveTick;   /* 上次收到有效帧的时间戳             */

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  解析 SBUS 原始数据帧，更新通道值
 * @param  buf  指向 25 字节 SBUS 原始数据帧的指针
 * @retval 1    解析成功 (帧头帧尾校验通过)
 * @retval 0    解析失败 (帧校验错误或信号异常)
 * @note   调用后会更新全局 SBUS_CH 结构体和 SBUS_LastReceiveTick
 */
uint8_t SBUS_Update(const uint8_t *buf);

/**
 * @brief  将 SBUS 通道原始值转换为 PWM 脉宽值 (us)
 * @param  sbus_value  SBUS 通道值 (范围 0~2047)
 * @retval PWM 脉宽值 (范围 1000~2000, 单位 us)
 * @note   映射关系: SBUS 300 → 1000us, SBUS 1700 → 2000us
 */
uint16_t SBUS_ToPWM(uint16_t sbus_value);

/**
 * @brief  将 SBUS 通道原始值映射到指定范围
 * @param  sbus_value  SBUS 通道值 (范围 0~2047)
 * @param  out_min     输出最小值
 * @param  out_max     输出最大值
 * @retval 映射后的值
 */
float SBUS_MapRange(uint16_t sbus_value, float out_min, float out_max);

/**
 * @brief  检查遥控器是否在线 (基于超时判断)
 * @param  timeout_ms  超时时间 (ms)，超过此时间未收到帧则判定离线
 * @retval 1           在线
 * @retval 0           离线
 */
uint8_t SBUS_IsOnline(uint32_t timeout_ms);

#endif /* RC_SBUS_H */

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
