/**
 ******************************************************************************
 * @file    RC_SBUS.c
 * @brief   SBUS 遥控器协议解析模块 (Futaba/FrSky SBUS 协议)
 * @note
 *          SBUS 帧结构 (25 字节):
 *          ┌──────┬─────────────────────┬──────┬──────┐
 *          │Byte 0│  Byte 1 ~ 22       │Byte23│Byte24│
 *          │0x0F  │  16ch × 11bit 数据 │标志位│ 0x00 │
 *          └──────┴─────────────────────┴──────┴──────┘
 *
 *          通道数据位布局 (176 bit → 22 byte):
 *          CH1  = buf[1] << 0  | buf[2] << 8  (低11位)
 *          CH2  = buf[2] << 3  | buf[3] << 5  (低11位)
 *          ...
 *
 *          标志位 (Byte 23):
 *          bit0 = CH17 (数字通道)
 *          bit1 = CH18 (数字通道)
 *          bit2 = 帧丢失 (Frame Lost)
 *          bit3 = 失控保护激活 (Failsafe Active)
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "RC_SBUS.h"

/* Private variables ---------------------------------------------------------*/

/**
 * @brief 全局 SBUS 通道数据
 * @note  外部通过 extern 声明后即可访问
 */
SBUS_CH_Struct SBUS_CH = {0};

/**
 * @brief 上次接收到有效 SBUS 帧的时间戳 (ms)
 * @note  由系统提供基础运行计数, 用于超时判断
 */
uint32_t SBUS_LastReceiveTick = 0;

/* Private function declarations ---------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  解析一帧 SBUS 数据 (25 字节)
 * @param  buf  指向 SBUS 原始数据帧的指针 (必须 ≥ 25 字节)
 * @retval 1    帧校验通过, 通道数据已更新
 * @retval 0    帧校验失败 (帧头/帧尾错误, 或信号丢失/失控保护)
 *
 * @note   通道值范围: 0 ~ 2047 (11-bit)
 *         中位值通常为 1000, 实际范围约 300 ~ 1700
 *
 *         数据解包方式:
 *         11 位 × 16 通道 = 176 bit 按小端序紧密排列在 buf[1]~buf[22] 中
 *
 *         信号状态由 buf[23] 标志位判断:
 *         - buf[23] == 0x00 → 正常连接
 *         - buf[23] & 0x0C  ≠ 0 → 丢帧或失控保护
 */
uint8_t SBUS_Update(const uint8_t *buf)
{
    /* ── 帧头帧尾校验 ─────────────────────────────────── */
    /* SBUS 帧必须以 0x0F 开头、0x00 结尾                  */
    if ((buf[0]  != SBUS_START_BYTE) ||
        (buf[24] != SBUS_END_BYTE))
    {
        SBUS_CH.ConnectState = 0;
        return 0;
    }

    /* ── 信号状态检查 ─────────────────────────────────── */
    /* buf[23] 标志位说明:                                 */
    /*   bit2 = Frame Lost  (丢帧)                         */
    /*   bit3 = Failsafe    (失控保护)                     */
    /*   正常时 buf[23] == 0x00                           */
    if (buf[23] != 0)
    {
        SBUS_CH.ConnectState = 0;
        return 0;
    }

    /* ── 更新连接状态 ──────────────────────────────────── */
    SBUS_CH.ConnectState = 1;

    /* ── 解析 16 通道数据 ──────────────────────────────── */
    /* 16 个 11-bit 通道值紧密排列在 buf[1] ~ buf[22]       */
    /* 每个通道取低 11 位 (& 0x07FF) 得到 0~2047 的值        */

    SBUS_CH.CH1  = ((int16_t)buf[ 1] >> 0 | ((int16_t)buf[ 2] << 8))                     & 0x07FF;
    SBUS_CH.CH2  = ((int16_t)buf[ 2] >> 3 | ((int16_t)buf[ 3] << 5))                     & 0x07FF;
    SBUS_CH.CH3  = ((int16_t)buf[ 3] >> 6 | ((int16_t)buf[ 4] << 2) | (int16_t)buf[ 5] << 10) & 0x07FF;
    SBUS_CH.CH4  = ((int16_t)buf[ 5] >> 1 | ((int16_t)buf[ 6] << 7))                     & 0x07FF;

    SBUS_CH.CH5  = ((int16_t)buf[ 6] >> 4 | ((int16_t)buf[ 7] << 4))                     & 0x07FF;
    SBUS_CH.CH6  = ((int16_t)buf[ 7] >> 7 | ((int16_t)buf[ 8] << 1) | (int16_t)buf[ 9] << 9)  & 0x07FF;
    SBUS_CH.CH7  = ((int16_t)buf[ 9] >> 2 | ((int16_t)buf[10] << 6))                     & 0x07FF;
    SBUS_CH.CH8  = ((int16_t)buf[10] >> 5 | ((int16_t)buf[11] << 3))                     & 0x07FF;

    SBUS_CH.CH9  = ((int16_t)buf[12] << 0 | ((int16_t)buf[13] << 8))                     & 0x07FF;
    SBUS_CH.CH10 = ((int16_t)buf[13] >> 3 | ((int16_t)buf[14] << 5))                     & 0x07FF;
    SBUS_CH.CH11 = ((int16_t)buf[14] >> 6 | ((int16_t)buf[15] << 2) | (int16_t)buf[16] << 10) & 0x07FF;
    SBUS_CH.CH12 = ((int16_t)buf[16] >> 1 | ((int16_t)buf[17] << 7))                     & 0x07FF;

    SBUS_CH.CH13 = ((int16_t)buf[17] >> 4 | ((int16_t)buf[18] << 4))                     & 0x07FF;
    SBUS_CH.CH14 = ((int16_t)buf[18] >> 7 | ((int16_t)buf[19] << 1) | (int16_t)buf[20] << 9)  & 0x07FF;
    SBUS_CH.CH15 = ((int16_t)buf[20] >> 2 | ((int16_t)buf[21] << 6))                     & 0x07FF;
    SBUS_CH.CH16 = ((int16_t)buf[21] >> 5 | ((int16_t)buf[22] << 3))                     & 0x07FF;

    return 1;
}

/**
 * @brief  将 SBUS 通道原始值 (0~2047) 转换为 PWM 脉宽值 (1000~2000 us)
 * @param  sbus_value  SBUS 通道原始值
 * @retval PWM 脉宽值 (us)
 *
 * @note   映射公式:
 *         PWM = 1000 + (sbus_value - 300) × (1000 / 1400)
 *
 *         典型映射:
 *         - sbus_value = 300  → PWM = 1000 us (最小)
 *         - sbus_value = 1000 → PWM = 1500 us (中位)
 *         - sbus_value = 1700 → PWM = 2000 us (最大)
 */
uint16_t SBUS_ToPWM(uint16_t sbus_value)
{
    float pwm;

    /* 将 SBUS 值线性映射到 PWM 范围 [1000, 2000] us */
    pwm = SBUS_TARGET_MIN + (float)(sbus_value - SBUS_RANGE_MIN) * SBUS_SCALE_FACTOR;

    /* 限幅保护 */
    if (pwm > (float)SBUS_PWM_MAX)  pwm = (float)SBUS_PWM_MAX;
    if (pwm < (float)SBUS_PWM_MIN)  pwm = (float)SBUS_PWM_MIN;

    return (uint16_t)pwm;
}

/**
 * @brief  将 SBUS 通道值映射到自定义范围
 * @param  sbus_value  SBUS 通道原始值 (0~2047)
 * @param  out_min     输出范围最小值
 * @param  out_max     输出范围最大值
 * @retval 映射后的浮点值
 *
 * @note   可用于将 SBUS 通道映射到:
 *         - 速度指令 (-1.0 ~ +1.0)
 *         - 角度指令 (-90° ~ +90°)
 *         - 任意自定义范围
 */
float SBUS_MapRange(uint16_t sbus_value, float out_min, float out_max)
{
    float ratio;

    /* 将原始值归一化到 [0, 1] */
    ratio = (float)(sbus_value - SBUS_RANGE_MIN) /
            (float)(SBUS_RANGE_MAX - SBUS_RANGE_MIN);

    /* 限幅 [0, 1] */
    if (ratio > 1.0f)  ratio = 1.0f;
    if (ratio < 0.0f)  ratio = 0.0f;

    /* 映射到目标范围 */
    return out_min + ratio * (out_max - out_min);
}

/**
 * @brief  基于超时判断遥控器是否在线
 * @param  timeout_ms  超时阈值 (ms)
 * @retval 1           在线 (最近收到过有效帧)
 * @retval 0           离线 (超过 timeout_ms 未收到有效帧)
 *
 * @note   需配合 HAL_GetTick() 使用
 *         调用示例: if (SBUS_IsOnline(500)) { ... }
 */
uint8_t SBUS_IsOnline(uint32_t timeout_ms)
{
    uint32_t now = HAL_GetTick();

    /* 检查最后一次收到有效帧的时间是否在超时范围内 */
    if ((now - SBUS_LastReceiveTick) < timeout_ms)
    {
        return 1;
    }

    /* 超时, 标记为断连 */
    SBUS_CH.ConnectState = 0;
    return 0;
}


/*
// 方式一: 直接读原始 SBUS 值 (范围 0~2047, 中位约 1000)
uint16_t throttle_raw = SBUS_CH.CH3;   // 油门
uint16_t roll_raw     = SBUS_CH.CH1;   // 副翼
uint16_t pitch_raw    = SBUS_CH.CH2;   // 升降
uint16_t yaw_raw      = SBUS_CH.CH4;   // 方向

// 判断遥控器是否在线
if (SBUS_CH.ConnectState)
{
    // 遥控器在线, 使用通道数据
}

// 方式二: 转为 PWM 脉宽值 (1000~2000 us)
uint16_t pwm = SBUS_ToPWM(SBUS_CH.CH3);  // 油门变成 1000~2000 us

// 方式三: 映射到自定义范围 (例如 -1.0 ~ +1.0 做速度环)
float speed_cmd = SBUS_MapRange(SBUS_CH.CH1, -1.0f, 1.0f);

// 方式四: 超时判断在线
if (SBUS_IsOnline(500))  // 500ms 内收到过有效帧
{
    // 在线
}

*/

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
