#include "bsp_key.h"

/* 按键事件编号, 0 = 无事件 */
static uint8_t Key_Num = 0;

/**
 * @brief  获取按键事件并清除 (消费式读取)
 * @retval 按键编号: KEY_xxx_SHORT / KEY_xxx_LONG, 0 表示无事件
 */
uint8_t Key_GetNum(void)
{
    uint8_t temp;

    if (Key_Num != 0)
    {
        temp = Key_Num;
        Key_Num = 0;
        return temp;
    }
    return 0;
}

/**
 * @brief  读取当前哪个按键被按下 (低电平有效)
 * @retval 0 = 无按下, 1 = KEY1, 2 = KEY2
 */
static uint8_t Key_GetState(void)
{
    if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
        return 1;
    if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET)
        return 2;
    return 0;
}

/* ===================== 按键扫描 (含长按检测) ===================== */

/**
 * @brief  按键周期扫描, 需在定时器中断或主循环中以固定周期调用
 * @note   调用周期建议 10ms, 长按阈值由 KEY_LONG_PRESS_COUNT 决定
 *
 *         检测逻辑:
 *         - 短按: 按键释放时, 按住时间 < 长按阈值 → 产生短按事件
 *         - 长按: 按住时间 >= 长按阈值 → 立即产生长按事件 (仅触发一次)
 *
 *         事件值定义 (见 bsp_key.h):
 *           KEY1_SHORT = 1,  KEY1_LONG = 2
 *           KEY2_SHORT = 3,  KEY2_LONG = 4
 */
void Key_Tick(void)
{
    static uint8_t  count     = 0;    /* 20 次计数器, 控制 10ms×20=200ms 扫描节拍 */
    static uint8_t  currState = 0;    /* 当前按键物理状态 */
    static uint8_t  prevState = 0;    /* 上一次按键物理状态 */
    static uint16_t holdTime  = 0;    /* 按键持续按下的计数值 */
    static uint8_t  longFired = 0;    /* 长按事件是否已经触发 (防止重复触发) */

    count++;
    if (count < 20)
        return;
    count = 0;

    /* 记录状态变化 */
    prevState = currState;
    currState = Key_GetState();

    /* ---- 按键正在按下 ---- */
    if (currState != 0)
    {
        holdTime++;

        /* 长按判定: 持续按住时间达到阈值, 且尚未触发过长按事件 */
        if (holdTime >= KEY_LONG_PRESS_COUNT && longFired == 0)
        {
            longFired = 1;
            Key_Num   = (currState - 1) * 2 + 2;   /* 偶数编号 = 长按 */
        }
    }
    /* ---- 按键已释放 (currState == 0) ---- */
    else
    {
        /* 之前有按下且未触发长按 → 判定为短按 */
        if (prevState != 0 && longFired == 0)
        {
            Key_Num = (prevState - 1) * 2 + 1;     /* 奇数编号 = 短按 */
        }

        /* 释放后复位计数, 为下次按键做准备 */
        holdTime  = 0;
        longFired = 0;
    }
}
