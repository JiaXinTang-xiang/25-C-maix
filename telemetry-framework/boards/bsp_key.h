#ifndef BSP_KEY_H
#define BSP_KEY_H
#include "main.h"

/* KEY1(PD7), KEY2(PD6) 已由 CubeMX 在 main.h 中定义 */

/* ===================== 长按阈值 ===================== */
/**
 * @brief  长按判定阈值 (以 Key_Tick 实际检测周期为单位)
 *         Key_Tick 每 1ms 调用, 内部每 20 次执行一次检测 → 实际检测周期 20ms
 *         长按时间 = 20ms × count
 *         默认 50 → 20ms × 50 = 1000ms = 1 秒触发长按
 */
#define KEY_LONG_PRESS_COUNT  50

/* ===================== 按键事件值定义 ===================== */
/* 短按: 奇数; 长按: 偶数; 编号规则: (按键序号-1)*2 + 类型偏移 */
#define KEY1_SHORT  1
#define KEY1_LONG   2
#define KEY2_SHORT  3
#define KEY2_LONG   4

/* ===================== 函数声明 ===================== */
/**
 * @brief  获取按键事件 (消费式读取, 读后清除)
 * @retval 0 = 无事件, KEY_xxx_SHORT / KEY_xxx_LONG = 对应事件
 */
uint8_t Key_GetNum(void);

/**
 * @brief  按键周期扫描, 需以固定周期调用 (建议 10ms)
 * @note   内部每 20 次调用执行一次实际检测 (即 200ms 扫描一次)
 */
void Key_Tick(void);

#endif /* BSP_KEY_H */
