#include "Timer_task.h"

/**
 * @brief  HAL 定时器溢出回调函数
 * @note   由 HAL 库在中断中自动调用, 需根据 htim->Instance 判断来源
 *         当前仅使用 TIM14, 周期 1ms / 1000Hz
 * @param  htim  触发中断的定时器句柄
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM14)
    {
        INS_Task();     /* 姿态解算: 每 1ms 执行一次, 更新 roll/pitch/yaw */
        Key_Tick();     /* 按键扫描: 每 1ms 调用, 内部每 20ms 检测一次按键状态 */
    }
}
