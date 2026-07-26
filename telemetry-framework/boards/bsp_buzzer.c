#include "bsp_buzzer.h"

/**
 * @brief  打开蜂鸣器
 * @note   蜂鸣器为高电平有效: SET = 响, RESET = 静音
 */
void Buzzer_on(void)
{
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
}

/**
 * @brief  关闭蜂鸣器
 */
void Buzzer_off(void)
{
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
}
