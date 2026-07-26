#include "bsp_led.h"

/* ===================== 通用 LED 操作 ===================== */

/**
 * @brief  打开 LED (低电平有效)
 * @param  led  指向 LED_Struct 的指针
 */
void led_on(LED_Struct *led)
{
    HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_RESET);
}

/**
 * @brief  关闭 LED (低电平有效)
 * @param  led  指向 LED_Struct 的指针
 */
void led_off(LED_Struct *led)
{
    HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_SET);
}

/**
 * @brief  翻转 LED 状态
 * @param  led  指向 LED_Struct 的指针
 */
void led_toggle(LED_Struct *led)
{
    HAL_GPIO_TogglePin(led->port, led->pin);
}

/* ===================== RGB LED 控制 ===================== */

/**
 * @brief  RGB LED 控制
 * @param  r  红色: 非0=亮, 0=灭
 * @param  g  绿色: 非0=亮, 0=灭
 * @param  b  蓝色: 非0=亮, 0=灭
 * @note   LED 为低电平有效, 内部自动取反
 *         示例:
 *           RGB_Control(1, 0, 0) → 红
 *           RGB_Control(0, 1, 0) → 绿
 *           RGB_Control(0, 0, 1) → 蓝
 *           RGB_Control(1, 1, 0) → 红+绿 (黄)
 *           RGB_Control(1, 1, 1) → 白
 *           RGB_Control(0, 0, 0) → 全灭
 */
void RGB_Control(uint8_t r, uint8_t g, uint8_t b)
{
    /* 低电平有效: 亮=RESET, 灭=SET */
    HAL_GPIO_WritePin(RGB_R_PORT, RGB_R_PIN, r ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(RGB_G_PORT, RGB_G_PIN, g ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(RGB_B_PORT, RGB_B_PIN, b ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/**
 * @brief  关闭 RGB LED (全部熄灭)
 */
void RGB_Off(void)
{
    RGB_Control(0, 0, 0);
}
