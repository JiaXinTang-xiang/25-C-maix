#ifndef BSP_LED_H
#define BSP_LED_H
#include "main.h"

/* ===================== 通用 LED 结构体 ===================== */

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
} LED_Struct;

void led_on(LED_Struct *led);
void led_off(LED_Struct *led);
void led_toggle(LED_Struct *led);

/* ===================== RGB LED 控制 ===================== */

/**
 * @brief  RGB LED 引脚映射宏
 *         对应 CubeMX 生成的标签:
 *           LED1 → 红色 (R)
 *           LED2 → 绿色 (G)
 *           LED3 → 蓝色 (B)
 *         LED 为低电平有效: RESET = 亮, SET = 灭
 *
 *         修改引脚只需在此处改宏, 无需改动 .c 文件
 */
#define RGB_R_PORT    LED1_GPIO_Port
#define RGB_R_PIN     LED1_Pin
#define RGB_G_PORT    LED2_GPIO_Port
#define RGB_G_PIN     LED2_Pin
#define RGB_B_PORT    LED3_GPIO_Port
#define RGB_B_PIN     LED3_Pin

/**
 * @brief  RGB LED 控制
 * @param  r  红色: 1=亮, 0=灭
 * @param  g  绿色: 1=亮, 0=灭
 * @param  b  蓝色: 1=亮, 0=灭
 * @note   示例:
 *           RGB_Control(1, 0, 0) → 红亮, 绿灭, 蓝灭
 *           RGB_Control(1, 1, 0) → 红绿亮, 蓝灭 (黄)
 *           RGB_Control(0, 0, 0) → 全灭
 */
void RGB_Control(uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief  关闭 RGB LED (全部熄灭)
 */
void RGB_Off(void);

#endif // BSP_LED_H
