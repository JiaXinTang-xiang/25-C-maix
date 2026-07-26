/**
 * @file    TB6612.h
 * @brief   TB6612FNG 双H桥电机驱动模块头文件
 * @note    硬件连接:
 *          - 电机A方向: AIN1, AIN2
 *          - 电机B方向: BIN1, BIN2
 *          - 电机C方向: AIN3, AIN4 (第二片TB6612)
 *          - 电机D方向: BIN3, BIN4 (第二片TB6612)
 *          - PWM定时器: TIM8 (CH1~CH4 分别对应 A/B/C/D)
 *          - STBY引脚: 需硬件上拉或由GPIO控制
 */

#ifndef TB6612_H
#define TB6612_H

#include "main.h"

/* ======================== PWM 限幅默认值 ======================== */
#define TB6612_PWM_MAX  500    /**< PWM最大值, 对应定时器ARR的占空比上限 */
#define TB6612_PWM_MIN  (-500) /**< PWM最小值, 负值代表反转 */

/* ======================== 电机编号枚举 ======================== */
typedef enum {
    MOTOR_A = 0,   /**< 电机A - TIM8 CH1 */
    MOTOR_B = 1,   /**< 电机B - TIM8 CH2 */
    MOTOR_C = 2,   /**< 电机C - TIM8 CH3 */
    MOTOR_D = 3,   /**< 电机D - TIM8 CH4 */
    MOTOR_COUNT    /**< 电机总数, 用于数组遍历 */
} MotorID_t;

/* ======================== 函数声明 ======================== */

/**
 * @brief  TB6612初始化函数
 */
void TB6612_Init(void);

/**
 * @brief  2电机加载: 根据PWM值设置电机A/B方向和占空比
 * @param  moto1  电机A的PWM值, 正=正转, 负=反转, 0=滑行
 * @param  moto2  电机B的PWM值, 正=正转, 负=反转, 0=滑行
 * @note   内部自动限幅到 [PWM_MIN, PWM_MAX]
 */
void Load(int moto1, int moto2);

/**
 * @brief  4电机加载: 根据PWM值设置电机A/B/C/D方向和占空比
 * @param  moto1  电机A的PWM值
 * @param  moto2  电机B的PWM值
 * @param  moto3  电机C的PWM值
 * @param  moto4  电机D的PWM值
 */
void Load4(int moto1, int moto2, int moto3, int moto4);

/**
 * @brief  单电机设置: 设置指定电机的方向和占空比
 * @param  motor  电机编号 (MOTOR_A ~ MOTOR_D)
 * @param  pwm    PWM值, 正=正转, 负=反转, 0=滑行
 */
void Motor_Set(MotorID_t motor, int pwm);

/**
 * @brief  单电机刹车 (Short Brake)
 * @param  motor  电机编号
 * @note   将IN1和IN2同时置高, TB6612进入短刹车模式
 */
void Motor_Brake(MotorID_t motor);

/**
 * @brief  全部电机刹车
 */
void Motor_BrakeAll(void);

/**
 * @brief  设置PWM限幅范围
 * @param  max  最大正值
 * @param  min  最大负值 (应为负数)
 */
void Motor_SetLimit(int max, int min);

/**
 * @brief  PWM限幅函数
 * @param  motoA  电机A PWM值指针
 * @param  motoB  电机B PWM值指针
 * @note   将值钳位到 [PWM_MIN, PWM_MAX]
 */
void Limit(int *motoA, int *motoB);

#endif /* TB6612_H */
