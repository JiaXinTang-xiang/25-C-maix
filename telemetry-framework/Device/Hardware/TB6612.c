#include "TB6612.h"

/* ======================== 方向引脚控制宏 ======================== */
/**
 * @brief  写方向引脚, 直接将0/1强转为GPIO_PinState, 省去三目运算
 * @note   HAL_GPIO_WritePin 的第三个参数类型为 GPIO_PinState:
 *         GPIO_PIN_RESET = 0, GPIO_PIN_SET = 1, 可直接强转
 */
#define AIN1_WRITE(x) HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, (GPIO_PinState)(x))
#define AIN2_WRITE(x) HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, (GPIO_PinState)(x))
#define BIN1_WRITE(x) HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, (GPIO_PinState)(x))
#define BIN2_WRITE(x) HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, (GPIO_PinState)(x))

#define AIN3_WRITE(x) HAL_GPIO_WritePin(AIN3_GPIO_Port, AIN3_Pin, (GPIO_PinState)(x))
#define AIN4_WRITE(x) HAL_GPIO_WritePin(AIN4_GPIO_Port, AIN4_Pin, (GPIO_PinState)(x))
#define BIN3_WRITE(x) HAL_GPIO_WritePin(BIN3_GPIO_Port, BIN3_Pin, (GPIO_PinState)(x))
#define BIN4_WRITE(x) HAL_GPIO_WritePin(BIN4_GPIO_Port, BIN4_Pin, (GPIO_PinState)(x))

/* ======================== 私有变量 ======================== */
static int pwm_max = TB6612_PWM_MAX;   /**< 当前PWM正向限幅值 */
static int pwm_min = TB6612_PWM_MIN;   /**< 当前PWM负向限幅值 */

extern TIM_HandleTypeDef htim8;         /**< TIM8句柄, 由CubeMX生成 */

/* ======================== 私有函数 ======================== */

/**
 * @brief  对单个PWM值进行限幅
 * @param  pwm  原始PWM值
 * @retval 限幅后的PWM值
 */
static int Clamp(int pwm)
{
    if (pwm > pwm_max) return pwm_max;
    if (pwm < pwm_min) return pwm_min;
    return pwm;
}

/**
 * @brief  设置单路电机的方向引脚
 * @param  motor  电机编号
 * @param  dir    方向: 1=正转(IN1=1,IN2=0), -1=反转(IN1=0,IN2=1), 0=滑行(IN1=0,IN2=0)
 */
static void SetDirection(MotorID_t motor, int dir)
{
    switch (motor) {
    case MOTOR_A:
        AIN1_WRITE(dir > 0 ? 1 : 0);
        AIN2_WRITE(dir < 0 ? 1 : 0);
        break;
    case MOTOR_B:
        BIN1_WRITE(dir > 0 ? 1 : 0);
        BIN2_WRITE(dir < 0 ? 1 : 0);
        break;
    case MOTOR_C:
        AIN3_WRITE(dir > 0 ? 1 : 0);
        AIN4_WRITE(dir < 0 ? 1 : 0);
        break;
    case MOTOR_D:
        BIN3_WRITE(dir > 0 ? 1 : 0);
        BIN4_WRITE(dir < 0 ? 1 : 0);
        break;
    default:
        break;
    }
}

/**
 * @brief  设置单路电机的PWM占空比
 * @param  motor    电机编号
 * @param  compare  比较值 (0 ~ pwm_max)
 */
static void SetPWM(MotorID_t motor, uint32_t compare)
{
    uint32_t channel;
    switch (motor) {
    case MOTOR_A: channel = TIM_CHANNEL_1; break;
    case MOTOR_B: channel = TIM_CHANNEL_2; break;
    case MOTOR_C: channel = TIM_CHANNEL_3; break;
    case MOTOR_D: channel = TIM_CHANNEL_4; break;
    default: return;
    }
    __HAL_TIM_SET_COMPARE(&htim8, channel, compare);
}

/* ======================== 公有函数 ======================== */

/**
 * @brief  TB6612初始化函数
 */
void TB6612_Init(void)
{
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);
}

/**
 * @brief  PWM限幅函数
 * @param  motoA  电机A PWM值指针
 * @param  motoB  电机B PWM值指针
 */
void Limit(int *motoA, int *motoB)
{
    if (motoA != NULL) {
        if (*motoA > pwm_max) *motoA = pwm_max;
        if (*motoA < pwm_min) *motoA = pwm_min;
    }
    if (motoB != NULL) {
        if (*motoB > pwm_max) *motoB = pwm_max;
        if (*motoB < pwm_min) *motoB = pwm_min;
    }
}

/**
 * @brief  2电机控制
 * @param  moto1  电机A的PWM值, 正=正转, 负=反转, 0=滑行
 * @param  moto2  电机B的PWM值, 正=正转, 负=反转, 0=滑行
 */
void Load(int moto1, int moto2)
{
    /* 限幅 */
    moto1 = Clamp(moto1);
    moto2 = Clamp(moto2);

    /* 设置方向: >0正转, <0反转, ==0滑行(IN1=0,IN2=0) */
    SetDirection(MOTOR_A, moto1 > 0 ? 1 : (moto1 < 0 ? -1 : 0));
    SetDirection(MOTOR_B, moto2 > 0 ? 1 : (moto2 < 0 ? -1 : 0));

    /* 设置PWM占空比 */
    SetPWM(MOTOR_A, (uint32_t)(moto1 > 0 ? moto1 : -moto1));
    SetPWM(MOTOR_B, (uint32_t)(moto2 > 0 ? moto2 : -moto2));
}

/**
 * @brief  4电机加载
 * @param  moto1  电机A的PWM值
 * @param  moto2  电机B的PWM值
 * @param  moto3  电机C的PWM值
 * @param  moto4  电机D的PWM值
 */
void Load4(int moto1, int moto2, int moto3, int moto4)
{
    int pwms[MOTOR_COUNT] = {moto1, moto2, moto3, moto4};

    for (int i = MOTOR_A; i < MOTOR_COUNT; i++) {
        pwms[i] = Clamp(pwms[i]);
        SetDirection((MotorID_t)i, pwms[i] > 0 ? 1 : (pwms[i] < 0 ? -1 : 0));
        SetPWM((MotorID_t)i, (uint32_t)(pwms[i] > 0 ? pwms[i] : -pwms[i]));
    }
}

/**
 * @brief  单电机设置
 * @param  motor  电机编号 (MOTOR_A ~ MOTOR_D)
 * @param  pwm    PWM值, 正=正转, 负=反转, 0=滑行
 */
void Motor_Set(MotorID_t motor, int pwm)
{
    if (motor >= MOTOR_COUNT) return;

    pwm = Clamp(pwm);
    SetDirection(motor, pwm > 0 ? 1 : (pwm < 0 ? -1 : 0));
    SetPWM(motor, (uint32_t)(pwm > 0 ? pwm : -pwm));
}

/**
 * @brief  单电机刹车 (Short Brake)
 * @param  motor  电机编号
 * @note   TB6612短刹车模式: IN1=1, IN2=1, 电机两端短接, 快速停转.
 *         区别于PWM=0的滑行模式(IN1=0, IN2=0, 电机自由减速).
 */
void Motor_Brake(MotorID_t motor)
{
    switch (motor) {
    case MOTOR_A:
        AIN1_WRITE(1); AIN2_WRITE(1);
        break;
    case MOTOR_B:
        BIN1_WRITE(1); BIN2_WRITE(1);
        break;
    case MOTOR_C:
        AIN3_WRITE(1); AIN4_WRITE(1);
        break;
    case MOTOR_D:
        BIN3_WRITE(1); BIN4_WRITE(1);
        break;
    default:
        break;
    }
    /* 刹车时将PWM占空比也设为0, 确保安全 */
    SetPWM(motor, 0);
}

/**
 * @brief  全部电机刹车
 */
void Motor_BrakeAll(void)
{
    for (int i = MOTOR_A; i < MOTOR_COUNT; i++) {
        Motor_Brake((MotorID_t)i);
    }
}

/**
 * @brief  设置PWM限幅范围
 * @param  max  最大正值 (应 > 0)
 * @param  min  最大负值 (应 < 0)
 */
void Motor_SetLimit(int max, int min)
{
    if (max > 0) pwm_max = max;
    if (min < 0) pwm_min = min;
}
