/**
 * @brief BSP 全称: Board Support Package
 * @note 中文释义: 板级支持包，嵌入式行业标准术语
 * @function 
 *  核心作用:
 *  1. 封装开发板所有硬件底层驱动
 *  2. 隔离芯片寄存器、GPIO、外设原始操作
 *  3. 上层业务逻辑无需直接操作寄存器，仅调用BSP对外接口函数
 * @author 小雨
 */

#ifndef BSP_H
#define BSP_H

/******************系统头文件***************/
#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include "stm32f4xx_hal.h"
/*******************************************/
#define u8 unsigned char
#define u32 unsigned int
#define u16  uint16_t
#define fp32 float
#define fp64 double
#ifndef bool_t
typedef uint8_t bool_t;
#endif
/******************用户自定义头文件*********/

/* board（板卡驱动层） */
#include "drv_spi.h"
#include "drv_can.h"
#include "drv_uart.h"
#include "bsp_key.h"
#include "bsp_led.h"
#include "bsp_oled.h"
#include "bsp_buzzer.h"
#include "tool_debug.h"
#include "tool_delay.h"
#include "tool_math.h"
#include "tool_serialplot.h"

/* application（应用层） */
#include "Timer_task.h"
#include "communication_task.h"
#include "Menu_task.h"
#include "imu_tempctrl_task.h"

/* Algorithm（算法层） */
#include "pid.h"
#include "alg_pid.h"

/* Hardware（其它硬件类） */
#include "TB6612.h"
#include "RC_SBUS.h"

/* IMU（陀螺仪类） */
#include "BMI088driver.h"
#include "BMI088reg.h"
#include "BMI088Middleware.h"

/* Motor（电机类） */
#include "QD4310.h"
#include "dvc_motor_dji.h"

/*******************************************/
#endif


