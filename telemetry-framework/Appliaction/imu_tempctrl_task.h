//
// Created by Admin on 2025/12/19.
//

#ifndef IMU_TEMPCTRL_TASK_H
#define IMU_TEMPCTRL_TASK_H

#include "main.h"
#include "PID.h"
#include "gpio.h"
#include "BMI088driver.h"
#include "MahonyAHRS.h"

extern uint8_t attitude_flag;
extern float gyro[3], accel[3], temp; //陀螺仪原始值
void INS_init(void);
void INS_Task(void);  //1khz

#endif //CLION_IMU_IMU_TEMP_CTRL_H

