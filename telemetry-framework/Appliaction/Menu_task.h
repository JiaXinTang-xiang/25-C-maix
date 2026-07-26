/**
 ******************************************************************************
 * @file    Menu_task.h
 * @brief   MaixCam 测量菜单任务接口
 ******************************************************************************
 */

#ifndef MENU_TASK_H
#define MENU_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "bsp.h"

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  初始化测量菜单
 */
void Menu_Task_Init(void);

/**
 * @brief  处理按键、测量状态和 OLED 显示
 * @note   在主循环中周期调用
 */
void Menu_Task_Process(void);

#endif /* MENU_TASK_H */

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
