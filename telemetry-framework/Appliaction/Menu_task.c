/**
 ******************************************************************************
 * @file    Menu_task.c
 * @brief   双按键 MaixCam 测量菜单
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "Menu_task.h"

/* Private macros ------------------------------------------------------------*/
#define MENU_ITEM_COUNT             5U
#define MENU_OLED_REFRESH_MS        100U

/* Private types -------------------------------------------------------------*/
enum Enum_Menu_Page
{
    Menu_Page_WAIT_READY = 0,
    Menu_Page_SELECT,
    Menu_Page_WAIT_RESULT,
    Menu_Page_RESULT,
    Menu_Page_TIMEOUT,
};

struct Struct_Menu_Item
{
    const char *Name;
    uint8_t Command;
};

/* Private variables ---------------------------------------------------------*/
static const struct Struct_Menu_Item Menu_Items[MENU_ITEM_COUNT] =
{
    {"BASIC",     VISION_COMMAND_BASIC},
    {"SEPARATED", VISION_COMMAND_ADVANCED_1},
    {"OVERLAP",   VISION_COMMAND_ADVANCED_2},
    {"DIGIT",     VISION_COMMAND_ADVANCED_3},
    {"ROTATION",  VISION_COMMAND_ADVANCED_4},
};

static uint8_t Menu_Page = Menu_Page_WAIT_READY;
static uint8_t Menu_Selected_Index = 0;
static uint32_t Menu_Last_Refresh_Tick = 0;

/* Private function declarations ---------------------------------------------*/
static void Menu_Draw_Line(uint8_t Row, const char *Format, ...);
static void Menu_Draw_Page(void);
static void Menu_Start_Selected_Task(void);

/**
 * @brief  覆盖显示一整行，避免短字符串留下旧字符
 */
static void Menu_Draw_Line(uint8_t Row, const char *Format, ...)
{
    char Buffer[22];
    va_list Args;
    uint8_t i;

    for (i = 0; i < 21; i++)
    {
        Buffer[i] = ' ';
    }
    Buffer[21] = '\0';

    va_start(Args, Format);
    vsnprintf(Buffer, sizeof(Buffer), Format, Args);
    va_end(Args);

    OLED_printf(Row, 0, "%-20s", Buffer);
}

/**
 * @brief  绘制当前页面
 */
static void Menu_Draw_Page(void)
{
    uint8_t i;

    OLED_operate_gram(PEN_CLEAR);

    switch (Menu_Page)
    {
        case Menu_Page_WAIT_READY:
            Menu_Draw_Line(0, "WAIT MAIXCAM");
            Menu_Draw_Line(1, "CALIBRATING...");
            Menu_Draw_Line(3, "K1/K2 DISABLED");
            break;

        case Menu_Page_SELECT:
            for (i = 0; i < MENU_ITEM_COUNT; i++)
            {
                Menu_Draw_Line(i, "%c%s",
                               (i == Menu_Selected_Index) ? '>' : ' ',
                               Menu_Items[i].Name);
            }
            break;

        case Menu_Page_WAIT_RESULT:
            Menu_Draw_Line(0, "TASK:%s", Menu_Items[Menu_Selected_Index].Name);
            Menu_Draw_Line(1, "MEASURING...");
            Menu_Draw_Line(2, "TIME:%lu.%lus",
                           (unsigned long)((HAL_GetTick() - Vision_Data.Send_Tick) / 1000U),
                           (unsigned long)(((HAL_GetTick() - Vision_Data.Send_Tick) % 1000U) / 100U));
            Menu_Draw_Line(4, "PLEASE WAIT");
            break;

        case Menu_Page_RESULT:
            Menu_Draw_Line(0, "TASK:%s", Menu_Items[Menu_Selected_Index].Name);
            Menu_Draw_Line(1, "D:%.2fcm", Vision_Data.Distance_Cm);
            Menu_Draw_Line(2, "X:%.2fcm", Vision_Data.Size_Cm);
            Menu_Draw_Line(3, "STATUS:OK");
            Menu_Draw_Line(4, "K1:MENU K2:AGAIN");
            break;

        case Menu_Page_TIMEOUT:
            Menu_Draw_Line(0, "TASK:%s", Menu_Items[Menu_Selected_Index].Name);
            Menu_Draw_Line(1, "COMM TIMEOUT");
            Menu_Draw_Line(2, "NO RESULT IN 3S");
            Menu_Draw_Line(4, "K1:MENU K2:RETRY");
            break;

        default:
            Menu_Page = Menu_Page_WAIT_READY;
            break;
    }

    OLED_refresh_gram();
}

/**
 * @brief  执行当前选中的测量任务
 */
static void Menu_Start_Selected_Task(void)
{
    if (Vision_Send_Command(Menu_Items[Menu_Selected_Index].Command) == 0U)
    {
        Menu_Page = Menu_Page_WAIT_RESULT;
    }
}

/**
 * @brief  初始化测量菜单
 */
void Menu_Task_Init(void)
{
    Menu_Page = Menu_Page_WAIT_READY;
    Menu_Selected_Index = 0;
    Menu_Last_Refresh_Tick = HAL_GetTick() - MENU_OLED_REFRESH_MS;
}

/**
 * @brief  处理按键、测量状态和 OLED 显示
 */
void Menu_Task_Process(void)
{
    uint8_t Key = Key_GetNum();
    uint32_t Current_Tick = HAL_GetTick();

    if ((Menu_Page == Menu_Page_WAIT_READY) && Vision_Data.MaixCam_Ready)
    {
        Menu_Page = Menu_Page_SELECT;
    }

    if (Menu_Page == Menu_Page_SELECT)
    {
        if (Key == KEY1_SHORT)
        {
            Menu_Selected_Index++;
            if (Menu_Selected_Index >= MENU_ITEM_COUNT)
            {
                Menu_Selected_Index = 0;
            }
        }
        else if ((Key == KEY2_SHORT) && Vision_Data.MaixCam_Ready)
        {
            Menu_Start_Selected_Task();
        }
    }
    else if (Menu_Page == Menu_Page_WAIT_RESULT)
    {
        /* 测量过程中消费但忽略所有按键事件 */
        if (Vision_Data.State == Vision_State_RESULT_READY)
        {
            Menu_Page = Menu_Page_RESULT;
        }
        else if (Vision_Data.State == Vision_State_TIMEOUT)
        {
            Menu_Page = Menu_Page_TIMEOUT;
        }
    }
    else if ((Menu_Page == Menu_Page_RESULT) ||
             (Menu_Page == Menu_Page_TIMEOUT))
    {
        if (Key == KEY1_SHORT)
        {
            Vision_Clear_Result();
            Menu_Page = Menu_Page_SELECT;
        }
        else if (Key == KEY2_SHORT)
        {
            Vision_Clear_Result();
            Menu_Start_Selected_Task();
        }
    }

    if ((Current_Tick - Menu_Last_Refresh_Tick) >= MENU_OLED_REFRESH_MS)
    {
        Menu_Last_Refresh_Tick = Current_Tick;
        Menu_Draw_Page();
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
