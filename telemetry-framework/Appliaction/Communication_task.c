/**
 ******************************************************************************
 * @file    Communication_task.c
 * @brief   通信任务回调函数
 *          - USART1 (huart1): MaixCam 任务命令与测量结果
 *          - USART3 (huart3): SBUS 遥控器协议解析
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "communication_task.h"

/* Exported variables --------------------------------------------------------*/
volatile Struct_Vision_Data Vision_Data = {0};

/* Private variables ---------------------------------------------------------*/
static uint8_t Vision_Rx_Frame[VISION_FRAME_SIZE];
static uint8_t Vision_Rx_State = 0;
static uint8_t Vision_Rx_Index = 0;
static uint8_t Vision_Tx_Command = 0;

/* Private function declarations ---------------------------------------------*/
static uint8_t Vision_Is_Valid_Command(uint8_t Command);
static uint16_t Vision_Read_Uint16_BE(const uint8_t *Data);
static void Vision_Parse_Frame(const uint8_t *Frame);
static void Vision_Input_Byte(uint8_t Data);

/**
 * @brief  检查 MaixCam 任务命令是否合法
 */
static uint8_t Vision_Is_Valid_Command(uint8_t Command)
{
    return ((Command >= VISION_COMMAND_BASIC) &&
            (Command <= VISION_COMMAND_ADVANCED_4));
}

/**
 * @brief  从字节数组读取一个大端 uint16_t 数据
 */
static uint16_t Vision_Read_Uint16_BE(const uint8_t *Data)
{
    return ((uint16_t)Data[0] << 8) | (uint16_t)Data[1];
}

/**
 * @brief  解析一帧 MaixCam 测量结果
 * @note   帧格式: AA BB + Distance(2) + Size(2) + Tail(2)
 */
static void Vision_Parse_Frame(const uint8_t *Frame)
{
    uint8_t Result_Type;

    /* 标定完成信号: AA BB 00 01 00 00 CC DD */
    if ((Frame[2] == 0x00U) && (Frame[3] == 0x01U) &&
        (Frame[4] == 0x00U) && (Frame[5] == 0x00U) &&
        (Frame[6] == VISION_FRAME_TAIL_BASIC_1) &&
        (Frame[7] == VISION_FRAME_TAIL_BASIC_2))
    {
        Vision_Data.MaixCam_Ready = 1;
        Vision_Data.Frame_Count++;
        return;
    }

    if ((Frame[6] == VISION_FRAME_TAIL_BASIC_1) &&
        (Frame[7] == VISION_FRAME_TAIL_BASIC_2))
    {
        Result_Type = Vision_Result_BASIC;
    }
    else if ((Frame[6] == VISION_FRAME_TAIL_ADVANCED_1) &&
             (Frame[7] == VISION_FRAME_TAIL_ADVANCED_2))
    {
        Result_Type = Vision_Result_ADVANCED;
    }
    else
    {
        Vision_Data.Error_Count++;
        return;
    }

    if (Vision_Data.State != Vision_State_WAIT_RESULT)
    {
        Vision_Data.Error_Count++;
        return;
    }

    if (((Vision_Data.Last_Command == VISION_COMMAND_BASIC) &&
         (Result_Type != Vision_Result_BASIC)) ||
        ((Vision_Data.Last_Command != VISION_COMMAND_BASIC) &&
         (Result_Type != Vision_Result_ADVANCED)))
    {
        Vision_Data.Error_Count++;
        return;
    }

    Vision_Data.Distance_Cm =
        (float)Vision_Read_Uint16_BE(&Frame[2]) / 100.0f;
    Vision_Data.Size_Cm =
        (float)Vision_Read_Uint16_BE(&Frame[4]) / 100.0f;
    Vision_Data.Result_Type = Result_Type;
    Vision_Data.State = Vision_State_RESULT_READY;
    Vision_Data.Frame_Count++;
}

/**
 * @brief  输入一个串口字节并完成帧同步
 * @note   可处理半帧、粘包以及帧前存在无效数据的情况
 */
static void Vision_Input_Byte(uint8_t Data)
{
    switch (Vision_Rx_State)
    {
        case 0:
            if (Data == VISION_FRAME_HEAD_1)
            {
                Vision_Rx_Frame[0] = Data;
                Vision_Rx_State = 1;
            }
            break;

        case 1:
            if (Data == VISION_FRAME_HEAD_2)
            {
                Vision_Rx_Frame[1] = Data;
                Vision_Rx_Index = 2;
                Vision_Rx_State = 2;
            }
            else if (Data != VISION_FRAME_HEAD_1)
            {
                Vision_Rx_State = 0;
            }
            break;

        case 2:
            Vision_Rx_Frame[Vision_Rx_Index++] = Data;
            if (Vision_Rx_Index >= VISION_FRAME_SIZE)
            {
                Vision_Parse_Frame(Vision_Rx_Frame);
                Vision_Rx_Index = 0;
                Vision_Rx_State = 0;
            }
            break;

        default:
            Vision_Rx_Index = 0;
            Vision_Rx_State = 0;
            break;
    }
}

/**
 * @brief  向 MaixCam 发送一个视觉任务
 */
uint8_t Vision_Send_Command(uint8_t Command)
{
    if (!Vision_Is_Valid_Command(Command))
    {
        return 1;
    }

    if (Vision_Data.State != Vision_State_IDLE)
    {
        return 2;
    }

    Vision_Tx_Command = Command;
    if (UART_Send_Data(&huart1, &Vision_Tx_Command, 1) != HAL_OK)
    {
        return 3;
    }

    Vision_Data.Last_Command = Command;
    Vision_Data.Result_Type = Vision_Result_NONE;
    Vision_Data.Send_Tick = HAL_GetTick();
    Vision_Data.State = Vision_State_WAIT_RESULT;
    return 0;
}

/**
 * @brief  处理 MaixCam 应答超时
 */
void Vision_Communication_Process(void)
{
    if ((Vision_Data.State == Vision_State_WAIT_RESULT) &&
        ((HAL_GetTick() - Vision_Data.Send_Tick) >= VISION_RESPONSE_TIMEOUT_MS))
    {
        Vision_Data.State = Vision_State_TIMEOUT;
        Vision_Data.Timeout_Count++;
    }
}

/**
 * @brief  释放当前结果或超时状态
 */
void Vision_Clear_Result(void)
{
    if ((Vision_Data.State == Vision_State_RESULT_READY) ||
        (Vision_Data.State == Vision_State_TIMEOUT))
    {
        Vision_Data.Result_Type = Vision_Result_NONE;
        Vision_Data.State = Vision_State_IDLE;
    }
}

/**
 * @brief  UART 串口接收 DMA 空闲中断回调 (USART1)
 */
void UART_MaixCam_Call_Back(uint8_t *Buffer, uint16_t Length)
{
    uint16_t i;

    if ((Buffer == 0) || (Length == 0))
    {
        return;
    }

    Vision_Data.Byte_Count += Length;
    for (i = 0; i < Length; i++)
    {
        Vision_Input_Byte(Buffer[i]);
    }
}

/**
 * @brief  SBUS 数据接收回调 (USART3 - DMA 空闲中断)
 * @param  Buffer  接收数据缓冲区
 * @param  Length  本次接收到的数据长度
 */
void SBUS_Data_Call_Back(uint8_t *Buffer, uint16_t Length)
{
    uint16_t i;

    if (Length < SBUS_FRAME_SIZE)
    {
        return;
    }

    for (i = Length - SBUS_FRAME_SIZE; i > 0; i--)
    {
        if ((Buffer[i] == SBUS_START_BYTE) &&
            (Buffer[i + SBUS_FRAME_SIZE - 1] == SBUS_END_BYTE))
        {
            SBUS_Update(&Buffer[i]);
            return;
        }
    }

    if ((Buffer[0] == SBUS_START_BYTE) &&
        (Buffer[SBUS_FRAME_SIZE - 1] == SBUS_END_BYTE))
    {
        SBUS_Update(&Buffer[0]);
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
