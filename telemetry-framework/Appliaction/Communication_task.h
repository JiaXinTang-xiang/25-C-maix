/**
 ******************************************************************************
 * @file    Communication_task.h
 * @brief   通信任务接口声明
 ******************************************************************************
 */

#ifndef COMMUNICATION_TASK_H
#define COMMUNICATION_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "bsp.h"

/* MaixCam protocol definitions ---------------------------------------------*/
#define VISION_FRAME_HEAD_1              0xAAU
#define VISION_FRAME_HEAD_2              0xBBU
#define VISION_FRAME_TAIL_BASIC_1        0xCCU
#define VISION_FRAME_TAIL_BASIC_2        0xDDU
#define VISION_FRAME_TAIL_ADVANCED_1     0xEEU
#define VISION_FRAME_TAIL_ADVANCED_2     0xFFU
#define VISION_FRAME_SIZE                8U

#define VISION_COMMAND_BASIC             0x01U
#define VISION_COMMAND_ADVANCED_1        0x02U
#define VISION_COMMAND_ADVANCED_2        0x03U
#define VISION_COMMAND_ADVANCED_3        0x04U
#define VISION_COMMAND_ADVANCED_4        0x05U

#define VISION_RESPONSE_TIMEOUT_MS       3000U

/* Exported types ------------------------------------------------------------*/
enum Enum_Vision_State
{
    Vision_State_IDLE = 0,
    Vision_State_WAIT_RESULT,
    Vision_State_RESULT_READY,
    Vision_State_TIMEOUT,
};

enum Enum_Vision_Result_Type
{
    Vision_Result_NONE = 0,
    Vision_Result_BASIC,
    Vision_Result_ADVANCED,
};

typedef struct
{
    float Distance_Cm;
    float Size_Cm;

    uint32_t Send_Tick;
    uint32_t Byte_Count;
    uint32_t Frame_Count;
    uint32_t Error_Count;
    uint32_t Timeout_Count;

    uint8_t Last_Command;
    uint8_t State;
    uint8_t Result_Type;
    uint8_t MaixCam_Ready;
} Struct_Vision_Data;

/* Exported variables --------------------------------------------------------*/
extern volatile Struct_Vision_Data Vision_Data;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  向 MaixCam 发送一个视觉任务
 * @param  Command  VISION_COMMAND_BASIC 或 VISION_COMMAND_ADVANCED_x
 * @return 0=发送成功, 其它值=参数错误、通信忙或发送失败
 */
uint8_t Vision_Send_Command(uint8_t Command);

/**
 * @brief  处理 MaixCam 应答超时
 * @note   在主循环中周期调用
 */
void Vision_Communication_Process(void);

/**
 * @brief  释放当前结果或超时状态，允许发送下一条命令
 */
void Vision_Clear_Result(void);

/**
 * @brief  USART1 MaixCam 接收回调
 * @param  Buffer  接收数据缓冲区
 * @param  Length  接收数据长度
 */
void UART_MaixCam_Call_Back(uint8_t *Buffer, uint16_t Length);

/**
 * @brief  USART3 SBUS 接收回调 (DMA 空闲中断)
 * @param  Buffer  接收数据缓冲区
 * @param  Length  接收数据长度 (通常为 25 字节 SBUS 帧)
 * @note   自动搜索帧同步, 调用 SBUS_Update() 解析通道数据
 */
void SBUS_Data_Call_Back(uint8_t *Buffer, uint16_t Length);

#endif /* COMMUNICATION_TASK_H */

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
