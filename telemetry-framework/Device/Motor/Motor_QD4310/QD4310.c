//
// Created by jason on 25-10-16.
//
#include "main.h"
#include "QD4310.h"

// 限制函数，用于替代C++的std::clamp
static float QD4310_Clamp(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// 初始化QD4310电机结构体
void QD4310_Init(QD4310_t *motor, CAN_HandleTypeDef *hcan, uint8_t id) {
    motor->hcan = hcan;
    motor->id = id;
    motor->enabled = false;
    motor->speed = 0.0f;
    motor->angle = 0.0f;
    motor->current = 0.0f;
}


// 发送命令到电机
void QD4310_SendCommand(QD4310_t *motor, QD4310_Command_t cmd, int16_t value) {
    static uint8_t TxBuffer[3];
    TxBuffer[0] = (uint8_t)cmd;
    // 将int16_t值拆分为两个字节
    TxBuffer[1] = (uint8_t)(value & 0xFF);
    TxBuffer[2] = (uint8_t)((value >> 8) & 0xFF);

    uint32_t txMailbox = CAN_TX_MAILBOX0;
    CAN_TxHeaderTypeDef TxHeader;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.StdId = 0x400 + motor->id;
    TxHeader.ExtId = 0x400 + motor->id;
    TxHeader.TransmitGlobalTime = DISABLE;
    TxHeader.DLC = 3;
    HAL_CAN_AddTxMessage(motor->hcan, &TxHeader, TxBuffer, &txMailbox);
    while (HAL_CAN_GetTxMailboxesFreeLevel(motor->hcan) == 0);
}


// 更新电机状态
void QD4310_Update(QD4310_t *motor, const uint8_t feedback[8]) {
    motor->enabled = feedback[0] & 0x01;

    // 重构int16_t值从字节数组
    int16_t current_raw = (int16_t)((feedback[3] << 8) | feedback[2]);
    motor->current = (float)current_raw * 10.0f / 32767.0f; // INT16_MAX = 32767

    int16_t speed_raw = (int16_t)((feedback[5] << 8) | feedback[4]);
    motor->speed = (float)speed_raw * 1000.0f / 32767.0f;

    uint16_t angle_raw = (uint16_t)((feedback[7] << 8) | feedback[6]);
    motor->angle = (float)angle_raw * QD4310_TWO_PI / 65535.0f; // UINT16_MAX = 65535
}


// 使能电机
void QD4310_Enable(QD4310_t *motor) {
    QD4310_SendCommand(motor, QD4310_CMD_ENABLE, 0x0000);
}

// 失能电机
void QD4310_Disable(QD4310_t *motor) {
    QD4310_SendCommand(motor, QD4310_CMD_DISABLE, 0x0000);
}

// 设置电机角度
void QD4310_SetAngle(QD4310_t *motor, float angle) {
    angle = QD4310_Clamp(angle, 0.0f, QD4310_TWO_PI);
    int16_t angle_value = (int16_t)(angle / QD4310_TWO_PI * 65535.0f);
    QD4310_SendCommand(motor, QD4310_CMD_ANGLE, angle_value);
}

// 设置电机转速
void QD4310_SetSpeed(QD4310_t *motor, float speed) {
    speed = QD4310_Clamp(speed, QD4310_MIN_SPEED, QD4310_MAX_SPEED);
    int16_t speed_value = (int16_t)(speed / 500.0f * 32767.0f);
    QD4310_SendCommand(motor, QD4310_CMD_SPEED, speed_value);
}

// 设置电机低速
void QD4310_SetLowSpeed(QD4310_t *motor, float speed) {
    speed = QD4310_Clamp(speed, QD4310_MIN_SPEED, QD4310_MAX_SPEED);
    int16_t speed_value = (int16_t)(speed / 1000.0f * 32767.0f);
    QD4310_SendCommand(motor, QD4310_CMD_LOW_SPEED, speed_value);
}

// 设置电机电流
void QD4310_SetCurrent(QD4310_t *motor, float current) {
    current = QD4310_Clamp(current, QD4310_MIN_CURRENT, QD4310_MAX_CURRENT);
    int16_t current_value = (int16_t)(current / 10.0f * 32767.0f);
    QD4310_SendCommand(motor, QD4310_CMD_CURRENT, current_value);
}
