/**
 * can_comm.h - CAN-FD通信模块
 * 硬件: CAN1 (PB8/PB9) 1Mbps, 连接: 电机驱动板 + 上位机(5G UART)
 * 协议: CAN2.0B 扩展帧, 29bit ID
 *
 * 帧ID分配:
 *   ID 0x100~0x1FF: 电机控制指令 (下行)
 *   ID 0x200~0x2FF: 传感器数据上报 (上行)
 *   ID 0x300~0x3FF: 状态 / 告警
 *   ID 0x400~0x4FF: 配置 / 应答
 */

#ifndef CAN_COMM_H
#define CAN_COMM_H

#include "rail_inspect.h"
#include <stdbool.h>

/* ── CAN帧类型 ── */
#define CAN_ID_MOTOR_BASE       0x100   /* 电机控制 */
#define CAN_ID_SENSOR_BASE      0x200   /* 传感器数据 */
#define CAN_ID_STATUS           0x300   /* 机器人状态 */
#define CAN_ID_ALERT            0x301   /* 告警 */
#define CAN_ID_CONFIG           0x400   /* 配置 */
#define CAN_ID_HEARTBEAT        0x401   /* 心跳 */

/* ── CAN数据长度 ── */
#define CAN_DLC_8               LL_CAN_DATAWIDTH_8B

/* ── API ── */

/* 初始化CAN1: PB8(RX)/PB9(TX), 1Mbps */
void CAN_Init(void);

/* 发送电机控制指令 (4电机速度+2舵机) */
void CAN_SendMotorCmd(const MotionCmd_t* cmd);

/* 发送传感器数据帧 (分片: IMU/超声/电池各1帧) */
void CAN_SendSensorFrame(const SensorFrame_t* frame);

/* 发送状态帧 */
void CAN_SendStatus(RobotState_t state, uint8_t flags);

/* 发送心跳 */
void CAN_SendHeartbeat(void);

/* CAN接收回调 (在CAN中断中调用) */
void CAN_RxCallback(CAN_RxHeaderTypeDef* header, uint8_t* data);

/* 解析收到的命令 */
void CAN_ParseCommand(const CAN_CmdFrame_t* cmd);

/* 获取最新收到的控制指令 */
MotionCmd_t CAN_GetLastCmd(void);

/* 错误处理 */
void CAN_ErrorCallback(void);

#endif
