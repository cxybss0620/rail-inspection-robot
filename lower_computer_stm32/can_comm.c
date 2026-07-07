/**
 * can_comm.c - CAN通信实现
 *
 * 协议设计:
 *   下行命令 (CAN_ID_MOTOR_BASE=0x100):
 *     Byte0: 制动等级 0-3
 *     Byte1-2: 目标速度 (×100, int16 big-endian)
 *     Byte3-4: 目标转角 (×100, int16 big-endian)
 *     Byte5: 舵机pan (×2, 0-180)
 *     Byte6: 舵机tilt (×2, 0-90)
 *     Byte7: 校验和 (前7字节异或)
 *
 *   上行数据 (CAN_ID_SENSOR_BASE=0x200):
 *     分片发送，每片6字节载荷
 *     Byte0: 类型(0x01=IMU, 0x02=超声, 0x03=电池, 0x04=姿态)
 *     Byte1: 分片序号 0-7
 *     Byte2-7: 数据载荷
 */

#include "can_comm.h"
#include <string.h>

/* ── 内部状态 ── */
static MotionCmd_t last_cmd = {0};
static uint8_t sensor_seq = 0;

/* ============================================================
 * CAN_Init
 * ============================================================ */
void CAN_Init(void)
{
    /*
    CAN_FilterTypeDef filter;
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0x0000;   // 接收全部ID
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = CAN_FILTER_ENABLE;
    HAL_CAN_ConfigFilter(&hcan1, &filter);

    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    */
}

/* ============================================================
 * CAN_SendMotorCmd - 发送电机控制指令
 * ============================================================ */
void CAN_SendMotorCmd(const MotionCmd_t* cmd)
{
    uint8_t data[8];
    CAN_TxHeaderTypeDef header;

    /* 编码 */
    data[0] = cmd->brake_level;

    int16_t speed_x100 = (int16_t)(cmd->target_speed * 100.0f);
    data[1] = (speed_x100 >> 8) & 0xFF;
    data[2] = speed_x100 & 0xFF;

    int16_t steer_x100 = (int16_t)(cmd->target_steer * 100.0f);
    data[3] = (steer_x100 >> 8) & 0xFF;
    data[4] = steer_x100 & 0xFF;

    data[5] = (uint8_t)(cmd->servo_pan * 2);    /* 0-360→0-180 */
    data[6] = (uint8_t)(cmd->servo_tilt * 2);

    /* 校验和 */
    data[7] = data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5] ^ data[6];

    /* CAN发送 */
    /*
    header.StdId = 0;                          // 不使用标准ID
    header.ExtId = CAN_ID_MOTOR_BASE;          // 29位扩展ID
    header.IDE = CAN_ID_EXT;
    header.RTR = CAN_RTR_DATA;
    header.DLC = CAN_DLC_8;
    header.TransmitGlobalTime = DISABLE;

    uint32_t mailbox;
    HAL_CAN_AddTxMessage(&hcan1, &header, data, &mailbox);
    */
}

/* ============================================================
 * CAN_SendSensorFrame - 发送传感器数据帧 (分片发送)
 *
 * 实际代码会:
 *   - 填充CAN_DataFrame_t结构
 *   - 逐分片发送 (每片6字节载荷)
 *   - 使用TX mailbox轮询/中断
 * ============================================================ */
void CAN_SendSensorFrame(const SensorFrame_t* frame)
{
    uint8_t data[8];
    CAN_TxHeaderDef header;
    header.ExtId = CAN_ID_SENSOR_BASE;
    header.IDE = CAN_ID_EXT;
    header.RTR = CAN_RTR_DATA;

    /* 分片0: IMU数据 */
    {
        data[0] = 0x01;  /* 类型: IMU */
        data[1] = sensor_seq++;
        /* 加速度 (float→half或×100取整) */
        int16_t ax = (int16_t)(frame->imu.accel_x * 1000);
        int16_t ay = (int16_t)(frame->imu.accel_y * 1000);
        int16_t az = (int16_t)(frame->imu.accel_z * 1000);
        data[2] = (ax >> 8) & 0xFF;
        data[3] = ax & 0xFF;
        data[4] = (ay >> 8) & 0xFF;
        data[5] = ay & 0xFF;
        data[6] = (az >> 8) & 0xFF;
        data[7] = az & 0xFF;
        header.DLC = 7;  /* 7字节有效 */
        /* 发送 */
    }

    /* 分片1: 超声波 */
    {
        data[0] = 0x02;
        data[1] = sensor_seq++;
        for (int i = 0; i < 6; i++) {
            uint16_t dist = (uint16_t)(frame->ultrasonic.distances[i] / 10);
            data[2 + i] = (dist > 255) ? 255 : (uint8_t)dist;
        }
        /* 发送 */
    }

    /* 分片2: 电池 + 温度 */
    {
        data[0] = 0x03;
        data[1] = sensor_seq++;
        uint16_t voltage = (uint16_t)(frame->battery.voltage * 100);
        data[2] = (voltage >> 8) & 0xFF;
        data[3] = voltage & 0xFF;
        data[4] = frame->battery.soc_percent;
        data[5] = (uint8_t)frame->rail_temp;
        data[6] = frame->alert_flags;
        data[7] = 0;
        /* 发送 */
    }
}

/* ============================================================
 * CAN_SendStatus - 发送机器人状态
 * ============================================================ */
void CAN_SendStatus(RobotState_t state, uint8_t flags)
{
    uint8_t data[8] = {0};
    data[0] = (uint8_t)state;
    data[1] = flags;
    /* 填充更多状态信息... */

    /*
    CAN_TxHeaderDef header;
    header.ExtId = CAN_ID_STATUS;
    header.IDE = CAN_ID_EXT;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8;
    // 发送
    */
}

/* ============================================================
 * CAN_SendHeartbeat - 心跳 (每500ms)
 * ============================================================ */
void CAN_SendHeartbeat(void)
{
    uint8_t data[8] = {0};
    data[0] = 0xAA;  /* 心跳魔数 */

    /*
    CAN_TxHeaderDef header;
    header.ExtId = CAN_ID_HEARTBEAT;
    header.IDE = CAN_ID_EXT;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 1;
    // 发送
    */
}

/* ============================================================
 * CAN_RxCallback - CAN接收中断回调
 * 调用位置: HAL_CAN_RxFifo0MsgPendingCallback()
 * ============================================================ */
void CAN_RxCallback(CAN_RxHeaderTypeDef* header, uint8_t* data)
{
    uint32_t id = header->ExtId;

    if (id == CAN_ID_MOTOR_BASE) {
        /* 解析控制指令 */
        CAN_CmdFrame_t cmd;
        cmd.cmd_id = id;
        cmd.param1 = data[0];  /* brake_level */
        cmd.param2 = (int16_t)((data[1] << 8) | data[2]);  /* speed×100 */
        cmd.param3 = (int16_t)((data[3] << 8) | data[4]);  /* steer×100 */

        /* 校验 */
        uint8_t checksum = data[0] ^ data[1] ^ data[2] ^ data[3]
                         ^ data[4] ^ data[5] ^ data[6];
        if (checksum == data[7]) {
            CAN_ParseCommand(&cmd);
        }
    }
    else if (id == CAN_ID_CONFIG) {
        /* 配置命令处理 */
    }
}

/* ============================================================
 * CAN_ParseCommand - 解析命令并执行
 * ============================================================ */
void CAN_ParseCommand(const CAN_CmdFrame_t* cmd)
{
    last_cmd.brake_level   = (uint8_t)cmd->param1;
    last_cmd.target_speed  = cmd->param2 / 100.0f;
    last_cmd.target_steer  = cmd->param3 / 100.0f;

    /* 制动等级处理 */
    if (last_cmd.brake_level >= 3) {
        Motor_EmergencyStop();
        g_robot_state = STATE_EMERGENCY_STOP;
    } else if (last_cmd.brake_level == 2) {
        Motor_SetAllSpeed(CREEP_SPEED_MS * 60.0f / (0.15f * 3.14159f));
    } else if (last_cmd.brake_level == 1) {
        Motor_SetAllSpeed(SLOW_SPEED_MS * 60.0f / (0.15f * 3.14159f));
    } else {
        Motor_SetAllSpeed(last_cmd.target_speed * 60.0f / (0.15f * 3.14159f));
    }
}

/* ============================================================
 * CAN_GetLastCmd - 获取最后收到的命令
 * ============================================================ */
MotionCmd_t CAN_GetLastCmd(void)
{
    return last_cmd;
}

/* ============================================================
 * CAN_ErrorCallback - CAN错误处理
 * ============================================================ */
void CAN_ErrorCallback(void)
{
    /* 检查错误寄存器, 尝试恢复 */
    /*
    uint32_t esr = CAN1->ESR;
    if (esr & CAN_ESR_BOFF) {
        // Bus-Off: 重新初始化CAN控制器
        CAN_Init();
    }
    */
}
