/**
 * motor.c - DJI M3508 底盘电机 + 达妙 DM4310 云台电机驱动
 *
 * ============================================================
 * M3508 通信协议 (CAN 1Mbps, 标准帧 11bit ID)
 * ============================================================
 * 控制帧 (STM32 → M3508):
 *   ID: 0x200 (同时控制4个电机, 每个电机2字节 int16_t)
 *   Data[0-1]: 前左电机电流 (-16384 ~ +16384)
 *   Data[2-3]: 前右电机电流
 *   Data[4-5]: 后左电机电流
 *   Data[6-7]: 后右电机电流
 *
 * 反馈帧 (M3508 → STM32):
 *   ID: 0x201~0x204 对应前左、前右、后左、后右
 *   Data[0-1]: 转子机械角度 0-8191 (uint16)
 *   Data[2-3]: 转子转速 r/min (int16)
 *   Data[4-5]: 扭矩电流    (int16)
 *   Data[6]:   电机温度 °C (uint8)
 *   Data[7]:   保留
 *
 * ============================================================
 * DM4310 通信协议 (CAN 1Mbps)
 * ============================================================
 * 控制帧 (STM32 → DM4310): MIT 模式
 *   ID: 0x1FF
 *   Data[0-1]: PAN 目标位置 (float×100, int16)
 *   Data[2-3]: PAN 目标速度 (float×100, int16)
 *   Data[4-5]: PAN 前馈扭矩 (int16)
 *   Data[6-7]: TILT 目标位置/速度/扭矩 (同上, 分两帧或紧凑编码)
 *
 * 实际简化: PAN 用 ID 0x1FF, TILT 用 ID 0x200 (DM4310 支持多 ID 配置)
 *
 * 反馈帧 (DM4310 → STM32):
 *   ID: 0x1FE (PAN) / 0x1FD (TILT)  (通过 DM4310 配置软件设定)
 *   与 M3508 类似: 角度(2B)/转速(2B)/电流(2B)/温度(1B)
 */

#include "motor.h"
#include <math.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

/* ── 内部 PID 参数 ── */
#define VEL_KP      3.0f        /* 速度环 P */
#define VEL_KI      0.5f        /* 速度环 I */
#define VEL_KD      0.02f       /* 速度环 D */
#define VEL_INTEGRAL_MAX  8000.0f
#define VEL_OUTPUT_MAX     16384.0f  /* M3508 电流上限 */

#define POS_KP      15.0f       /* 云台位置环 P */
#define POS_KD      0.8f        /* 云台速度阻尼 D */

/* ── M3508 参数 ── */
#define WHEEL_RADIUS_M      0.076f   /* 76mm 轮子半径 */
#define WHEEL_BASE_M        0.35f    /* 轴距 */
#define WHEEL_TRACK_M       0.30f    /* 轮距 */

/* ── 全局状态 ── */
static MotorState_t motors[4];
static GimbalState_t gimbals[2];
static uint32_t motor_last_rx[4];    /* 最后收到反馈的 tick */

/* PID 状态 */
static float vel_integral[4] = {0};
static float vel_last_error[4] = {0};
static float pos_integral[2] = {0};   /* 云台位置环积分 */

/* CAN 外设句柄 (extern 来自 HAL 初始化) */
/*
extern CAN_HandleTypeDef hcan1;
static CAN_TxHeaderTypeDef tx_header;
*/

/* ============================================================
 * 辅助: 发送 CAN 帧
 * ============================================================ */
static void CAN_SendFrame(uint32_t ext_id, const uint8_t data[8])
{
    /* 实际 STM32 HAL 代码:
    uint32_t mailbox;
    tx_header.ExtId = ext_id;
    tx_header.IDE = CAN_ID_EXT;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;
    HAL_CAN_AddTxMessage(&hcan1, &tx_header, data, &mailbox);
    */
    (void)ext_id;
    (void)data;
}

/* ============================================================
 * Motor_Init - 初始化 CAN1, 配置滤波器
 * ============================================================ */
void Motor_Init(void)
{
    memset(motors,  0, sizeof(motors));
    memset(gimbals, 0, sizeof(gimbals));
    memset(motor_last_rx, 0, sizeof(motor_last_rx));
    memset(vel_integral,  0, sizeof(vel_integral));
    memset(vel_last_error, 0, sizeof(vel_last_error));

    /* CAN 滤波器: 接收 0x200~0x210, 0x1F0~0x200 范围内的帧 */
    /*
    CAN_FilterTypeDef filter;
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow  = 0x0000;
    filter.FilterMaskIdHigh = 0xFFC0;   // 仅匹配高10bit, 允许0x200~0x2FF
    filter.FilterMaskIdLow  = 0x0000;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = CAN_FILTER_ENABLE;
    HAL_CAN_ConfigFilter(&hcan1, &filter);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    */
}

/* ============================================================
 * PID 速度控制器 (输出电流指令)
 * ============================================================ */
static int16_t PID_Velocity(uint8_t idx, float target, float actual, float dt)
{
    if (dt < 0.0001f) dt = 0.001f;

    float error = target - actual;

    float p = VEL_KP * error;
    vel_integral[idx] += VEL_KI * error * dt;
    if (vel_integral[idx] >  VEL_INTEGRAL_MAX) vel_integral[idx] =  VEL_INTEGRAL_MAX;
    if (vel_integral[idx] < -VEL_INTEGRAL_MAX) vel_integral[idx] = -VEL_INTEGRAL_MAX;
    float d = VEL_KD * (error - vel_last_error[idx]) / dt;
    vel_last_error[idx] = error;

    float out = p + vel_integral[idx] + d;
    if (out >  VEL_OUTPUT_MAX) out =  VEL_OUTPUT_MAX;
    if (out < -VEL_OUTPUT_MAX) out = -VEL_OUTPUT_MAX;

    return (int16_t)out;
}

/* ============================================================
 * Motor_SetCurrent - 直接电流控制
 * ============================================================ */
void Motor_SetCurrent(MotorID_t motor, int16_t current)
{
    if (motor >= 4) return;

    /* 限幅 */
    if (current >  M3508_MAX_CURRENT) current =  M3508_MAX_CURRENT;
    if (current < -M3508_MAX_CURRENT) current = -M3508_MAX_CURRENT;

    motors[motor].target_current = current;

    /* 发送 CAN 控制帧 (0x200 同时控制4个电机) */
    /*
    uint8_t data[8];
    data[0] = (motors[0].target_current >> 8) & 0xFF;
    data[1] =  motors[0].target_current       & 0xFF;
    data[2] = (motors[1].target_current >> 8) & 0xFF;
    data[3] =  motors[1].target_current       & 0xFF;
    data[4] = (motors[2].target_current >> 8) & 0xFF;
    data[5] =  motors[2].target_current       & 0xFF;
    data[6] = (motors[3].target_current >> 8) & 0xFF;
    data[7] =  motors[3].target_current       & 0xFF;
    CAN_SendFrame(0x200, data);
    */
}

/* ============================================================
 * Motor_SetSpeed - RPM → 电流 PID → CAN
 * ============================================================ */
void Motor_SetSpeed(MotorID_t motor, float speed_rpm)
{
    if (motor >= 4 || motors[motor].fault) return;

    /* PID 计算 */
    float dt = 0.01f;
    int16_t current = PID_Velocity(motor, speed_rpm, (float)motors[motor].speed_rpm, dt);
    Motor_SetCurrent(motor, current);
}

/* ============================================================
 * Motor_DiffDrive - 差速驱动
 *
 * 四轮差速运动学:
 *   v_FL = v - ω×(L+H)/2   (左侧)
 *   v_FR = v + ω×(L+H)/2   (右侧)
 *   简化(同轴转向): ω_rad_s = v / r  → 角速度 × 轮半径 = 轮速增量
 *   四轮独立驱动：前轴/后轴分别用阿克曼几何计算
 *
 * 简化 Ackermann: 前轮转角 δ = atan2(L×ω, v)
 *   实际此处直接用四轮独立差速(滑移转向), 不需要舵机
 * ============================================================ */
void Motor_DiffDrive(float v_linear, float v_angular)
{
    /* 限幅 */
    if (v_linear  >  MAX_SPEED_MS)  v_linear  =  MAX_SPEED_MS;
    if (v_linear  < -MAX_SPEED_MS)  v_linear  = -MAX_SPEED_MS;

    /* 四轮差速: 内侧减速, 外侧加速 */
    float half_track = WHEEL_TRACK_M / 2.0f;

    /* 左/右侧线速度 (m/s) */
    float v_left  = v_linear - v_angular * half_track;
    float v_right = v_linear + v_angular * half_track;

    /* 线速度 → 轮转速 (r/min) */
    /* rpm = (v / (2πr)) × 60  =  (v / r) × 60/2π ≈ (v/r) × 9.549 */
    float rpm_left  = (v_left  / WHEEL_RADIUS_M) * 9.549f;
    float rpm_right = (v_right / WHEEL_RADIUS_M) * 9.549f;

    /* 减速比换算: 电机转子 rpm = 轮子 rpm × 减速比 */
    rpm_left  *= M3508_REDUCTION_RATIO;
    rpm_right *= M3508_REDUCTION_RATIO;

    Motor_SetSpeed(MOTOR_FRONT_LEFT,  rpm_left);
    Motor_SetSpeed(MOTOR_REAR_LEFT,   rpm_left);
    Motor_SetSpeed(MOTOR_FRONT_RIGHT, rpm_right);
    Motor_SetSpeed(MOTOR_REAR_RIGHT,  rpm_right);
}

/* ============================================================
 * Motor_EmergencyStop - 紧急制动
 *
 * M3508 CAN 电流置零 → 电机进入自然阻尼状态
 * 如需硬制动: 发送反向小电流 (-3000) 短暂刹车, 然后置零
 * ============================================================ */
void Motor_EmergencyStop(void)
{
    for (int i = 0; i < 4; i++) {
        motors[i].target_current = 0;
        vel_integral[i] = 0;
        vel_last_error[i] = 0;
    }

    /* 发送零电流帧 */
    uint8_t data[8] = {0};
    CAN_SendFrame(0x200, data);

    /* 蜂鸣器 */
    /* HAL_GPIO_WritePin(GPIOC, BUZZER_PIN, SET); */
}

/* ============================================================
 * Motor_CoastStop - 缓停
 * ============================================================ */
void Motor_CoastStop(void)
{
    for (int i = 0; i < 4; i++) {
        if (motors[i].speed_rpm > 100) {
            Motor_SetCurrent((MotorID_t)i, -2000);  /* 反力矩减速 */
        } else if (motors[i].speed_rpm < -100) {
            Motor_SetCurrent((MotorID_t)i, 2000);
        } else {
            Motor_SetCurrent((MotorID_t)i, 0);
        }
        vel_integral[i] = 0;
    }
}

/* ============================================================
 * M3508 反馈解析 (CAN RX 中断中调用)
 *
 * 输入: CAN帧 ID + 8字节数据
 * 根据 ID 确定是哪个电机 (0x201~0x204)
 *
 * 数据格式 (大疆手册):
 *   Data[0] = 转子角度高 8bit
 *   Data[1] = 转子角度低 8bit    (0~8191)
 *   Data[2] = 转速高 8bit
 *   Data[3] = 转速低 8bit        (int16, r/min)
 *   Data[4] = 电流高 8bit
 *   Data[5] = 电流低 8bit        (int16)
 *   Data[6] = 温度 (uint8, °C)
 *   Data[7] = 0
 * ============================================================ */
void M3508_ParseFeedback(uint32_t can_id, const uint8_t data[8])
{
    uint8_t idx = (uint8_t)(can_id - 0x201);  /* 0x201→0, 0x202→1, ... */
    if (idx >= 4) return;

    motors[idx].encoder  = ((uint16_t)data[0] << 8) | data[1];
    motors[idx].speed_rpm = ((int16_t)(((uint16_t)data[2] << 8) | data[3]));
    motors[idx].actual_current = ((int16_t)(((uint16_t)data[4] << 8) | data[5]));
    motors[idx].temperature   = data[6];
    motors[idx].online  = true;
    motors[idx].fault   = (motors[idx].temperature > 100);  /* >100°C 过热 */
    motor_last_rx[idx]  = xTaskGetTickCount();
}

/* ============================================================
 * DM4310 反馈解析
 *
 * DM4310 数据格式与 M3508 类似 (达妙兼容大疆协议)
 * 编码器为多圈绝对值, 范围 0~65535 (/360°)
 * 额外: Data[6] 温度, Data[7] 错误码
 * ============================================================ */
void DM4310_ParseFeedback(uint32_t can_id, const uint8_t data[8])
{
    uint8_t idx;
    if (can_id == 0x1FE)      idx = 0;   /* PAN */
    else if (can_id == 0x1FD) idx = 1;   /* TILT */
    else return;

    uint16_t raw_pos = ((uint16_t)data[0] << 8) | data[1];

    /* DM4310 编码器 0~65535 映射到 0~2π rad */
    gimbals[idx].position_rad  = (float)raw_pos / 65535.0f * 6.283185f;
    gimbals[idx].velocity_rad_s = (float)((int16_t)((data[2] << 8) | data[3])) * 0.1f;
    gimbals[idx].torque_current = (int16_t)((data[4] << 8) | data[5]);
    gimbals[idx].temperature   = data[6];
    gimbals[idx].online        = true;
}

/* ============================================================
 * Gimbal_MITControl - DM4310 MIT 模式全状态控制
 *
 * MIT 模式公式:
 *   τ = Kp × (θ_target - θ_actual) + Kd × (ω_target - ω_actual) + τ_ff
 *   τ → int16_t 电流指令
 *
 * 简化: 位置+速度前馈, 扭矩前馈用于重力补偿
 * ============================================================ */
void Gimbal_MITControl(float pan_pos, float pan_vel, float pan_torque,
                       float tilt_pos, float tilt_vel, float tilt_torque)
{
    /* PAN 电机 */
    {
        float pos_err = pan_pos - gimbals[0].position_rad;
        float vel_err = pan_vel - gimbals[0].velocity_rad_s;
        float tau = POS_KP * pos_err + POS_KD * vel_err + pan_torque;

        /* 电流限幅 */
        if (tau >  DM4310_MAX_CURRENT)  tau =  DM4310_MAX_CURRENT;
        if (tau < -DM4310_MAX_CURRENT)  tau = -DM4310_MAX_CURRENT;

        gimbals[0].target_current   = (int16_t)tau;
        gimbals[0].target_position  = pan_pos;
        gimbals[0].target_velocity  = pan_vel;
        gimbals[0].kp = POS_KP;
        gimbals[0].kd = POS_KD;
    }

    /* TILT 电机 */
    {
        float pos_err = tilt_pos - gimbals[1].position_rad;
        float vel_err = tilt_vel - gimbals[1].velocity_rad_s;
        float tau = POS_KP * pos_err + POS_KD * vel_err + tilt_torque;

        if (tau >  DM4310_MAX_CURRENT)  tau =  DM4310_MAX_CURRENT;
        if (tau < -DM4310_MAX_CURRENT)  tau = -DM4310_MAX_CURRENT;

        gimbals[1].target_current   = (int16_t)tau;
        gimbals[1].target_position  = tilt_pos;
        gimbals[1].target_velocity  = tilt_vel;
        gimbals[1].kp = POS_KP;
        gimbals[1].kd = POS_KD;
    }

    /* 发送 CAN 控制帧: PAN (ID 0x1FF) + TILT (ID 0x200) */
    /*
    // PAN: ID 0x1FF
    {
        uint8_t data[8];
        int16_t pos_i = (int16_t)(pan_pos * 100.0f);
        int16_t vel_i = (int16_t)(pan_vel * 100.0f);
        data[0] = (pos_i >> 8) & 0xFF;  data[1] = pos_i & 0xFF;
        data[2] = (vel_i >> 8) & 0xFF;  data[3] = vel_i & 0xFF;
        data[4] = (pan_torque >> 8) & 0xFF; data[5] = pan_torque & 0xFF;
        data[6] = 0;  data[7] = 0;
        CAN_SendFrame(0x1FF, data);
    }

    // TILT: ID 0x200 (DM4310 第二通道)
    {
        uint8_t data[8];
        int16_t pos_i = (int16_t)(tilt_pos * 100.0f);
        int16_t vel_i = (int16_t)(tilt_vel * 100.0f);
        data[0] = (pos_i >> 8) & 0xFF;  data[1] = pos_i & 0xFF;
        data[2] = (vel_i >> 8) & 0xFF;  data[3] = vel_i & 0xFF;
        data[4] = (tilt_torque >> 8) & 0xFF; data[5] = tilt_torque & 0xFF;
        data[6] = 0;  data[7] = 0;
        CAN_SendFrame(0x200, data);
    }
    */
}

/* ============================================================
 * Gimbal_SetAngle - 位置闭环: 匀速逼近目标角度
 * ============================================================ */
void Gimbal_SetAngle(float pan_rad, float tilt_rad)
{
    /* 角度限幅 */
    if (pan_rad  < DM4310_POS_MIN)  pan_rad  = DM4310_POS_MIN;
    if (pan_rad  > DM4310_POS_MAX)  pan_rad  = DM4310_POS_MAX;
    if (tilt_rad < DM4310_POS_MIN)  tilt_rad = DM4310_POS_MIN;
    if (tilt_rad > DM4310_POS_MAX)  tilt_rad = DM4310_POS_MAX;

    /* 目标速度: 根据位置差设定 (梯形速度规划简化) */
    float pan_err  = pan_rad  - gimbals[0].position_rad;
    float tilt_err = tilt_rad - gimbals[1].position_rad;

    float pan_vel  = (fabsf(pan_err)  > 0.1f) ? (pan_err > 0 ? 5.0f : -5.0f) : 0.0f;
    float tilt_vel = (fabsf(tilt_err) > 0.1f) ? (tilt_err > 0 ? 5.0f : -5.0f) : 0.0f;

    /* 速度限幅 */
    if (pan_vel  >  DM4310_VEL_MAX)  pan_vel  =  DM4310_VEL_MAX;
    if (pan_vel  < -DM4310_VEL_MAX)  pan_vel  = -DM4310_VEL_MAX;
    if (tilt_vel >  DM4310_VEL_MAX)  tilt_vel =  DM4310_VEL_MAX;
    if (tilt_vel < -DM4310_VEL_MAX)  tilt_vel = -DM4310_VEL_MAX;

    Gimbal_MITControl(pan_rad, pan_vel, 0.0f, tilt_rad, tilt_vel, 0.0f);
}

/* ============================================================
 * Motor_GetSpeedRPM / Motor_GetPositionDeg
 * ============================================================ */
float Motor_GetSpeedRPM(MotorID_t motor)
{
    if (motor >= 4) return 0;
    return (float)motors[motor].speed_rpm;
}

float Motor_GetPositionDeg(MotorID_t motor)
{
    if (motor >= 4) return 0;
    return (float)motors[motor].encoder / (float)M3508_POS_RANGE * 360.0f;
}

void Motor_GetStates(MotorState_t states[4])
{
    memcpy(states, motors, sizeof(motors));
}

void Gimbal_GetStates(GimbalState_t states[2])
{
    memcpy(states, gimbals, sizeof(gimbals));
}

/* ============================================================
 * Motor_SelfTest - 开机自检
 * 低速正转→反转→停机, 检查编码器反馈是否正常
 * ============================================================ */
bool Motor_SelfTest(void)
{
    /* 发送小电流测试 */
    for (int i = 0; i < 4; i++) {
    