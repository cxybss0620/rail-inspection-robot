/**
 * motor.h - 电机驱动模块
 *
 * 底盘电机: DJI M3508 无刷减速电机 ×4（CAN 总线, 1Mbps）
 *   CAN ID: 0x201(前左) / 0x202(前右) / 0x203(后左) / 0x204(后右)
 *   反馈: 转子角度(0-8191) / 转速(rpm) / 扭矩电流(A)
 *   控制: 电流环, int16_t -16384~16384 映射 -20A~+20A
 *
 * 云台电机: 达妙 DM4310 无刷云台电机 ×2（CAN 总线）
 *   CAN ID: 0x1FF(PAN俯仰+偏航一起控制)
 *   反馈: 编码器角度 / 转速
 *   控制: MIT 模式 (位置+速度+扭矩前馈)
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "rail_inspect.h"
#include <stdbool.h>

/* ── M3508 参数 ── */
#define M3508_CAN_BASE_ID       0x200       /* 底盘电机起始ID: 0x201~0x204 */
#define M3508_REDUCTION_RATIO   19.203208f  /* 3591/187 */
#define M3508_MAX_CURRENT       16384       /* 电流指令范围 -16384 ~ +16384 */
#define M3508_POS_RANGE         8192        /* 转子一圈编码器计数 */
#define M3508_KT                0.3f        /* 扭矩常数 Nm/A (估算) */

/* ── DM4310 云台电机参数 ── */
#define DM4310_CAN_ID           0x1FF       /* 云台电机 CAN ID */
#define DM4310_MAX_CURRENT      20000       /* 电流限幅 */
#define DM4310_POS_MIN          -12.5f      /* 角度下限 rad */
#define DM4310_POS_MAX          12.5f       /* 角度上限 rad */
#define DM4310_VEL_MAX          30.0f       /* 最大角速度 rad/s */

/* ── 电机索引 ── */
typedef enum {
    MOTOR_FRONT_LEFT = 0,
    MOTOR_FRONT_RIGHT,
    MOTOR_REAR_LEFT,
    MOTOR_REAR_RIGHT,
} MotorID_t;

/* M3508 反馈数据 (CAN接收端解析) */
typedef struct {
    uint16_t encoder;           /* 转子机械角度 0-8191 */
    int16_t speed_rpm;          /* 转子转速 r/min */
    int16_t torque_current;     /* 扭矩电流 (×0.01A) */
    uint8_t temperature;        /* 电机温度 °C */
} M3508_Feedback_t;

/* DM4310 反馈数据 */
typedef struct {
    float position_rad;         /* 编码器角度 rad */
    float velocity_rad_s;       /* 角速度 rad/s */
    int16_t torque_current;     /* 扭矩电流 */
    uint8_t temperature;
} DM4310_Feedback_t;

/* 电机状态 */
typedef struct {
    uint16_t encoder;           /* 当前编码器值 */
    int16_t speed_rpm;          /* 实测转速 */
    int16_t target_current;     /* 目标电流 -16384~+16384 */
    int16_t actual_current;     /* 实际电流 */
    uint8_t temperature;        /* 温度 */
    bool online;                /* CAN通信在线(最后一次收到反馈3ms以内) */
    bool fault;                 /* 故障标志 */
} MotorState_t;

/* 云台状态 */
typedef struct {
    float position_rad;
    float velocity_rad_s;
    float target_position;
    float target_velocity;
    float kp, kd;               /* MIT模式刚度/阻尼 */
    int16_t target_current;
    uint8_t temperature;
    bool online;
} GimbalState_t;

/* ========== API ========== */

/* 初始化: CAN滤波器配置 */
void Motor_Init(void);

/* ── M3508 底盘电机 ── */

/* 设置单个电机电流 (范围 -16384 ~ +16384) */
void Motor_SetCurrent(MotorID_t motor, int16_t current);

/* 设置电机转速(内部速度PI → 电流指令) */
void Motor_SetSpeed(MotorID_t motor, float speed_rpm);

/* 四轮差速: 线速度 m/s + 角速度 rad/s → 四轮转速 */
void Motor_DiffDrive(float v_linear, float v_angular);

/* 紧急制动: 电流清零 + 短路制动 */
void Motor_EmergencyStop(void);

/* 电机缓停 (斜坡减速) */
void Motor_CoastStop(void);

/* ── CAN 回调 ── */

/* M3508 反馈解析 (CAN1 RX中断中调用) */
void M3508_ParseFeedback(uint32_t can_id, const uint8_t data[8]);

/* DM4310 反馈解析 */
void DM4310_ParseFeedback(uint32_t can_id, const uint8_t data[8]);

/* ── DM4310 云台电机 ── */

/* MIT 模式控制: 位置+速度+扭矩前馈 */
void Gimbal_MITControl(float pan_pos, float pan_vel, float pan_torque,
                       float tilt_pos, float tilt_vel, float tilt_torque);

/* 位置闭环控制 (内部 MIT) */
void Gimbal_SetAngle(float pan_rad, float tilt_rad);

/* ── 状态查询 ── */
float Motor_GetSpeedRPM(MotorID_t motor);
float Motor_GetPositionDeg(MotorID_t motor);
void M