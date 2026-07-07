/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       auto_control.c/h
  * @brief      miniPC接管操作全自动控制时所需要用到的功能与相关函数。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2023-6-4        Penguin         1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#ifndef AUTO_CONTROL_H
#define AUTO_CONTROL_H

#include "struct_typedef.h"
#include "stdbool.h"

#define MAX_AIM_ERROR_PITCH 0.1f //(rad) 最大瞄准pitch误差
#define MAX_AIM_ERROR_YAW 0.1f //(rad) 最大瞄准yaw误差
#define SCAN_DYAW 0.2f     // (rad) 扫描偏航角增量

#define DIAMETER 17 // (mm) 弹丸直径

// 弹丸相关参数
#if (DIAMETER == 17)
#define V0 14                // (m/s) 小弹丸初速度
#define FRICTION_COEFF 0.038f // 17mm小单丸受到的空气阻力系数

#elif (DIAMETER == 42)
#define V0 8                 // (m/s) 大弹丸初速度
#define FRICTION_COEFF 0.16f // 42mm大弹丸受到的空气阻力系数//get_friction_coeff(0.53, 1.293, 0.00001385);

#else
//当弹丸直径填写错误的时候跳出错误提示
#error "The DIAMETER is not 17 or 42, plase write 17 or 42! ! ! !"
#endif


#define BIAS_TIME 5 // (ms) USB通信偏差时间
#define FIRE_CONTINUE_TIME 500 //(ms) 连续开火时间

// 机器人结构相关参数
#define S_STATIC 0.04734f // (m) 枪口前推的距离 根据机器人的实际情况做出改变，具体值请从图纸上查阅
#define Z_STATIC 0.17025f // (m) yaw轴电机到枪口水平面的垂直距离 根据机器人的实际情况做出改变，具体值请从图纸上查阅

// 旋转矩阵用相关参数
#define ROTATE_YAW 0   // (rad) yaw旋转参数
#define ROTATE_PITCH 0 // (rad) pitch旋转参数
#define ROTATE_ROLL 0  // (rad) roll旋转参数

typedef struct
{
  float yaw;         // 射击偏航角
  float pitch;       // 射击俯仰角
  float x;           // 落点x坐标
  float y;           // 落点y坐标
  float z;           // 落点z坐标
} trajectory_data_t; // 弹道参数

typedef struct
{
  fp32 yaw;         // 陀螺仪测得的偏航角
  fp32 pitch;       // 陀螺仪测得的俯仰角
  fp32 roll;        // 陀螺仪测得的滚转角
} gyro_angle_rad_t; // 陀螺仪参数

typedef struct
{
  float x;
  float y;
  float z;
  float vx;
  float vy;
  float vz;
} target_data_t; // 锁定目标的参数

// 开放调用的变量

static bool JudgeFireFlag(trajectory_data_t *AimTra, gyro_angle_rad_t *gyro_angle_rad);

static void AimPositionCalculate(trajectory_data_t *AimTra, gyro_angle_rad_t GAR_A,
                                 float s_static, float z_static,
                                 float v0, float k, int BiasTime);

static void CurrentPositionCalculate(trajectory_data_t *CurTra, gyro_angle_rad_t GAR_C, float v0, float k, float distance);

static float get_distance(float dx, float dy, float dz);

static void GyroAngleRadRotate(gyro_angle_rad_t *gyro_angle_rad, float yaw, float pitch, float roll);

void AutoAimConsole(fp32 *yaw, fp32 *pitch);

bool GetFireFlag(void);

const bool *GetFireFlagPoint(void);

#endif // AUTO_CONTROL_H