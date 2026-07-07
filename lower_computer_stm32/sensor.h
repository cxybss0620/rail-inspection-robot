/**
 * sensor.h - 传感器驱动模块
 * STM32F407: I2C IMU + UART超声波 + UART RTK + ADC电池
 */

#ifndef SENSOR_H
#define SENSOR_H

#include "rail_inspect.h"

/* 初始化所有传感器 */
void Sensor_Init(void);

/* 读取IMU (MPU6050 / ICM20948 via I2C) */
bool IMU_ReadRaw(IMUData_t* data);

/* 读取超声波测距 (8路轮询) */
bool Ultrasonic_Read(UltrasonicData_t* data);

/* 读取RTK定位 (UART NMEA解析) */
bool RTK_ReadNMEA(RTKData_t* data);

/* 读取电池状态 (ADC采样) */
bool Battery_Read(BatteryState_t* bat);

/* 读取轨道温度 (红外测温探头 ADC) */
float GetRailTemperature(void);

/* 传感器自检 */
bool Sensor_SelfTest(void);

/* 更新全局传感器帧(供CAN发送) */
void Sensor_UpdateFrame(void);

#endif
