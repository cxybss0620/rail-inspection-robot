/**
 * sensor.c - 传感器驱动实现
 * STM32F407 外设: I2C1(IMU) / UART2(LiDAR) / UART3(5G) / USART1(DEBUG)
 * ADC: PA0(电池电压) / PA1(电池电流) / PA2(红外测温)
 * 定时器: TIM2(超声波触发) / TIM3(PWM电机) / TIM4(PWM舵机)
 */

#include "sensor.h"
#include <math.h>
#include <string.h>

/* ── 内部状态 ── */
static bool sensors_ok = false;

/* ── I2C: MPU6050/ICM20948寄存器 ── */
#define MPU6050_ADDR        0x68
#define REG_ACCEL_XOUT_H    0x3B
#define REG_GYRO_XOUT_H     0x43
#define REG_PWR_MGMT1       0x6B
#define ACCEL_SCALE         (16384.0f)   /* ±2g -> LSB/g */
#define GYRO_SCALE          (131.0f)     /* ±250°/s -> LSB/°/s */

/* ============================================================
 * Sensor_Init - 初始化所有外设
 * ============================================================ */
void Sensor_Init(void)
{
    /* --- I2C1: IMU --- */
    /* 实际代码: HAL_I2C_Init(&hi2c1) */
    /* 唤醒 MPU6050: 写0x00到 PWR_MGMT1 */
    {
        uint8_t buf[2] = {REG_PWR_MGMT1, 0x00};
        /* HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR<<1, buf, 2, 100); */
    }

    /* --- ADC: 电池电压/电流/红外测温 --- */
    /* HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, 3); */

    /* --- 超声波: GPIO触发 + 输入捕获 --- */
    /* TIM2 CH1-CH4 用于4路超声波, 另外4路用EXTI */

    /* --- UART: RTK (115200-8N1) --- */
    /* HAL_UART_Receive_DMA(&huart2, rtk_buf, RTK_BUF_SIZE); */

    sensors_ok = true;
}

/* ============================================================
 * IMU_ReadRaw - 读取IMU原始数据
 * 返回: true=成功
 * ============================================================ */
bool IMU_ReadRaw(IMUData_t* data)
{
    if (!sensors_ok) return false;

    /* 实际代码: 通过I2C突发读取14字节(ACCEL[6]+TEMP[2]+GYRO[6]) */
    /*
    uint8_t raw[14];
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR<<1, REG_ACCEL_XOUT_H,
                     I2C_MEMADD_SIZE_8BIT, raw, 14, 100);

    int16_t ax = (raw[0]<<8)|raw[1];
    int16_t ay = (raw[2]<<8)|raw[3];
    int16_t az = (raw[4]<<8)|raw[5];
    int16_t tp = (raw[6]<<8)|raw[7];
    int16_t gx = (raw[8]<<8)|raw[9];
    int16_t gy = (raw[10]<<8)|raw[11];
    int16_t gz = (raw[12]<<8)|raw[13];

    data->accel_x = ax / ACCEL_SCALE * 9.81f;
    data->accel_y = ay / ACCEL_SCALE * 9.81f;
    data->accel_z = az / ACCEL_SCALE * 9.81f;
    data->gyro_x  = gx / GYRO_SCALE * (3.14159f/180.0f);
    data->gyro_y  = gy / GYRO_SCALE * (3.14159f/180.0f);
    data->gyro_z  = gz / GYRO_SCALE * (3.14159f/180.0f);
    data->temperature = tp/340.0f + 36.53f;
    */

    /* --- 模拟数据(无硬件时的测试用) --- */
    data->accel_x = 0.0f;
    data->accel_y = 0.0f;
    data->accel_z = 9.81f;
    data->gyro_x  = 0.0f;
    data->gyro_y  = 0.0f;
    data->gyro_z  = 0.0f;
    data->temperature = 28.5f;

    /* 时间戳 (FreeRTOS tick → ms) */
    data->timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    return true;
}

/* ============================================================
 * Ultrasonic_Read - 读取8路超声波测距
 * 原理: GPIO Trig 10us高电平 → 测量Echo高电平时间 → 距离=时间*声速/2
 * 返回: true=全部通道有效
 * ============================================================ */
bool Ultrasonic_Read(UltrasonicData_t* data)
{
    if (!sensors_ok) return false;

    /* 实际代码: 依次触发8路, 输入捕获测量回波时间 */
    /*
    for (int ch = 0; ch < ULTRASONIC_CHANNELS; ch++) {
        TriggerUltrasonic(ch);
        uint32_t pulse_us = WaitEcho(ch, 50000);  // 超时50ms
        data->distances[ch] = pulse_us * 0.1715f;  // 声速343m/s, 往返/2
        if (pulse_us == 0) data->distances[ch] = 9999.0f; // 超时
    }
    */

    /* 模拟数据 */
    for (int i = 0; i < ULTRASONIC_CHANNELS; i++) {
        data->distances[i] = 500.0f + (float)(i * 200);  /* 500~1900mm */
    }
    data->timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    return true;
}

/* ============================================================
 * RTK_ReadNMEA - 解析RTK NMEA报文
 * 格式: $GNGGA,time,lat,N,lon,E,quality,numSV,HDOP,alt,M,...
 * 示例: $GNGGA,092750.00,3123.04123,N,12147.37234,E,4,12,0.8,45.2,M,...
 * ============================================================ */
bool RTK_ReadNMEA(RTKData_t* data)
{
    /* 实际代码: UART DMA环形缓冲区 + NMEA解析状态机 */
    /*
    if (!g_nmea_ready) return false;

    // 解析纬度: ddmm.mmmmm → dd + mm/60
    float lat_raw = g_nmea.lat_raw;  // 3123.04123
    int lat_deg = (int)(lat_raw / 100);
    float lat_min = lat_raw - lat_deg * 100;
    data->latitude = lat_deg + lat_min / 60.0f;

    // 解析经度
    float lon_raw = g_nmea.lon_raw;
    int lon_deg = (int)(lon_raw / 100);
    float lon_min = lon_raw - lon_deg * 100;
    data->longitude = lon_deg + lon_min / 60.0f;

    data->fix_quality = g_nmea.fix_quality;
    data->hdop = g_nmea.hdop;
    data->altitude = g_nmea.altitude;
    */

    /* 模拟数据 */
    data->latitude  = 31.230412;
    data->longitude = 121.473723;
    data->altitude  = 45.2f;
    data->fix_quality = 4;  /* RTK固定解 */
    data->hdop = 0.8f;
    data->timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    return true;
}

/* ============================================================
 * Battery_Read - 读取电池状态 (ADC采样)
 * ============================================================ */
bool Battery_Read(BatteryState_t* bat)
{
    /* 分压电阻: R1=100k, R2=10k → 分压比 = 10/(100+10) = 1/11 */
    /* ADC 12bit, Vref=3.3V → 每LSB=3.3/4096=0.805mV */
    /*
    uint16_t volt_adc = adc_buf[0];
    uint16_t curr_adc = adc_buf[1];
    float v_adc = volt_adc * 3.3f / 4096.0f;
    bat->voltage = v_adc * 11.0f;  // 反推分压

    // 电流采样(ACS712 185mV/A)
    float c_adc = curr_adc * 3.3f / 4096.0f;
    bat->current = (c_adc - 1.65f) / 0.185f;  // ACS712零点1.65V
    */

    bat->voltage = 46.5f;
    bat->current = 2.3f;
    bat->soc_percent = (uint8_t)((bat->voltage - 36.0f) / (48.0f - 36.0f) * 100);
    bat->temperature = 32.0f;

    return true;
}

/* ============================================================
 * GetRailTemperature - 红外测温探头 (MLX90614 或 ADC热电堆)
 * ============================================================ */
float GetRailTemperature(void)
{
    /* 实际: I2C读取 MLX90614 或 ADC读取热电堆 */
    return 28.0f;
}

/* ============================================================
 * Sensor_SelfTest - 开机自检
 * ============================================================ */
bool Sensor_SelfTest(void)
{
    /* 检查各传感器是否响应 */
    IMUData_t imu;
    if (!IMU_ReadRaw(&imu)) return false;

    BatteryState_t bat;
    if (!Battery_Read(&bat)) return false;

    /* 检查电池电压 */
    if (bat.voltage < BAT_VOLT_CRITICAL) return false;

    return true;
}

/* ============================================================
 * Sensor_UpdateFrame - 更新全局传感器帧 (供CAN周期性发送)
 * ============================================================ */
void Sensor_UpdateFrame(void)
{
    SensorFrame_t* f = (SensorFrame_t*)&g_sensor_frame;

    f->frame_id++;
    f->timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    IMU_ReadRaw(&f->imu);
    Ultrasonic_Read(&f->ultrasonic);
    Battery_Read(&f->battery);
    f->rail_temp = GetRailTemperature();

    /* 姿态估计(简化: 仅用IMU积分) */
    static float yaw = 0;
    yaw += f->imu.gyro_z * 0.01f;  /* 100Hz积分 */
    f->pose_estimate.yaw = yaw;

    /* 告警标志 */
    f->alert_flags = 0;
    if (f->battery.voltage < BAT_VOLT_LOW)
        f->alert_flags |= 0x01;  /* 低电量 */
    for (int i = 0; i < ULTRASONIC_CHANNELS; i++) {
        if (f->ultrasonic.distances[i] < BRAKE_L3_DIST)
            f->alert_flags |= 0x02;  /* 障碍物紧急 */
    }
}
