/**
 * rail_inspect.h - 轨道巡检机器人下位机主头文件
 * 平台: STM32F407IGT6, ARM Cortex-M4 @168MHz
 * 系统: FreeRTOS v10
 * 通信: CAN 1Mbps + UART(5G模组)
 */

#ifndef RAIL_INSPECT_H
#define RAIL_INSPECT_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 * 硬件引脚定义
 * ============================================================ */
/* CAN */
#define CAN1_RX_PIN         GPIO_PIN_11
#define CAN1_TX_PIN         GPIO_PIN_12
#define CAN1_PORT           GPIOA

/* 电机 - 4路PWM + 4路方向 */
#define MOTOR1_PWM_PIN      GPIO_PIN_6
#define MOTOR1_DIR_PIN      GPIO_PIN_7
#define MOTOR1_PORT         GPIOA
#define MOTOR2_PWM_PIN      GPIO_PIN_0
#define MOTOR2_DIR_PIN      GPIO_PIN_1
#define MOTOR2_PORT         GPIOB
#define MOTOR3_PWM_PIN      GPIO_PIN_8
#define MOTOR3_DIR_PIN      GPIO_PIN_9
#define MOTOR3_PORT         GPIOC
#define MOTOR4_PWM_PIN      GPIO_PIN_6
#define MOTOR4_DIR_PIN      GPIO_PIN_7
#define MOTOR4_PORT         GPIOC

/* 舵机云台 */
#define SERVO_PAN_PIN       GPIO_PIN_0
#define SERVO_TILT_PIN      GPIO_PIN_1
#define SERVO_PORT          GPIOC

/* UART - 传感器/5G */
#define UART1_TX_PIN        GPIO_PIN_9   /* PA9 - 调试 */
#define UART1_RX_PIN        GPIO_PIN_10  /* PA10 */
#define UART2_TX_PIN        GPIO_PIN_2   /* PD5 - 激光雷达 */
#define UART2_RX_PIN        GPIO_PIN_3   /* PD6 */
#define UART3_TX_PIN        GPIO_PIN_10  /* PB10 - 5G模组 */
#define UART3_RX_PIN        GPIO_PIN_11  /* PB11 */

/* I2C - IMU (MPU6050 / ICM-20948) */
#define I2C1_SCL_PIN        GPIO_PIN_6
#define I2C1_SDA_PIN        GPIO_PIN_7
#define I2C1_PORT           GPIOB

/* SPI - 外部Flash (W25Q128) */
#define SPI2_SCK_PIN        GPIO_PIN_13
#define SPI2_MISO_PIN       GPIO_PIN_14
#define SPI2_MOSI_PIN       GPIO_PIN_15
#define SPI2_PORT           GPIOB

/* ADC - 电池电压/电流检测 */
#define ADC_BAT_VOLT_CH     ADC_CHANNEL_0   /* PA0 */
#define ADC_BAT_CURR_CH     ADC_CHANNEL_1   /* PA1 */

/* GPIO - LED/蜂鸣器 */
#define LED_RUN_PIN         GPIO_PIN_13     /* PC13 */
#define LED_ALERT_PIN       GPIO_PIN_14     /* PC14 */
#define BUZZER_PIN          GPIO_PIN_15     /* PC15 */
#define LED_ALERT_PORT      GPIOC

/* ============================================================
 * 系统参数
 * ============================================================ */
#define TASK_STACK_DEPTH    256
#define MAIN_LOOP_FREQ      100     /* Hz */
#define CAN_BAUDRATE        1000000 /* 1Mbps */
#define IMU_SAMPLE_RATE     200     /* Hz */

/* 运动参数 */
#define MAX_SPEED_MS        0.8f    /* 最大线速度 m/s */
#define CRUISE_SPEED_MS     0.5f
#define SLOW_SPEED_MS       0.3f
#define CREEP_SPEED_MS      0.1f
#define MAX_ACCEL_MS2       0.5f    /* 最大加速度 */

/* 制动距离阈值 mm */
#define BRAKE_L1_DIST       5000.0f /* >5m 安全 */
#define BRAKE_L2_DIST       2000.0f /* <2m 警惕 */
#define BRAKE_L3_DIST       500.0f  /* <0.5m 紧急 */

/* 电池 */
#define BAT_VOLT_FULL       48.0f
#define BAT_VOLT_LOW        36.0f   /* 低电量门限 */
#define BAT_VOLT_CRITICAL   32.0f   /* 临界关机 */

/* 超声波通道数 */
#define ULTRASONIC_CHANNELS 8

/* ============================================================
 * 数据结构
 * ============================================================ */

/* IMU 数据 */
typedef struct {
    float accel_x, accel_y, accel_z;     /* m/s^2 */
    float gyro_x, gyro_y, gyro_z;        /* rad/s */
    float temperature;                   /* °C */
    uint32_t timestamp_ms;
} IMUData_t;

/* RTK 定位 */
typedef struct {
    double latitude, longitude;          /* 度 */
    float altitude;                      /* m */
    uint8_t fix_quality;                 /* 0/1/2/4 */
    float hdop;
    uint32_t timestamp_ms;
} RTKData_t;

/* 超声波 */
typedef struct {
    float distances[ULTRASONIC_CHANNELS]; /* mm */
    uint32_t timestamp_ms;
} UltrasonicData_t;

/* 电池状态 */
typedef struct {
    float voltage;                       /* V */
    float current;                       /* A */
    uint8_t soc_percent;                 /* 0-100% */
    float temperature;
} BatteryState_t;

/* 机器人位姿 */
typedef struct {
    float x, y, z;                       /* 世界坐标 m */
    float roll, pitch, yaw;              /* rad */
    float v_linear, v_angular;           /* 速度 */
} RobotPose_t;

/* 运动控制指令 */
typedef struct {
    float target_speed;                  /* m/s */
    float target_steer;                  /* rad */
    uint8_t brake_level;                 /* 0-3 */
    float motor_pwm[4];                  /* 0-100% */
    float servo_pan, servo_tilt;         /* 0-180° */
} MotionCmd_t;

/* 传感器数据帧(通过CAN上传) */
typedef struct {
    uint32_t frame_id;
    uint32_t timestamp_ms;
    IMUData_t imu;
    UltrasonicData_t ultrasonic;
    BatteryState_t battery;
    RobotPose_t pose_estimate;
    float rail_temp;                     /* 轨道温度 */
    uint8_t alert_flags;                 /* 告警标志位 */
} SensorFrame_t;

/* CAN通信协议 - 命令帧 */
typedef struct {
    uint16_t cmd_id;                     /* 命令ID */
    int16_t param1, param2, param3;     /* 参数 */
    uint32_t checksum;
} CAN_CmdFrame_t;

/* CAN通信协议 - 数据帧(分片) */
typedef struct {
    uint8_t type;                        /* 0x01=IMU, 0x02=ULTRASONIC, 0x03=BATTERY */
    uint8_t seq;                         /* 分片序号 0-7 */
    uint8_t data[6];                     /* 载荷 */
    uint16_t crc16;
} CAN_DataFrame_t;

/* ============================================================
 * 全局状态
 * ============================================================ */
typedef enum {
    STATE_IDLE = 0,
    STATE_INIT,
    STATE_READY,
    STATE_CRUISING,       /* 匀速巡检 */
    STATE_INSPECTING,     /* 检测中 */
    STATE_ALERT,          /* 告警 */
    STATE_EMERGENCY_STOP, /* 紧急制动 */
    STATE_CHARGING,       /* 充电中 */
    STATE_ERROR,
} RobotState_t;

extern volatile RobotState_t g_robot_state;
extern volatile SensorFrame_t g_sensor_frame;
extern volatile MotionCmd_t g_motion_cmd;

#endif /* RAIL_INSPECT_H */
