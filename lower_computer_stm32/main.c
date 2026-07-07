/**
 * main.c - 轨道巡检机器人下位机主程序
 * 平台: STM32F407IGT6, FreeRTOS v10
 * 功能: 多任务调度 - 传感器采集/数据融合/导航控制/通信/安全监控
 *
 * FreeRTOS 任务分配:
 *   Task1 (高优先级, 100Hz): 传感器采集+预处理
 *   Task2 (高优先级, 100Hz): 运动控制+PID闭环
 *   Task3 (中优先级, 50Hz):  数据融合+位姿估计
 *   Task4 (中优先级, 20Hz):  通信管理(CAN发送/5G上传)
 *   Task5 (低优先级, 10Hz):  安全监控+告警
 *   Task6 (低优先级, 1Hz):   心跳+日志
 */

#include "rail_inspect.h"
#include "sensor.h"
#include "motor.h"
#include "can_comm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

/* ── FreeRTOS 任务句柄 ── */
static TaskHandle_t task_sensor_handle;
static TaskHandle_t task_motion_handle;
static TaskHandle_t task_fusion_handle;
static TaskHandle_t task_comm_handle;
static TaskHandle_t task_safety_handle;
static TaskHandle_t task_heartbeat_handle;

/* ── 队列: 传感器数据 [size: 8] ── */
static QueueHandle_t sensor_queue;
/* ── 队列: 运动指令 [size: 4] ── */
static QueueHandle_t cmd_queue;
/* ── 信号量: CAN发送完成 ── */
static SemaphoreHandle_t can_tx_sem;

/* ── 全局状态 ── */
volatile RobotState_t g_robot_state = STATE_IDLE;
volatile SensorFrame_t g_sensor_frame = {0};
volatile MotionCmd_t g_motion_cmd = {0};

/* ── 系统运行标志 ── */
static volatile bool system_running = false;

/* ============================================================
 * 系统初始化
 * ============================================================ */
static void SystemClock_Config(void)
{
    /* HSE 8MHz → PLL → 168MHz */
    /*
    RCC_OscInitTypeDef osc;
    RCC_ClkInitTypeDef clk;

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = 8;
    osc.PLL.PLLN = 336;
    osc.PLL.PLLP = RCC_PLLP_DIV2;   // 168MHz
    osc.PLL.PLLQ = 7;               // 48MHz (USB/SDIO)
    HAL_RCC_OscConfig(&osc);
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5);
    */
}

static void GPIO_Init(void)
{
    /* LED: PC13/PC14/PC15 */
    /* 蜂鸣器: PC15 */
    /* 按键: PA0 (WKUP) */
}

/* ============================================================
 * 初始化入口
 * ============================================================ */
void System_Init(void)
{
    /* HAL库初始化 */
    /* HAL_Init(); */
    SystemClock_Config();
    GPIO_Init();

    /* 外设初始化 */
    Sensor_Init();
    Motor_Init();
    CAN_Init();

    /* FreeRTOS 对象 */
    sensor_queue = xQueueCreate(8, sizeof(SensorFrame_t));
    cmd_queue = xQueueCreate(4, sizeof(MotionCmd_t));
    can_tx_sem = xSemaphoreCreateBinary();

    /* 上电自检 */
    if (Sensor_SelfTest() && Motor_SelfTest()) {
        g_robot_state = STATE_READY;
        system_running = true;
    } else {
        g_robot_state = STATE_ERROR;
    }
}

/* ============================================================
 * Task 1: 传感器采集 (100Hz, 最高优先级)
 * 读取IMU/超声/RTK/电池 → 预处理 → 写入队列
 * ============================================================ */
void Task_SensorAcquisition(void* param)
{
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10);  /* 10ms = 100Hz */

    while (system_running) {
        vTaskDelayUntil(&last_wake, period);

        /* 更新全局传感器帧 */
        Sensor_UpdateFrame();

        /* 写入队列 (非阻塞) */
        SensorFrame_t frame = g_sensor_frame;
        xQueueOverwrite(sensor_queue, &frame);

        /* 紧急状态: 障碍物<500mm → 立即制动 */
        if (frame.alert_flags & 0x02) {
            Motor_EmergencyStop();
            g_robot_state = STATE_EMERGENCY_STOP;
            /* 蜂鸣器+LED */
            /* HAL_GPIO_WritePin(GPIOC, BUZZER_PIN, GPIO_PIN_SET); */
            /* HAL_GPIO_WritePin(LED_ALERT_PORT, LED_ALERT_PIN, GPIO_PIN_SET); */
        }
    }
}

/* ============================================================
 * Task 2: 运动控制 (100Hz)
 * 读取指令队列 → PID闭环 → PWM输出 → 编码器反馈
 * ============================================================ */
void Task_MotionControl(void* param)
{
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10);

    while (system_running) {
        vTaskDelayUntil(&last_wake, period);

        MotionCmd_t cmd;
        /* 从队列读取最新指令 (非阻塞) */
        if (xQueueReceive(cmd_queue, &cmd, 0) == pdTRUE) {
            g_motion_cmd = cmd;
        }

        /* 执行运动控制 */
        if (g_robot_state == STATE_CRUISING || g_robot_state == STATE_INSPECTING) {
            /* 实际速度 (RPM) */
            float wheel_rpm = g_motion_cmd.target_speed * 60.0f / (0.15f * 3.14159f);
            Motor_SetAllSpeed(wheel_rpm);
            Servo_SetAngle(g_motion_cmd.servo_pan, g_motion_cmd.servo_tilt);
        } else if (g_robot_state == STATE_EMERGENCY_STOP) {
            Motor_EmergencyStop();
        }
    }
}

/* ============================================================
 * Task 3: 数据融合 (50Hz)
 * 读取传感器队列 → 姿态解算(互补滤波) → 位姿估计
 * ============================================================ */
void Task_DataFusion(void* param)
{
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20);  /* 20ms = 50Hz */
    SensorFrame_t frame;

    /* 姿态角 (互补滤波) */
    static float roll = 0, pitch = 0, yaw = 0;
    const float alpha = 0.98f;  /* 互补滤波系数 */

    while (system_running) {
        vTaskDelayUntil(&last_wake, period);

        if (xQueuePeek(sensor_queue, &frame, 0) != pdTRUE)
            continue;

        float dt = 0.02f;

        /* 陀螺仪积分 */
        float gx = frame.imu.gyro_x, gy = frame.imu.gyro_y, gz = frame.imu.gyro_z;
        roll += gx * dt;
        pitch += gy * dt;
        yaw += gz * dt;

        /* 加速度计计算倾角 */
        float ax = frame.imu.accel_x, ay = frame.imu.accel_y, az = frame.imu.accel_z;
        float acc_roll = atan2f(ay, az);
        float acc_pitch = atan2f(-ax, sqrtf(ay*ay + az*az));

        /* 互补滤波融合 */
        roll = alpha * roll + (1.0f - alpha) * acc_roll;
        pitch = alpha * pitch + (1.0f - alpha) * acc_pitch;

        /* 更新全局位姿 */
        g_sensor_frame.pose_estimate.roll = roll;
        g_sensor_frame.pose_estimate.pitch = pitch;
        g_sensor_frame.pose_estimate.yaw = yaw;
    }
}

/* ============================================================
 * Task 4: 通信管理 (20Hz)
 * CAN数据发送 + 5G UART透传 + 指令接收
 * ============================================================ */
void Task_Communication(void* param)
{
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(50);  /* 50ms = 20Hz */

    while (system_running) {
        vTaskDelayUntil(&last_wake, period);

        /* 发送传感器数据帧 (CAN) */
        CAN_SendSensorFrame((SensorFrame_t*)&g_sensor_frame);

        /* 发送状态 */
        CAN_SendStatus(g_robot_state, g_sensor_frame.alert_flags);

        /* 5G UART 数据透传 (JSON格式) */
        /*
        char json_buf[256];
        snprintf(json_buf, sizeof(json_buf),
            "{\"f\":%lu,\"bat\":%.1f,\"soc\":%d,\"yaw\":%.2f}",
            g_sensor_frame.frame_id,
            g_sensor_frame.battery.voltage,
            g_sensor_frame.battery.soc_percent,
            g_sensor_frame.pose_estimate.yaw);
        HAL_UART_Transmit(&huart3, (uint8_t*)json_buf, strlen(json_buf), 100);
        */
    }
}

/* ============================================================
 * Task 5: 安全监控 (10Hz)
 * 检查电池/温度/通信/故障
 * ============================================================ */
void Task_SafetyMonitor(void* param)
{
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(100);  /* 100ms = 10Hz */

    while (system_running) {
        vTaskDelayUntil(&last_wake, period);

        BatteryState_t bat;
        Battery_Read(&bat);

        /* 电池低电量 → 返回充电 */
        if (bat.voltage < BAT_VOLT_LOW && g_robot_state != STATE_CHARGING) {
            g_robot_state = STATE_ALERT;
            Motor_SetAllSpeed(0);
            /* 触发返回充电桩导航 */
        }

        /* 电池临界 → 立即停机 */
        if (bat.voltage < BAT_VOLT_CRITICAL) {
            Motor_EmergencyStop();
            g_robot_state = STATE_EMERGENCY_STOP;
        }

        /* 轨道温度异常 */
        float rail_temp = GetRailTemperature();
        if (rail_temp > 65.0f) {  /* >65°C 异常 */
            g_sensor_frame.alert_flags |= 0x04;
        }

        /* 电机故障检测 */
        MotorState_t ms[4];
        Motor_GetStates(ms);
        for (int i = 0; i < 4; i++) {
            if (ms[i].fault) {
                Motor_EmergencyStop();
                g_robot_state = STATE_ERROR;
                break;
            }
        }
    }
}

/* ============================================================
 * Task 6: 心跳 + 状态LED (1Hz)
 * ============================================================ */
void Task_Heartbeat(void* param)
{
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000);

    while (system_running) {
        vTaskDelayUntil(&last_wake, period);

        /* LED闪烁 */
        /* HAL_GPIO_TogglePin(GPIOC, LED_RUN_PIN); */

        /* CAN心跳 */
        CAN_SendHeartbeat();

        /* 调试输出 */
        /*
        printf("[HEARTBEAT] State=%d Batt=%.1fV SOC=%d%%\r\n",
               g_robot_state,
               g_sensor_frame.battery.voltage,
               g_sensor_frame.battery.soc_percent);
        */
    }
}

/* ============================================================
 * CAN 接收中断回调 (在 stm32f4xx_it.c 中转发)
 * ============================================================ */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) == HAL_OK) {
        CAN_RxCallback(&header, data);

        /* 如果是电机指令, 写入指令队列 */
        if (header.ExtId == CAN_ID_MOTOR_BASE) {
            MotionCmd_t cmd = CAN_GetLastCmd();
            xQueueOverwrite(cmd_queue, &cmd);
        }
    }
}

/* ============================================================
 * main() - FreeRTOS入口
 * ============================================================ */
int main(void)
{
    System_Init();

    /* 创建任务 */
    xTaskCreate(Task_SensorAcquisition, "Sensor",   TASK_STACK_DEPTH, NULL, 5, &task_sensor_handle);
    xTaskCreate(Task_MotionControl,     "Motion",   TASK_STACK_DEPTH, NULL, 4, &task_motion_handle);
    xTaskCreate(Task_DataFusion,        "Fusion",   TASK_STACK_DEPTH, NULL, 3, &task_fusion_handle);
    xTaskCreate(Task_Communication,     "Comm",     TASK_STACK_DEPTH, NULL, 2, &task_comm_handle);
    xTaskCreate(Task_SafetyMonitor,     "Safety",   TASK_STACK_DEPTH, NULL, 1, &task_safety_handle);
    xTaskCreate(Task_Heartbeat,         "Heartbeat",TASK_STACK_DEPTH, NULL, 0, &task_heartbeat_handle);

    /* 启动调度器 */
    vTaskStartScheduler();

    /* 不应该到达这里 */
    while (1) {
        /* 调度器异常退出时的应急处理 */
        Motor_EmergencyStop();
    }
}
