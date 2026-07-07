# 铁脉医者 — 轨道巡检机器人

> **铁脉医者** — 铁轨如血脉，医者治未病。
>
> 基于 STM32F407 + Intel NUC 的智能轨道巡检机器人三层架构，实现传感器数据采集、YOLOv8 视觉病害检测、电机 PID 闭环控制、CAN 总线通信、云端可视化管控。**嵌入式芯片设计大赛参赛作品**。

---

## 系统架构

```
┌──────────────────────────────────────────────┐
│              云端管控平台 (PC)                 │
│         Flask Web + SQLite + WebSocket        │
│     仪表盘 / 病害查询 / 远程遥控 / 数据导出     │
└──────────────────┬───────────────────────────┘
                   │ HTTP (NUC → Cloud)
┌──────────────────┴───────────────────────────┐
│         计算平台 (Intel NUC / Linux)           │
│    YOLOv8 视觉检测 · EKF 多源数据融合          │
│    STM32 串口驱动 · 传感器帧解析 · 决策控制      │
└──────────────────┬───────────────────────────┘
                   │ UART (STM32 → NUC)
┌──────────────────┴───────────────────────────┐
│          运动控制平台 (STM32F407IGT6)           │
│              FreeRTOS 多任务调度                │
│  ┌──────────┐ ┌──────────┐ ┌──────────────┐  │
│  │ 传感器    │ │ 电机驱动  │ │  CAN 通信    │  │
│  │ IMU/超声 │ │ DJI M3508│ │  CAN 1Mbps   │  │
│  │ RTK/电池 │ │ 达妙4310 │ │  UART 透传   │  │
│  └──────────┘ └──────────┘ └──────────────┘  │
│  ┌──────────┐ ┌──────────┐ ┌──────────────┐  │
│  │ 姿态解算 │ │ 安全监控  │ │  心跳+LED     │  │
│  │ 互补滤波 │ │ 电池/温度 │ │  蜂鸣器告警   │  │
│  └──────────┘ └──────────┘ └──────────────┘  │
└──────────────────────────────────────────────┘
```

### 数据流

1. **STM32** 采集 IMU/超声/RTK/电池/电机编码器 → 40 字节二进制帧 → UART → **NUC**
2. **NUC** 解析帧 + 相机采集 + YOLOv8 检测 → EKF 融合 → 决策指令 → UART → **STM32**
3. **NUC** 聚合数据 → HTTP → **云端 Flask** → WebSocket 推送 → **Web 仪表盘**

---

## 目录结构

```
├── lower_computer_stm32/       # STM32 运动控制层 (C / FreeRTOS)
│   ├── rail_inspect.h          # 引脚定义 · 数据结构 · 全局状态
│   ├── sensor.h / sensor.c     # 传感器驱动 (I2C·UART·ADC)
│   ├── motor.h / motor.c       # DJI M3508 + 达妙 DM4310 电机驱动
│   ├── can_comm.h / can_comm.c # CAN 通信协议
│   ├── main.c                  # 主程序 (FreeRTOS 6 任务调度)
│   ├── stm32f4xx_it.c          # 中断服务程序
│   └── FreeRTOSConfig.h        # RTOS 配置
│
├── nuc_computer/               # NUC 计算层 (Python)
│   ├── config.py               # 全局参数配置
│   ├── stm32_serial.py         # STM32 串口驱动 · 帧解析
│   ├── defect_detector.py      # YOLOv8 病害检测 · 尺寸测算 · 分级预警
│   ├── fusion_engine.py        # EKF 融合 · 互补滤波 · 制动决策
│   └── engine.py               # 主调度引擎
│
├── upper_computer/             # 云端管控层 (Flask)
│   ├── app.py                  # Web 仪表盘 · REST API · WebSocket
│   └── requirements.txt        # Python 依赖
│
└── README.md
```

---

## 各层技术说明

### 一、STM32 运动控制层

| 项目 | 参数 |
|------|------|
| MCU | STM32F407IGT6 (Cortex-M4 @168MHz) |
| RTOS | FreeRTOS v10 |
| 传感器 | MPU6050 IMU (I2C) · 8ch 超声波 · RTK GNSS (UART) · ADC 电池检测 |
| 底盘电机 | **DJI M3508** 无刷减速电机 ×4 (CAN 1Mbps 电流闭环) |
| 云台电机 | **达妙 DM4310** 无刷云台电机 ×2 (CAN MIT 模式) |
| 通信 | CAN 1Mbps · UART1 Debug · UART2 LiDAR · UART3 5G/NUC |

**FreeRTOS 任务分配**：

| 任务 | 频率 | 优先级 | 职责 |
|------|------|--------|------|
| Task_Sensor | 100Hz | 5 | 传感器轮询采集, 填充全局帧 |
| Task_Mot