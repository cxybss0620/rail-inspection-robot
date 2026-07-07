/**
 * stm32f4xx_it.c - STM32F4 中断服务程序
 * 外设中断: CAN RX / UART DMA / TIM 编码器 / EXTI 超声波
 */

#include "main.h"
#include "can_comm.h"

/* 外部变量 */
extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;

/* ============================================================
 * SysTick 中断 (1ms) - FreeRTOS 时钟节拍
 * ============================================================ */
void SysTick_Handler(void)
{
    HAL_IncTick();
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xPortSysTickHandler();
    }
}

/* ============================================================
 * CAN1 RX FIFO0 消息挂起中断
 * ============================================================ */
void CAN1_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan1);
}

/* ============================================================
 * CAN1 TX 中断 (可选)
 * ============================================================ */
void CAN1_TX_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan1);
}

/* ============================================================
 * CAN1 SCE (状态变化/错误) 中断
 * ============================================================ */
void CAN1_SCE_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan1);
    CAN_ErrorCallback();
}

/* ============================================================
 * UART2 DMA接收完成中断 (RTK NMEA数据)
 * ============================================================ */
void DMA1_Stream5_IRQHandler(void)
{
    HAL_DMA_IRQHandler(huart2.hdmarx);
}

/* ============================================================
 * UART3 DMA接收完成中断 (5G模组)
 * ============================================================ */
void DMA1_Stream1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(huart3.hdmarx);
}

/* ============================================================
 * TIM5 编码器溢出中断 (前轮编码器)
 * ============================================================ */
void TIM5_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim5);
}

/* ============================================================
 * TIM8 编码器溢出中断 (后轮编码器)
 * ============================================================ */
void TIM8_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim8);
}

/* ============================================================
 * EXTI 超声波回波中断 (8路)
 * 实际: PA4-PA7, PB4-PB7 配置为 EXTI 上升沿/下降沿中断
 * ============================================================ */

/*
void EXTI4_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}

void EXTI9_5_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
}
*/

/* ============================================================
 * HardFault 异常处理
 * ============================================================ */
void HardFault_Handler(void)
{
    /* 紧急制动 */
    Motor_EmergencyStop();
    g_robot_state = STATE_ERROR;

    /* LED快速闪烁指示故障 */
    while (1) {
        /* HAL_GPIO_TogglePin(GPIOC, LED_ALERT_PIN); */
        for (volatile uint32_t i = 0; i < 1000000; i++);
    }
}

/* ============================================================
 * UsageFault / BusFault / MemManage
 * ============================================================ */
void UsageFault_Handler(void)  { while(1); }
void BusFault_Handler(void)    { while(1); }
void MemManage_Handler(void)   { while(1); }
void NMI_Handler(void)         {}
void SVC_Handler(void)         {}
void DebugMon_Handler(void)    {}
void PendSV_Handler(void)      {}
