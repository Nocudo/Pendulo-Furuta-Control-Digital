/**
 ******************************************************************************
 * @file    stm32f4xx_it.c
 * @brief   Manejadores de Interrupción (ISRs) para STM32 Nucleo-F446RE.
 *          - TIM6_DAC_IRQHandler: Dispara el lazo determinista de 1 kHz.
 *          - USART2_IRQHandler: Recepción asíncrona de comandos desde PC.
 *          - EXTI15_10_IRQHandler: Botón de usuario B1 (PC13) para Homing/Modo.
 ******************************************************************************
 */

#include "main.h"
#include "furuta_control.h"

/* Flags volátiles compartidos entre ISRs y el superloop de main.c */
volatile bool g_flag_telemetry_ready = false;
volatile bool g_flag_user_btn_pressed = false;

/**
 * @brief ISR del Sistema Operativo / HAL Tick (1 ms)
 */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

/**
 * @brief ISR del Timer 6 (Base de tiempo exacta de control digital a 1000 Hz)
 */
void TIM6_DAC_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim6);
}

/**
 * @brief Callback de desbordamiento de Timer en HAL (invocado por TIM6 a 1 kHz)
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        /* 1. Ejecutar un paso completo del lazo de control digital (1.0 ms) */
        Furuta_ControlISR_1kHz(&g_furuta);

        /* 2. Decimar envío de telemetría UART a 200 Hz (cada 5 muestras) */
        if ((g_furuta.tick_counter_1khz % TELEMETRY_DECIMATION) == 0U) {
            g_flag_telemetry_ready = true;
        }
    }
}

/**
 * @brief ISR de Interrupción Externa para el Botón Azul B1 (PC13)
 */
void EXTI15_10_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(USER_BTN_PIN);
}

/**
 * @brief Callback de GPIO EXTI al presionar el botón B1 (PC13)
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == USER_BTN_PIN) {
        g_flag_user_btn_pressed = true;
    }
}
