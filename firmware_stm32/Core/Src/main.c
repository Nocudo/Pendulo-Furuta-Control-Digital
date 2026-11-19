/**
 ******************************************************************************
 * @file    main.c
 * @brief   Punto de entrada principal y configuración completa de periféricos
 *          para STM32 Nucleo-F446RE (ARM Cortex-M4F @ 180 MHz):
 *            - RCC: PLL a 180 MHz con Over-Drive (APB1 = 45 MHz, APB2 = 90 MHz)
 *            - TIM1_CH1 (PA8): PWM a 20.0 kHz para ENA del driver L298N
 *            - GPIO (PB4, PB5): Salidas digitales IN1, IN2 para el L298N
 *            - I2C1 (PB8, PB9): Fast Mode 400 kHz -> AS5600 Base (Brazo Theta)
 *            - I2C2 (PB10, PB11): Fast Mode 400 kHz -> AS5600 Distal (Péndulo Alfa)
 *            - TIM6: Interrupción determinista de muestreo a 1.0 kHz (Ts = 1 ms)
 *            - USART2 (PA2, PA3): Telemetría CSV y comandos interactivos (115200 bps)
 ******************************************************************************
 */

#include "main.h"
#include "furuta_control.h"
#include <stdio.h>
#include <string.h>

/* Handles Globales de Periféricos HAL ---------------------------------------*/
TIM_HandleTypeDef  htim1;
TIM_HandleTypeDef  htim6;
I2C_HandleTypeDef  hi2c1;
I2C_HandleTypeDef  hi2c2;
UART_HandleTypeDef huart2;

/* Variables de sincronización con ISRs (definidas en stm32f4xx_it.c) --------*/
extern volatile bool g_flag_telemetry_ready;
extern volatile bool g_flag_user_btn_pressed;
extern volatile char g_uart_rx_ring[128];
extern volatile uint8_t g_uart_rx_head;
extern volatile uint8_t g_uart_rx_tail;

/* Buffer de recepción UART por polling no bloqueante ------------------------*/
static char s_uart_cmd_buf[64];
static uint8_t s_uart_cmd_idx = 0U;

/* Prototipos privados de configuración de periféricos -----------------------*/
static void MX_GPIO_Init(void);
static void MX_TIM1_PWM_Init(void);
static void MX_TIM6_ControlTick_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART2_UART_Init(void);
static void UART_PollCommandRX(void);

/**
 * @brief Función principal (Entry Point)
 */
int main(void)
{
    /* 1. Inicializar capa HAL y configurar SysTick a 1 ms */
    HAL_Init();

    /* 2. Configurar árbol de relojes a 180 MHz (máximo desempeño Cortex-M4F) */
    SystemClock_Config();

    /* 3. Inicializar periféricos de hardware */
    MX_GPIO_Init();
    MX_TIM1_PWM_Init();
    MX_I2C1_Init();
    MX_I2C2_Init();
    MX_USART2_UART_Init();
    MX_TIM6_ControlTick_Init();

    /* 4. Inicializar arquitectura de control del Péndulo de Furuta y PIDs */
    Furuta_Init(&g_furuta);

    /* 5. Calibración inicial de cero con el péndulo colgado en reposo (Homing) */
    HAL_GPIO_WritePin(USER_LED_PORT, USER_LED_PIN, GPIO_PIN_SET);
    HAL_Delay(500);
    (void)AS5600_CalibrateZero(&g_furuta.enc_arm,  64U);
    (void)AS5600_CalibrateZero(&g_furuta.enc_pend, 64U);
    HAL_GPIO_WritePin(USER_LED_PORT, USER_LED_PIN, GPIO_PIN_RESET);

    /* Mensaje de bienvenida y diagnóstico de sensores I2C por USART2 */
    bool mag_arm_ok = false, mag_pend_ok = false;
    HAL_StatusTypeDef st_arm  = AS5600_CheckMagnet(&g_furuta.enc_arm,  &mag_arm_ok);
    HAL_StatusTypeDef st_pend = AS5600_CheckMagnet(&g_furuta.enc_pend, &mag_pend_ok);

    char diag_msg[512];
    int dlen = snprintf(diag_msg, sizeof(diag_msg),
        "\r\n==================================================================\r\n"
        " STM32 Nucleo-F446RE - Control Digital Pendulo Invertido de Furuta\r\n"
        " [I2C1 Motor   PB8/PB9 ] Status=%s | Reg0x0B=0x%02X | AGC=%u | Raw=%u\r\n"
        " [I2C2 Pendulo PB10/PB3] Status=%s | Reg0x0B=0x%02X | AGC=%u | Raw=%u\r\n"
        "==================================================================\r\n"
        "t_ms,state,pwm_pct,u_volts,theta_rad,omega_rad_s,alpha_rad,alpha_err_rad,alpha_dot_rad_s\r\n",
        (st_arm == HAL_OK) ? "OK" : "ERR_I2C",
        g_furuta.enc_arm.magnet_status, g_furuta.enc_arm.agc_value, g_furuta.enc_arm.raw_counts,
        (st_pend == HAL_OK) ? "OK" : "ERR_I2C",
        g_furuta.enc_pend.magnet_status, g_furuta.enc_pend.agc_value, g_furuta.enc_pend.raw_counts);
    if (dlen > 0) {
        (void)HAL_UART_Transmit(&huart2, (uint8_t *)diag_msg, (uint16_t)dlen, 200U);
    }

    /* 6. Arrancar interrupción periódica determinista de control a 1 kHz (TIM6) */
    (void)HAL_TIM_Base_Start_IT(&htim6);

    /* 7. Superloop de fondo (Prioridad baja: Telemetría UART, Comandos y Botón) */
    while (1) {
        /* A. Procesar recepción de comandos ASCII desde MATLAB / Python / Terminal */
        UART_PollCommandRX();

        /* B. Gestionar pulsación del botón azul B1 (PC13) con antirrebote */
        if (g_flag_user_btn_pressed) {
            g_flag_user_btn_pressed = false;
            HAL_Delay(40);
            if (HAL_GPIO_ReadPin(USER_BTN_PORT, USER_BTN_PIN) == GPIO_PIN_RESET) {
                if (g_furuta.state == FURUTA_STATE_IDLE ||
                    g_furuta.state == FURUTA_STATE_FAULT_STOP) {
                    /* Recalibrar cero en posición colgada y habilitar captura PID */
                    Furuta_ProcessCommand(&g_furuta, "ZERO");
                } else {
                    /* Parada inmediata si estaba en movimiento */
                    Furuta_ProcessCommand(&g_furuta, "STOP");
                }
            }
        }

        /* C. Enviar trama de telemetría CSV a 200 Hz (cada 5 ms) */
        if (g_flag_telemetry_ready) {
            g_flag_telemetry_ready = false;
            Furuta_SendTelemetry_UART(&g_furuta);

            /* Parpadeo indicador en LED LD2 (PA5) cuando el PID está estabilizando */
            if (g_furuta.state == FURUTA_STATE_BALANCE_PID) {
                HAL_GPIO_WritePin(USER_LED_PORT, USER_LED_PIN, GPIO_PIN_SET);
            } else {
                HAL_GPIO_TogglePin(USER_LED_PORT, USER_LED_PIN);
            }
        }
    }
}

/**
 * @brief Extrae bytes del ring buffer llenado por USART2_IRQHandler
 */
static void UART_PollCommandRX(void)
{
    while (g_uart_rx_tail != g_uart_rx_head) {
        char rx_byte = g_uart_rx_ring[g_uart_rx_tail];
        g_uart_rx_tail = (uint8_t)((g_uart_rx_tail + 1U) % 128U);

        if (rx_byte == '\n' || rx_byte == '\r') {
            if (s_uart_cmd_idx > 0U) {
                s_uart_cmd_buf[s_uart_cmd_idx] = '\0';
                Furuta_ProcessCommand(&g_furuta, s_uart_cmd_buf);
                s_uart_cmd_idx = 0U;
            }
        } else if (s_uart_cmd_idx < (sizeof(s_uart_cmd_buf) - 1U)) {
            s_uart_cmd_buf[s_uart_cmd_idx++] = rx_byte;
        }
    }
}

/**
 * @brief Configuración del Árbol de Relojes (Clock Tree) para STM32F446RE:
 *        - Fuente HSE Bypass (8 MHz desde MCO del ST-LINK)
 *        - PLL: PLLM = 4, PLLN = 180, PLLP = 2 -> SYSCLK = 180 MHz
 *        - HCLK = 180 MHz, APB1 = 45 MHz (Timers APB1 = 90 MHz),
 *          APB2 = 90 MHz (Timers APB2 = 180 MHz)
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_BYPASS;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 4;
    RCC_OscInitStruct.PLL.PLLN       = 180;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = 2;
    RCC_OscInitStruct.PLL.PLLR       = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;   /* PCLK1 = 45 MHz -> TIM6 = 90 MHz */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;   /* PCLK2 = 90 MHz -> TIM1 = 180 MHz */

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief Configuración de TIM1_CH1 (PA8) para PWM de 20.0 kHz
 *        f_PWM = 180 MHz / ((9 + 1) * (899 + 1)) = 20,000 Hz
 */
static void MX_TIM1_PWM_Init(void)
{
    __HAL_RCC_TIM1_CLK_ENABLE();

    TIM_OC_InitTypeDef sConfigOC = {0};

    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = TIM1_PRESCALER;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = TIM1_ARR_PERIOD;
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
        Error_Handler();
    }

    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = 0;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief Configuración de TIM6 para interrupción periódica de control a 1.0 kHz
 *        f_ISR = 90 MHz / ((89 + 1) * (999 + 1)) = 1,000 Hz (Ts = 1.0 ms)
 */
static void MX_TIM6_ControlTick_Init(void)
{
    __HAL_RCC_TIM6_CLK_ENABLE();

    htim6.Instance               = TIM6;
    htim6.Init.Prescaler         = TIM6_PRESCALER;
    htim6.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim6.Init.Period            = TIM6_ARR_PERIOD;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
        Error_Handler();
    }

    /* Prioridad alta (1) para garantizar determinismo temporal del muestreo Ts */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

/**
 * @brief Configuración de I2C1 (PB8=SCL1, PB9=SDA1) a 400 kHz Fast Mode
 *        Conectado al Sensor AS5600 Base (Ángulo del Brazo Theta)
 */
static void MX_I2C1_Init(void)
{
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_I2C1_FORCE_RESET();
    HAL_Delay(2);
    __HAL_RCC_I2C1_RELEASE_RESET();

    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 400000;
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief Configuración de I2C2 (PB10=SCL2, PB3=SDA2) a 400 kHz Fast Mode
 *        Conectado al Sensor AS5600 Distal (Ángulo del Péndulo Alfa)
 */
static void MX_I2C2_Init(void)
{
    __HAL_RCC_I2C2_CLK_ENABLE();
    __HAL_RCC_I2C2_FORCE_RESET();
    HAL_Delay(2);
    __HAL_RCC_I2C2_RELEASE_RESET();

    hi2c2.Instance             = I2C2;
    hi2c2.Init.ClockSpeed      = 400000;
    hi2c2.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1     = 0;
    hi2c2.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2     = 0;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief Configuración de USART2 (PA2=TX, PA3=RX) a 115200 bps 8N1
 */
static void MX_USART2_UART_Init(void)
{
    __HAL_RCC_USART2_CLK_ENABLE();

    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = UART_BAUDRATE;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }

    /* Habilitar interrupción RXNE en USART2 con máxima prioridad (0) (~0.3 us)
     * para que nunca pierda un byte a 115200 bps (86.8 us/byte) durante TIM6 */
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/**
 * @brief Configuración de todos los pines GPIO y funciones alternativas (AF)
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* 1. Pines de Dirección L298N: PB4 (IN1) y PB5 (IN2) -> Output Push-Pull */
    HAL_GPIO_WritePin(GPIOB, MOTOR_IN1_PIN | MOTOR_IN2_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = MOTOR_IN1_PIN | MOTOR_IN2_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 2. Pin PWM L298N: PA8 (TIM1_CH1 -> ENA) -> Alternate Function AF1 */
    GPIO_InitStruct.Pin       = MOTOR_PWM_TIM_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(MOTOR_PWM_TIM_PORT, &GPIO_InitStruct);

    /* 3. Pines I2C1 (PB8=D15 SCL1, PB9=D14 SDA1) e I2C2 (PB10=D6 SCL2, PB3=D3 SDA2) -> AF4 Open-Drain */
    GPIO_InitStruct.Pin       = ENC_ARM_SCL_PIN | ENC_ARM_SDA_PIN | ENC_PEND_SCL_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = ENC_PEND_SDA_PIN;       /* PB3 (Arduino D3) -> I2C2_SDA (AF4_I2C2) */
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 4. Pines USART2 (PA2=TX, PA3=RX) -> Alternate Function AF7 */
    GPIO_InitStruct.Pin       = UART_TX_PIN | UART_RX_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(UART_TELEMETRY_PORT, &GPIO_InitStruct);

    /* 5. LED Verde de Usuario LD2 (PA5) -> Output Push-Pull */
    HAL_GPIO_WritePin(USER_LED_PORT, USER_LED_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = USER_LED_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(USER_LED_PORT, &GPIO_InitStruct);

    /* 6. Botón Azul B1 (PC13) -> Interrupción por flanco de bajada */
    GPIO_InitStruct.Pin  = USER_BTN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(USER_BTN_PORT, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
 * @brief Manejador de errores críticos de hardware: apaga el motor de inmediato
 */
void Error_Handler(void)
{
    __disable_irq();
    HAL_GPIO_WritePin(GPIOB, MOTOR_IN1_PIN | MOTOR_IN2_PIN, GPIO_PIN_RESET);
    while (1) {
    }
}
