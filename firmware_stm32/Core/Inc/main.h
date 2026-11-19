/**
 ******************************************************************************
 * @file    main.h
 * @brief   Header principal y definición de pines para el Péndulo de Furuta
 *          en STM32 Nucleo-F446RE (ARM Cortex-M4F @ 180 MHz).
 * @project Control Digital — Péndulo Invertido Rotacional de Furuta
 ******************************************************************************
 */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes de HAL STM32F4 ---------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* Constantes Matemáticas ----------------------------------------------------*/
#ifndef M_PI
#define M_PI                    3.14159265358979323846f
#endif
#define TWO_PI                  6.28318530717958647692f
#define DEG_TO_RAD              0.01745329251994329577f
#define RAD_TO_DEG              57.2957795130823208768f

/* Parámetros Temporales del Lazo de Control Digital -------------------------*/
#define CONTROL_LOOP_FREQ_HZ    1000.0f                 /* Frecuencia ISR TIM6: 1 kHz */
#define TS_CONTROL_SEC          0.001f                  /* Periodo de muestreo Ts = 1.0 ms */
#define TELEMETRY_DECIMATION    10U                     /* Envío UART cada 10 ms (100 Hz limpios a 115200 bps) */

/* Configuración del Timer PWM (TIM1_CH1 @ 20 kHz, APB2 Timer Clock = 180 MHz) */
#define TIM1_PRESCALER          9U                      /* 180 MHz / (9 + 1) = 18 MHz */
#define TIM1_ARR_PERIOD         899U                    /* 18 MHz / (899 + 1) = 20.0 kHz */
#define PWM_MAX_DUTY            900U                    /* Cuentas para 100% Duty Cycle */

/* Configuración del Timer de Muestreo (TIM6 @ 1 kHz, APB1 Timer Clock = 90 MHz)*/
#define TIM6_PRESCALER          89U                     /* 90 MHz / (89 + 1) = 1 MHz */
#define TIM6_ARR_PERIOD         999U                    /* 1 MHz / (999 + 1) = 1000 Hz */

/* Mapeo de Pines Hardware STM32 Nucleo-F446RE (100% I2C, Cero Protoboard) ---
 * Total del sistema: 12 jumpers 1-a-1 directos a los headers hembra Arduino
 * (CN5, CN6, CN9) sin necesidad de protoboard ni empalmes en Y:
 *   - 4 jumpers para SENSOR 1 (AS5600 del Motor / Brazo Theta -> Bus I2C1)
 *   - 4 jumpers para SENSOR 2 (AS5600 del Péndulo Alfa        -> Bus I2C2)
 *   - 4 jumpers para DRIVER L298N (ENA, IN1, IN2, GND)
 * En ambos módulos AS5600 los pines OUT, DIR y GPO quedan LIBRES (sin jumper). */

/* 1. Puente H L298N — Agrupados en Header Arduino CN9 (D4, D5, D7) + GND en CN6-7 */
#define MOTOR_PWM_TIM_PORT      GPIOA
#define MOTOR_PWM_TIM_PIN       GPIO_PIN_8              /* PA8 (Arduino D7, CN9-8) -> TIM1_CH1 (L298N ENA) */

#define MOTOR_IN1_PORT          GPIOB
#define MOTOR_IN1_PIN           GPIO_PIN_4              /* PB4 (Arduino D5, CN9-6) -> GPIO Out (L298N IN1) */

#define MOTOR_IN2_PORT          GPIOB
#define MOTOR_IN2_PIN           GPIO_PIN_5              /* PB5 (Arduino D4, CN9-5) -> GPIO Out (L298N IN2) */
#define MOTOR_IN2_MIRROR_PORT   GPIOA
#define MOTOR_IN2_MIRROR_PIN    (GPIO_PIN_9 | GPIO_PIN_10) /* PA9 (D8) y PA10 (D2) -> Espejos redundantes de IN2 */

/* 2. SENSOR 1: AS5600 DEL MOTOR / BRAZO (Mide ángulo horizontal Theta - Bus I2C1)
 *    Ubicación física: Parte trasera del motor Namiki 22CL (dentro de la torre).
 *    Conexión en Nucleo:
 *      - SCL -> PB8 (Arduino D15 / SCL, CN5-10)
 *      - SDA -> PB9 (Arduino D14 / SDA, CN5-9)
 *      - GND -> GND (Arduino CN5-7, en la misma tira superior derecha)
 *      - VCC -> 3V3 (Arduino CN6-4, tira izquierda Power) */
#define ENC_ARM_I2C_PORT        GPIOB
#define ENC_ARM_SCL_PIN         GPIO_PIN_8              /* PB8 (Arduino D15) -> I2C1_SCL (AF4_I2C1) */
#define ENC_ARM_SDA_PIN         GPIO_PIN_9              /* PB9 (Arduino D14) -> I2C1_SDA (AF4_I2C1) */

/* 3. SENSOR 2: AS5600 DEL PÉNDULO (Mide ángulo vertical de la barra Alfa - Bus I2C2)
 *    Ubicación física: Extremo distal (punta/codo) del brazo horizontal junto al rodamiento 608ZZ.
 *    Conexión en Nucleo:
 *      - SCL -> PB10 (Arduino D6, CN9-7)
 *      - SDA -> PB3  (Arduino D3, CN9-4)
 *      - GND -> GND  (Arduino CN6-6, tira izquierda Power)
 *      - VCC -> IOREF/3V3 (Arduino CN6-2 = 3.3V directo, o Morpho CN7-16 = 3.3V) */
#define ENC_PEND_I2C_PORT       GPIOB
#define ENC_PEND_SCL_PIN        GPIO_PIN_10             /* PB10 (Arduino D6) -> I2C2_SCL (AF4_I2C2) */
#define ENC_PEND_SDA_PIN        GPIO_PIN_3              /* PB3  (Arduino D3) -> I2C2_SDA (AF4_I2C2) */

/* 4. Comunicación UART de Telemetría y Comandos (ST-LINK Virtual COM Port) */
#define UART_TELEMETRY_PORT     GPIOA
#define UART_TX_PIN             GPIO_PIN_2              /* PA2  -> USART2_TX (AF7) */
#define UART_RX_PIN             GPIO_PIN_3              /* PA3  -> USART2_RX (AF7) */
#define UART_BAUDRATE           115200U

/* 5. Interfaz de Usuario en Placa Nucleo-64 */
#define USER_BTN_PORT           GPIOC
#define USER_BTN_PIN            GPIO_PIN_13             /* PC13 -> Botón Azul B1 */
#define USER_LED_PORT           GPIOA
#define USER_LED_PIN            GPIO_PIN_5              /* PA5  -> LED Verde LD2 */

/* Handles Globales de Periféricos HAL ---------------------------------------*/
extern TIM_HandleTypeDef        htim1;
extern TIM_HandleTypeDef        htim6;
extern I2C_HandleTypeDef        hi2c1;
extern I2C_HandleTypeDef        hi2c2;
extern UART_HandleTypeDef       huart2;

/* Prototipos de Inicialización del Sistema ----------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
