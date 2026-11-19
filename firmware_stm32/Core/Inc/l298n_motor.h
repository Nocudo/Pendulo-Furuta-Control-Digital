/**
 ******************************************************************************
 * @file    l298n_motor.h
 * @brief   Driver de etapa de potencia Puente H L298N para el motor DC con
 *          reductor planetario Namiki 22CL-3501PG.
 *          Incluye control PWM a 20 kHz (TIM1_CH1), control direccional
 *          (IN1=PB4, IN2=PB5), freno electromagnético y compensación inversa
 *          de zona muerta (Dead-Zone Compensation).
 ******************************************************************************
 */

#ifndef __L298N_MOTOR_H
#define __L298N_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* Parámetros Nominales del Actuador (Motor Namiki 22CL + Puente H L298N) ----*/
#define MOTOR_NOMINAL_VOLTAGE       12.0f   /* Tensión máxima efectiva (V) */
#define MOTOR_DEFAULT_DEADZONE_PCT  11.5f   /* Zona muerta típica L298N + fricción (%) */
#define MOTOR_MIN_CMD_THRESHOLD_V   0.03f   /* Umbral de corte para evitar zumbido (V) */

/**
 * @brief Estructura de configuración y estado del driver L298N
 */
typedef struct {
    TIM_HandleTypeDef *htim_pwm;    /* Timer PWM (htim1) */
    uint32_t pwm_channel;           /* Canal PWM (TIM_CHANNEL_1 -> PA8) */
    GPIO_TypeDef *in1_port;         /* Puerto GPIO para IN1 (GPIOB) */
    uint16_t in1_pin;               /* Pin GPIO para IN1 (GPIO_PIN_4) */
    GPIO_TypeDef *in2_port;         /* Puerto GPIO para IN2 (GPIOB) */
    uint16_t in2_pin;               /* Pin GPIO para IN2 (GPIO_PIN_5) */

    float v_supply_max;             /* Tensión máxima de escala (12.0 V) */
    float deadzone_pos_pct;         /* Zona muerta positiva medida (% PWM) */
    float deadzone_neg_pct;         /* Zona muerta negativa medida (% PWM) */
    bool enable_deadzone_comp;      /* Activar compensación inversa de zona muerta */
    bool invert_polarity;           /* Invertir polaridad lógica del motor */

    /* Telemetría del actuador en tiempo real */
    float current_voltage_cmd;      /* Tensión solicitada por el controlador (V) */
    float current_pwm_pct;          /* Ciclo útil efectivo aplicado (-100..+100 %) */
    uint32_t current_ccr_ticks;     /* Valor cargado en TIM1->CCR1 (0..900) */
} L298N_Handle_t;

/* Prototipos de Funciones Públicas ------------------------------------------*/
void L298N_Init(L298N_Handle_t *motor,
                TIM_HandleTypeDef *htim_pwm,
                uint32_t pwm_channel);

void L298N_SetDeadzone(L298N_Handle_t *motor,
                       float deadzone_pos_pct,
                       float deadzone_neg_pct,
                       bool enable_comp);

void L298N_SetRawPWM(L298N_Handle_t *motor, float pwm_percent);
void L298N_SetVoltage(L298N_Handle_t *motor, float voltage_v);
void L298N_Brake(L298N_Handle_t *motor);
void L298N_Coast(L298N_Handle_t *motor);

#ifdef __cplusplus
}
#endif

#endif /* __L298N_MOTOR_H */
