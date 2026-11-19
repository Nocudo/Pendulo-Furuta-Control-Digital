/**
 ******************************************************************************
 * @file    l298n_motor.c
 * @brief   Implementación del driver de potencia Puente H L298N para el motor
 *          Namiki 22CL-3501PG.
 *          - PWM en PA8 (TIM1_CH1) a 20.0 kHz (ARR = 899, PSC = 9)
 *          - Dirección en PB4 (IN1) y PB5 (IN2)
 *          - Compensación no lineal inversa de zona muerta para linealizar
 *            la planta frente a la caída Darlington del L298N y fricción seca.
 ******************************************************************************
 */

#include "l298n_motor.h"

/**
 * @brief Inicializa el puente H L298N y arranca la generación PWM en TIM1_CH1
 */
void L298N_Init(L298N_Handle_t *motor,
                TIM_HandleTypeDef *htim_pwm,
                uint32_t pwm_channel)
{
    motor->htim_pwm             = htim_pwm;
    motor->pwm_channel          = pwm_channel;
    motor->in1_port             = MOTOR_IN1_PORT;
    motor->in1_pin              = MOTOR_IN1_PIN;
    motor->in2_port             = MOTOR_IN2_PORT;
    motor->in2_pin              = MOTOR_IN2_PIN;

    motor->v_supply_max         = MOTOR_NOMINAL_VOLTAGE;
    motor->deadzone_pos_pct     = MOTOR_DEFAULT_DEADZONE_PCT;
    motor->deadzone_neg_pct     = MOTOR_DEFAULT_DEADZONE_PCT;
    motor->enable_deadzone_comp = true;
    motor->invert_polarity      = false;

    motor->current_voltage_cmd  = 0.0f;
    motor->current_pwm_pct      = 0.0f;
    motor->current_ccr_ticks    = 0U;

    /* Asegurar motor detenido antes de habilitar el canal PWM */
    HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN2_MIRROR_PORT, MOTOR_IN2_MIRROR_PIN, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(motor->htim_pwm, motor->pwm_channel, 0U);
    (void)HAL_TIM_PWM_Start(motor->htim_pwm, motor->pwm_channel);
}

/**
 * @brief Configura los porcentajes de zona muerta positiva y negativa obtenidos
 *        en el Ensayo 1 de caracterización experimental
 */
void L298N_SetDeadzone(L298N_Handle_t *motor,
                       float deadzone_pos_pct,
                       float deadzone_neg_pct,
                       bool enable_comp)
{
    if (deadzone_pos_pct >= 0.0f && deadzone_pos_pct < 60.0f) {
        motor->deadzone_pos_pct = deadzone_pos_pct;
    }
    if (deadzone_neg_pct >= 0.0f && deadzone_neg_pct < 60.0f) {
        motor->deadzone_neg_pct = deadzone_neg_pct;
    }
    motor->enable_deadzone_comp = enable_comp;
}

/**
 * @brief Aplica un ciclo útil PWM directo en porcentaje [-100.0%, +100.0%]
 *        sin compensación de zona muerta (usado en ensayos de lazo abierto)
 */
void L298N_SetRawPWM(L298N_Handle_t *motor, float pwm_percent)
{
    if (motor->invert_polarity) {
        pwm_percent = -pwm_percent;
    }

    /* Saturación a [-100.0%, +100.0%] */
    if (pwm_percent > 100.0f) {
        pwm_percent = 100.0f;
    } else if (pwm_percent < -100.0f) {
        pwm_percent = -100.0f;
    }

    float abs_pwm = fabsf(pwm_percent);
    uint32_t ccr = (uint32_t)((abs_pwm / 100.0f) * (float)PWM_MAX_DUTY);
    if (ccr > PWM_MAX_DUTY) {
        ccr = PWM_MAX_DUTY;
    }

    if (pwm_percent > 0.05f) {
        /* Giro Positivo / Antihorario (IN1 = 1, IN2 = 0) */
        HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_IN2_MIRROR_PORT, MOTOR_IN2_MIRROR_PIN, GPIO_PIN_RESET);
    } else if (pwm_percent < -0.05f) {
        /* Giro Negativo / Horario (IN1 = 0, IN2 = 1 en D4/PB5 y espejo D8/PA9) */
        HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_IN2_MIRROR_PORT, MOTOR_IN2_MIRROR_PIN, GPIO_PIN_SET);
    } else {
        /* Zona neutra: desacople suave */
        HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_IN2_MIRROR_PORT, MOTOR_IN2_MIRROR_PIN, GPIO_PIN_RESET);
        ccr = 0U;
        pwm_percent = 0.0f;
    }

    __HAL_TIM_SET_COMPARE(motor->htim_pwm, motor->pwm_channel, ccr);

    motor->current_pwm_pct     = pwm_percent;
    motor->current_voltage_cmd = (pwm_percent / 100.0f) * motor->v_supply_max;
    motor->current_ccr_ticks   = ccr;
}

/**
 * @brief Aplica una tensión de control en voltios [-12.0 V, +12.0 V] con
 *        compensación inversa de zona muerta:
 *        PWM_eff = sign(u) * [ PWM_dz + (100 - PWM_dz) * (|u| / V_max) ]
 */
void L298N_SetVoltage(L298N_Handle_t *motor, float voltage_v)
{
    /* 1. Saturación a límites de fuente [-Vmax, +Vmax] */
    if (voltage_v > motor->v_supply_max) {
        voltage_v = motor->v_supply_max;
    } else if (voltage_v < -motor->v_supply_max) {
        voltage_v = -motor->v_supply_max;
    }

    motor->current_voltage_cmd = voltage_v;

    /* 2. Banda de silencio central para evitar conmutación de alta frecuencia en reposo */
    if (fabsf(voltage_v) < MOTOR_MIN_CMD_THRESHOLD_V) {
        L298N_SetRawPWM(motor, 0.0f);
        motor->current_voltage_cmd = 0.0f;
        return;
    }

    /* 3. Cálculo del porcentaje PWM con o sin compensación de zona muerta */
    float norm_mag = fabsf(voltage_v) / motor->v_supply_max; /* [0.0 .. 1.0] */
    float pwm_out_pct = 0.0f;

    if (motor->enable_deadzone_comp) {
        if (voltage_v > 0.0f) {
            float dz = motor->deadzone_pos_pct;
            pwm_out_pct = dz + ((100.0f - dz) * norm_mag);
        } else {
            float dz = motor->deadzone_neg_pct;
            pwm_out_pct = -(dz + ((100.0f - dz) * norm_mag));
        }
    } else {
        pwm_out_pct = (voltage_v / motor->v_supply_max) * 100.0f;
    }

    L298N_SetRawPWM(motor, pwm_out_pct);
    motor->current_voltage_cmd = voltage_v;
}

/**
 * @brief Freno electromagnético activo (cortocircuita terminales de armadura
 *        con IN1 = 1, IN2 = 1, ENA = 100%) — usado en el Ensayo 3 del péndulo
 */
void L298N_Brake(L298N_Handle_t *motor)
{
    HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_IN2_MIRROR_PORT, MOTOR_IN2_MIRROR_PIN, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(motor->htim_pwm, motor->pwm_channel, PWM_MAX_DUTY);

    motor->current_voltage_cmd = 0.0f;
    motor->current_pwm_pct     = 0.0f;
    motor->current_ccr_ticks   = PWM_MAX_DUTY;
}

/**
 * @brief Parada en rueda libre (alta impedancia en puente H: IN1 = 0, IN2 = 0, ENA = 0)
 */
void L298N_Coast(L298N_Handle_t *motor)
{
    HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN2_MIRROR_PORT, MOTOR_IN2_MIRROR_PIN, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(motor->htim_pwm, motor->pwm_channel, 0U);

    motor->current_voltage_cmd = 0.0f;
    motor->current_pwm_pct     = 0.0f;
    motor->current_ccr_ticks   = 0U;
}
